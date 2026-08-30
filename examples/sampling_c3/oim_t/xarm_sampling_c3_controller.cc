#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Geometry>

#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_object_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

#include "core/c3_plus.h"
#include "core/lcs.h"

#include "examples/sampling_c3/oim_t/xarm_process_common.h"
#include "lcm/lcm_trajectory.h"
#include "systems/framework/lcm_driven_loop.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");
DEFINE_double(smoke_offset_x, 0.0,
              "Publish a task-space smoke target this many meters in +x");
DEFINE_int32(publish_count, 100, "Number of trajectory messages to publish");
DEFINE_bool(first_solve_only, false,
            "Run one exact-T linearized Sampling-C3+ solve and exit");
DEFINE_bool(live_sampled_plan, false,
            "Sample exact-T boundary contacts from live state and publish");
DEFINE_int32(live_control_steps, 1,
             "Number of live sampled-plan control steps");
DEFINE_int32(live_step_period_ms, 500,
             "State refresh and target hold period for each live step");

namespace dairlib::oim {

struct ContactSolveResult {
  bool finite{};
  double elapsed{};
  double gap{};
  Eigen::Vector2d pusher_target;
  Eigen::Vector2d object_prediction;
};

ContactSolveResult SolveOneContact(const OimTParams& params,
                                   const Eigen::Vector2d& pusher,
                                   const Eigen::Vector2d& object,
                                   const Eigen::Vector2d& normal_W,
                                   const Eigen::Vector2d& boundary_W) {
  constexpr int n = 8, k = 2, m = 1, N = 5;
  const double dt = params.task.planning_time_step;
  const double pusher_mass = 1.0;
  const double object_mass = 0.1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n);
  A.block<4, 4>(0, 4) = dt * Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, k);
  B(4, 0) = dt / pusher_mass;
  B(5, 1) = dt / pusher_mass;
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, m);
  D.block<2, 1>(4, 0) = dt / pusher_mass * normal_W;
  D.block<2, 1>(6, 0) = -dt / object_mass * normal_W;
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(m, n);
  E.block<1, 2>(0, 0) = normal_W.transpose();
  E.block<1, 2>(0, 2) = -normal_W.transpose();
  // Small compliance keeps the one-contact projection well-conditioned while
  // retaining a stiff unilateral contact for this first linearized solve.
  Eigen::MatrixXd F = 1.0e-3 * Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, k);
  Eigen::VectorXd c(1);
  c[0] = -normal_W.dot(boundary_W) - 0.00555;
  c3::LCS lcs(A, B, D, Eigen::VectorXd::Zero(n), E, F, H, c, N, dt);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  x0.segment<2>(0) = pusher;
  x0.segment<2>(2) = object;
  Eigen::VectorXd xd = x0;
  xd.segment<2>(2) = params.object.goal_pose.head<2>();
  std::vector<Eigen::VectorXd> desired(N + 1, xd);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);
  Q.diagonal() << 0.01, 0.01, 200.0, 200.0, 1.0, 1.0, 0.05, 0.05;
  Eigen::MatrixXd R = 0.01 * Eigen::MatrixXd::Identity(k, k);
  const int nz = n + 2 * m + k;
  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(nz, nz);
  Eigen::MatrixXd U = Eigen::MatrixXd::Identity(nz, nz);
  c3::C3::CostMatrices costs(std::vector<Eigen::MatrixXd>(N + 1, Q),
                             std::vector<Eigen::MatrixXd>(N, R),
                             std::vector<Eigen::MatrixXd>(N, G),
                             std::vector<Eigen::MatrixXd>(N, U));
  c3::C3Options options;
  options.admm_iter = 3;
  options.num_threads = 1;
  options.warm_start = false;
  options.end_on_qp_step = true;
  options.scale_lcs = false;
  options.rho_scale = 3.0;
  options.gamma = 1.0;
  options.qp_projection_alpha = 0.01;
  options.qp_projection_scaling = 1.0;
  c3::C3Plus optimizer(lcs, costs, desired, options);
  const auto start = std::chrono::steady_clock::now();
  optimizer.Solve(x0);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  const auto states = optimizer.GetStateSolution();
  // This pinned C3+ revision stores N states (the terminal knot is omitted
  // from GetStateSolution even though the optimization contains it).
  const auto inputs = optimizer.GetInputSolution();
  bool finite = states.size() == N && inputs.size() == N;
  for (int i = 0; i < N && finite; ++i) finite = states[i].allFinite();
  for (const auto& input : inputs) finite = finite && input.allFinite();
  if (!finite) {
    std::cerr << "C3+ non-finite diagnostic:";
    for (std::size_t i = 0; i < states.size(); ++i) {
      std::cerr << " x" << i << "=" << states[i].transpose();
    }
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      std::cerr << " u" << i << "=" << inputs[i].transpose();
    }
    std::cerr << std::endl;
    return ContactSolveResult{};
  }
  return ContactSolveResult{true, elapsed, (E * x0 + c)[0],
                            states[std::min(2, N - 1)].segment<2>(0),
                            states[N - 1].segment<2>(2)};
}

void RunFirstC3PlusSolve(const OimTParams& params) {
  const auto result = SolveOneContact(
      params, Eigen::Vector2d(0.254967, -0.000911),
      params.object.start_pose.head<2>(), Eigen::Vector2d(0.0, -1.0),
      Eigen::Vector2d(0.0, -0.0794));
  if (!result.finite) throw std::runtime_error("first C3+ solve failed");
  std::cout << "first_c3plus_solve=PASS elapsed_s=" << result.elapsed
            << " final_stored_object_xy=" << result.object_prediction.transpose()
            << " contact_gap_m=" << result.gap << std::endl;
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = LoadAndValidateConfig(FLAGS_config);
  if (FLAGS_first_solve_only) {
    RunFirstC3PlusSolve(params);
    return 0;
  }
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ValidateXarmPlant(plant, params);
  auto context = plant.CreateDefaultContext();
  SetXarmHome(params, plant, context.get());
  const Eigen::Vector3d home_tip =
      plant.EvalBodyPoseInWorld(
          *context, plant.GetBodyByName(params.robot.end_effector_body)) *
      params.robot.end_effector_point;

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  systems::Subscriber<dairlib::lcmt_robot_output> state_subscriber(
      &lcm, params.lcm.robot_state_channel);
  systems::Subscriber<dairlib::lcmt_object_state> object_subscriber(
      &lcm, params.lcm.object_state_channel);
  while (state_subscriber.count() == 0 || object_subscriber.count() == 0) {
    lcm.HandleSubscriptions(100);
  }

  auto publish_target = [&](const Eigen::Vector3d& task_target,
                            int publish_count) {
    LcmTrajectory::Trajectory target;
    target.traj_name = "end_effector_position_target";
    target.datatypes = {"x", "y", "z"};
    target.time_vector.resize(1);
    target.time_vector[0] = state_subscriber.message().utime * 1e-6;
    target.datapoints.resize(3, 1);
    target.datapoints.col(0) = task_target;
    LcmTrajectory trajectory({target}, {target.traj_name}, target.traj_name,
                             "OIM xArm task-space target", false);
    dairlib::lcmt_timestamped_saved_traj message;
    message.utime = state_subscriber.message().utime;
    message.saved_traj = trajectory.GenerateLcmObject();
    for (int i = 0; i < publish_count; ++i) {
      drake::lcm::Publish(&lcm, params.lcm.tracking_trajectory_channel, message);
      lcm.HandleSubscriptions(10);
    }
  };

  auto solve_live_target = [&](int step_index) {
    const auto& robot_message = state_subscriber.message();
    for (int i = 0; i < robot_message.num_positions; ++i) {
      plant.GetJointByName(robot_message.position_names[i])
          .SetPositions(context.get(), Eigen::VectorXd::Constant(
                                          1, robot_message.position[i]));
    }
    const Eigen::Vector3d live_tip =
        plant.EvalBodyPoseInWorld(
            *context, plant.GetBodyByName(params.robot.end_effector_body)) *
        params.robot.end_effector_point;
    const auto& object_message = object_subscriber.message();
    auto position_value = [&](const std::string& name) {
      for (int i = 0; i < object_message.num_positions; ++i) {
        if (object_message.position_names[i] == name)
          return object_message.position[i];
      }
      throw std::runtime_error("live T state missing " + name);
    };
    const Eigen::Vector2d object_xy(position_value("block_x"),
                                    position_value("block_y"));
    const double qw = position_value("block_qw");
    const double qx = position_value("block_qx");
    const double qy = position_value("block_qy");
    const double qz = position_value("block_qz");
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz));
    const Eigen::Rotation2Dd R_WO(yaw);
    struct Sample { Eigen::Vector2d point, normal; const char* name; };
    const std::vector<Sample> samples = {
        {{-0.0445, 0.0198}, {0, 1}, "crossbar_top_left"},
        {{0.0, 0.0198}, {0, 1}, "crossbar_top"},
        {{0.0445, 0.0198}, {0, 1}, "crossbar_top_right"},
        {{-0.0445, 0.0099}, {-1, 0}, "crossbar_left"},
        {{0.0445, 0.0099}, {1, 0}, "crossbar_right"},
        {{-0.0099, -0.0397}, {-1, 0}, "stem_left"},
        {{0.0099, -0.0397}, {1, 0}, "stem_right"},
        {{0.0, -0.0794}, {0, -1}, "stem_bottom"}};
    ContactSolveResult best;
    double best_cost = std::numeric_limits<double>::infinity();
    const char* best_name = "none";
    for (const auto& sample : samples) {
      const auto result = SolveOneContact(
          params, live_tip.head<2>(), object_xy, R_WO * sample.normal,
          R_WO * sample.point);
      if (!result.finite) continue;
      const double cost =
          (result.object_prediction - params.object.goal_pose.head<2>())
              .squaredNorm();
      if (cost < best_cost) {
        best = result;
        best_cost = cost;
        best_name = sample.name;
      }
    }
    if (!best.finite) throw std::runtime_error("all live C3+ samples failed");
    Eigen::Vector2d step = best.pusher_target - live_tip.head<2>();
    if (step.norm() > params.controller.task_space_plan_step_limit) {
      step *= params.controller.task_space_plan_step_limit / step.norm();
    }
    Eigen::Vector3d task_target = live_tip;
    task_target.head<2>() = live_tip.head<2>() + step;
    task_target.z() = live_tip.z();
    std::cout << "live_sampled_c3plus=PASS step=" << step_index
              << " contact=" << best_name
              << " gap_m=" << best.gap << " target_W="
              << task_target.transpose() << std::endl;
    return task_target;
  };

  if (FLAGS_live_sampled_plan) {
    if (FLAGS_live_control_steps <= 0 || FLAGS_live_step_period_ms <= 0) {
      throw std::runtime_error("live step count and period must be positive");
    }
    const int publishes_per_step =
        std::max(1, FLAGS_live_step_period_ms / 10);
    for (int i = 0; i < FLAGS_live_control_steps; ++i) {
      publish_target(solve_live_target(i), publishes_per_step);
    }
  } else {
    publish_target(home_tip + Eigen::Vector3d(FLAGS_smoke_offset_x, 0.0, 0.0),
                   FLAGS_publish_count);
  }
  return 0;
}

}  // namespace dairlib::oim

int main(int argc, char* argv[]) { return dairlib::oim::DoMain(argc, argv); }
