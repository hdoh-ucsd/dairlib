#include <chrono>
#include <iostream>
#include <thread>

#include <dairlib/lcmt_robot_output.hpp>
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

namespace dairlib::oim {

void RunFirstC3PlusSolve(const OimTParams& params) {
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
  D(5, 0) = -dt / pusher_mass;
  D(7, 0) = dt / object_mass;
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(m, n);
  E(0, 1) = -1.0;
  E(0, 3) = 1.0;
  // Small compliance keeps the one-contact projection well-conditioned while
  // retaining a stiff unilateral contact for this first linearized solve.
  Eigen::MatrixXd F = 1.0e-3 * Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, k);
  // Exact OIM T minimum-y footprint (-79.4 mm) plus the 5.55 mm pusher radius.
  Eigen::VectorXd c(1);
  c[0] = -0.0794 - 0.00555;
  c3::LCS lcs(A, B, D, Eigen::VectorXd::Zero(n), E, F, H, c, N, dt);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  x0.segment<2>(0) << 0.254967, -0.000911;
  x0.segment<2>(2) = params.object.start_pose.head<2>();
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
    for (int i = 0; i < states.size(); ++i) {
      std::cerr << " x" << i << "=" << states[i].transpose();
    }
    for (int i = 0; i < inputs.size(); ++i) {
      std::cerr << " u" << i << "=" << inputs[i].transpose();
    }
    std::cerr << std::endl;
    throw std::runtime_error("first Sampling-C3+ solution is not finite");
  }
  std::cout << "first_c3plus_solve=PASS elapsed_s=" << elapsed
            << " final_stored_object_xy="
            << states[N - 1].segment<2>(2).transpose()
            << " contact_gap_m=" << (E * x0 + c)[0] << std::endl;
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
  while (state_subscriber.count() == 0) lcm.HandleSubscriptions(100);

  LcmTrajectory::Trajectory target;
  target.traj_name = "end_effector_position_target";
  target.datatypes = {"x", "y", "z"};
  target.time_vector.resize(1);
  target.time_vector[0] = state_subscriber.message().utime * 1e-6;
  target.datapoints.resize(3, 1);
  target.datapoints.col(0) =
      home_tip + Eigen::Vector3d(FLAGS_smoke_offset_x, 0.0, 0.0);
  LcmTrajectory trajectory({target}, {target.traj_name}, target.traj_name,
                           "OIM xArm task-space target", false);
  dairlib::lcmt_timestamped_saved_traj message;
  message.utime = state_subscriber.message().utime;
  message.saved_traj = trajectory.GenerateLcmObject();
  for (int i = 0; i < FLAGS_publish_count; ++i) {
    drake::lcm::Publish(&lcm, params.lcm.tracking_trajectory_channel, message);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}

}  // namespace dairlib::oim

int main(int argc, char* argv[]) { return dairlib::oim::DoMain(argc, argv); }
