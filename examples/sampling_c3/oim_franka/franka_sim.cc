#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcm/lcm_messages.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/contact_results.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_franka/franka_full_sampling_c3plus.h"
#include "examples/sampling_c3/oim_franka/franka_process_common.h"
#include "systems/robot_lcm_systems.h"

DEFINE_string(config, "examples/sampling_c3/oim_franka/parameters/oim_franka.yaml",
              "Canonical OIM-T configuration");
DEFINE_double(duration, 0.0, "Seconds to simulate; zero performs a startup check");
DEFINE_bool(gravity_hold, true,
            "Apply the source model's gravcomp=1 semantics to the xArm joints");
DEFINE_bool(lcm, false, "Exchange robot command/state and T state over LCM");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");
DEFINE_double(object_start_yaw_override,
              std::numeric_limits<double>::quiet_NaN(),
              "Diagnostic initial T yaw override in radians");
DEFINE_string(robot_start_positions_override, "",
              "Diagnostic comma-separated controlled-joint start positions");
DEFINE_string(record_dir, "",
              "Optional new directory for renderer-compatible state telemetry");
DEFINE_double(record_period, 0.1,
              "Simulation seconds between renderer telemetry samples");

namespace {

class XarmStateSender final : public drake::systems::LeafSystem<double> {
 public:
  XarmStateSender(const drake::multibody::MultibodyPlant<double>& plant,
                  const OimTParams& params)
      : plant_(plant), params_(params) {
    this->DeclareVectorInputPort(
        "plant_state", drake::systems::BasicVector<double>(
                           plant.num_positions() + plant.num_velocities()));
    this->DeclareAbstractOutputPort("xarm_state", &XarmStateSender::Output);
  }

 private:
  void Output(const drake::systems::Context<double>& context,
              dairlib::lcmt_robot_output* message) const {
    const auto* state = this->EvalVectorInput(context, 0);
    message->utime = context.get_time() * 1e6;
    message->num_positions = message->num_velocities =
        message->num_efforts = params_.robot.controlled_joints.size();
    message->position_names = params_.robot.controlled_joints;
    message->velocity_names.resize(message->num_velocities);
    message->effort_names.resize(message->num_efforts);
    message->position.resize(message->num_positions);
    message->velocity.resize(message->num_velocities);
    message->effort.assign(message->num_efforts, 0.0);
    for (int i = 0; i < message->num_positions; ++i) {
      const auto& joint = plant_.GetJointByName(params_.robot.controlled_joints[i]);
      message->position[i] = state->GetAtIndex(joint.position_start());
      message->velocity[i] =
          state->GetAtIndex(plant_.num_positions() + joint.velocity_start());
      message->velocity_names[i] = params_.robot.controlled_joints[i] + "dot";
      // Use the plant's actual actuator name for the joint: MJCF-path
      // actuators are "<joint>_actuator", URDF transmissions carry vendor
      // motor names (e.g. panda_motor1).
      message->effort_names[i] =
          ActuatorNameForJoint(params_.robot.controlled_joints[i]);
    }
  }

  std::string ActuatorNameForJoint(const std::string& joint_name) const {
    for (drake::multibody::JointActuatorIndex index :
         plant_.GetJointActuatorIndices()) {
      const auto& actuator = plant_.get_joint_actuator(index);
      if (actuator.joint().name() == joint_name) return actuator.name();
    }
    throw std::runtime_error("no actuator drives joint " + joint_name);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const OimTParams params_;
};

class XarmObjectContactMonitor final
    : public drake::systems::LeafSystem<double> {
 public:
  XarmObjectContactMonitor(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::BodyIndex xarm_body,
      drake::multibody::BodyIndex object_body,
      const Eigen::Vector3d& end_effector_point,
      double pusher_radius,
      double contact_activation_tolerance,
      double period)
      : plant_(plant),
        xarm_body_(xarm_body),
        object_body_(object_body),
        end_effector_point_(end_effector_point),
        pusher_radius_(pusher_radius),
        contact_activation_tolerance_(contact_activation_tolerance),
        plant_context_(plant.CreateDefaultContext()) {
    this->DeclareAbstractInputPort(
        "contact_results",
        drake::Value<drake::multibody::ContactResults<double>>());
    this->DeclareVectorInputPort(
        "plant_state", drake::systems::BasicVector<double>(
                           plant.num_positions() + plant.num_velocities()));
    this->DeclarePeriodicPublishEvent(
        period, 0.0, &XarmObjectContactMonitor::Observe);
  }

  // Publish the live physical-contact receipt on every observation period so
  // the controller can join it to its selected-face transaction before any
  // dwell credit (Gate 381). An inactive receipt is still published: a fresh
  // "no contact" message is distinct from a dead channel.
  void EnableLcmPublishing(drake::lcm::DrakeLcm* lcm, std::string channel) {
    lcm_ = lcm;
    contact_channel_ = std::move(channel);
  }

  void PrintSummary() const {
    std::cout << "xarm_t_physical_contact_episodes=" << contact_episodes_
              << "\nxarm_t_physical_contact_samples=" << contact_samples_
              << "\nxarm_t_physical_contact_max_force_N=" << max_force_
              << "\nxarm_t_physical_contact_max_depth_m=" << max_depth_
              << "\nxarm_t_tip_side_candidate_samples="
              << class_counts_[0]
              << "\nxarm_t_side_shaft_samples=" << class_counts_[1]
              << "\nxarm_t_top_graze_samples=" << class_counts_[2]
              << "\nxarm_t_bottom_graze_samples=" << class_counts_[3]
              << std::endl;
  }

  void WriteTelemetry(const std::filesystem::path& path) const {
    std::ofstream stream(path);
    if (!stream) {
      throw std::runtime_error("failed to create contact telemetry " +
                               path.string());
    }
    stream << std::setprecision(17);
    for (const auto& receipt : telemetry_) {
      stream << "{\"sim_time\":" << receipt.sim_time
             << ",\"episode\":" << receipt.episode
             << ",\"force_N\":" << receipt.force
             << ",\"depth_m\":" << receipt.depth
             << ",\"point_W\":[" << receipt.point.x() << ','
             << receipt.point.y() << ',' << receipt.point.z() << ']'
             << ",\"force_on_object_W\":["
             << receipt.force_on_object.x() << ','
             << receipt.force_on_object.y() << ','
             << receipt.force_on_object.z() << ']'
             << ",\"classification\":\""
             << dairlib::oim::XarmPhysicalContactClassName(
                    receipt.classification.contact_class)
             << "\",\"relative_height_m\":"
             << receipt.classification.relative_height_m
             << ",\"point_to_tip_m\":"
             << receipt.classification.point_to_tip_m << "}\n";
    }
  }

 private:
  struct ContactTelemetry {
    double sim_time{};
    int episode{};
    double force{};
    double depth{};
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    Eigen::Vector3d force_on_object{Eigen::Vector3d::Zero()};
    dairlib::oim::XarmPhysicalContactClassification classification;
  };

  drake::systems::EventStatus Observe(
      const drake::systems::Context<double>& context) const {
    const auto& results =
        this->get_input_port(0)
            .Eval<drake::multibody::ContactResults<double>>(context);
    bool has_xarm_object_contact = false;
    double force = 0.0;
    double depth = 0.0;
    int contact_class_index = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_on_object = Eigen::Vector3d::Zero();
    for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
      const auto& info = results.point_pair_contact_info(i);
      const bool matching_pair =
          (info.bodyA_index() == xarm_body_ &&
           info.bodyB_index() == object_body_) ||
          (info.bodyA_index() == object_body_ &&
           info.bodyB_index() == xarm_body_);
      if (!matching_pair) continue;
      has_xarm_object_contact = true;
      const double candidate_force = info.contact_force().norm();
      if (candidate_force >= force) {
        force = candidate_force;
        depth = info.point_pair().depth;
        point = info.contact_point();
        // PointPairContactInfo reports force on body B. Normalize this
        // diagnostic vector to force on the manipulated object.
        force_on_object = info.bodyB_index() == object_body_
            ? info.contact_force()
            : -info.contact_force();
      }
    }
    if (has_xarm_object_contact) {
      const auto* state = this->EvalVectorInput(context, 1);
      plant_.SetPositionsAndVelocities(plant_context_.get(),
                                       state->get_value());
      const auto& xarm_body = plant_.get_body(xarm_body_);
      const auto& object_body = plant_.get_body(object_body_);
      const Eigen::Vector3d p_WTip =
          plant_.EvalBodyPoseInWorld(*plant_context_, xarm_body) *
          end_effector_point_;
      const Eigen::Vector3d p_WObject =
          plant_.EvalBodyPoseInWorld(*plant_context_, object_body).translation();
      const auto classification = dairlib::oim::ClassifyXarmPhysicalContact(
          point, p_WTip, p_WObject, pusher_radius_,
          contact_activation_tolerance_);
      ++contact_samples_;
      max_force_ = std::max(max_force_, force);
      max_depth_ = std::max(max_depth_, depth);
      last_contact_time_ = context.get_time();
      if (!in_contact_) {
        in_contact_ = true;
        ++contact_episodes_;
        std::cout << "xarm_t_physical_contact_begin=PASS episode="
                  << contact_episodes_
                  << " sim_time_s=" << context.get_time()
                  << " force_N=" << force
                  << " depth_m=" << depth
                  << " point_W=" << point.transpose()
                  << " class="
                  << dairlib::oim::XarmPhysicalContactClassName(
                         classification.contact_class)
                  << " relative_height_m="
                  << classification.relative_height_m
                  << " point_to_tip_m=" << classification.point_to_tip_m
                  << std::endl;
      }
      const int class_index =
          static_cast<int>(classification.contact_class);
      ++class_counts_.at(class_index);
      contact_class_index = class_index;
      telemetry_.push_back(ContactTelemetry{
          context.get_time(), contact_episodes_, force, depth, point,
          force_on_object, classification});
    } else if (in_contact_ &&
               context.get_time() - last_contact_time_ >= 0.05) {
      in_contact_ = false;
      std::cout << "xarm_t_physical_contact_end episode=" << contact_episodes_
                << " sim_time_s=" << context.get_time() << std::endl;
    }
    if (has_xarm_object_contact) {
      last_active_point_ = point;
      last_active_force_on_object_ = force_on_object;
      last_active_depth_ = depth;
      last_active_class_index_ = contact_class_index;
    }
    if (lcm_ != nullptr) {
      // contact_active carries the episode-level state (the same 0.05 s
      // end-debounce as the episode receipts) with the latest in-episode
      // observation retained: instantaneous 2 ms point-pair samples are
      // intermittent during real continuous contact, and a 20 ms consumer
      // sampling them raw would read a live push as repeated absence.
      // contact_sample_active preserves the instantaneous truth as a
      // diagnostic.
      const bool episode_active = in_contact_;
      dairlib::lcmt_robot_output receipt{};
      receipt.utime = static_cast<int64_t>(context.get_time() * 1.0e6);
      receipt.num_positions = 11;
      receipt.position_names = {
          "contact_active", "contact_sample_active", "contact_point_x",
          "contact_point_y", "contact_point_z", "contact_force_x",
          "contact_force_y", "contact_force_z", "penetration_depth",
          "contact_episode", "contact_class"};
      const Eigen::Vector3d receipt_point =
          episode_active ? last_active_point_ : Eigen::Vector3d::Zero();
      const Eigen::Vector3d receipt_force = episode_active
          ? last_active_force_on_object_ : Eigen::Vector3d::Zero();
      receipt.position = {
          episode_active ? 1.0 : 0.0,
          has_xarm_object_contact ? 1.0 : 0.0, receipt_point.x(),
          receipt_point.y(), receipt_point.z(), receipt_force.x(),
          receipt_force.y(), receipt_force.z(),
          episode_active ? last_active_depth_ : 0.0,
          static_cast<double>(contact_episodes_),
          static_cast<double>(
              episode_active ? last_active_class_index_ : -1)};
      receipt.num_velocities = 0;
      receipt.num_efforts = 0;
      receipt.imu_accel[0] = 0.0;
      receipt.imu_accel[1] = 0.0;
      receipt.imu_accel[2] = 0.0;
      drake::lcm::Publish(lcm_, contact_channel_, receipt);
    }
    return drake::systems::EventStatus::Succeeded();
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyIndex xarm_body_;
  const drake::multibody::BodyIndex object_body_;
  const Eigen::Vector3d end_effector_point_;
  const double pusher_radius_;
  const double contact_activation_tolerance_;
  mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;
  mutable bool in_contact_{false};
  mutable int contact_episodes_{0};
  mutable int contact_samples_{0};
  mutable std::array<int, 4> class_counts_{{0, 0, 0, 0}};
  mutable std::vector<ContactTelemetry> telemetry_;
  mutable double max_force_{0.0};
  mutable double max_depth_{0.0};
  mutable double last_contact_time_{-
      std::numeric_limits<double>::infinity()};
  mutable Eigen::Vector3d last_active_point_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d last_active_force_on_object_{
      Eigen::Vector3d::Zero()};
  mutable double last_active_depth_{0.0};
  mutable int last_active_class_index_{-1};
  drake::lcm::DrakeLcm* lcm_{nullptr};
  std::string contact_channel_;
};

}  // namespace

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = dairlib::oim::LoadAndValidateConfig(FLAGS_config);
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, params.simulation.time_step);
  drake::multibody::Parser parser(&plant, &scene_graph);
  const auto robot_models = parser.AddModels(params.robot.model);
  const auto object_models = parser.AddModels(params.object.model);
  if (robot_models.size() != 1 || object_models.size() != 1) {
    throw std::runtime_error("open_table requires one robot and one object model");
  }
  const drake::math::RigidTransformd X_WTable(
      Eigen::Vector3d(0.35, 0.0, -0.455));
  plant.RegisterCollisionGeometry(
      plant.world_body(), X_WTable, drake::geometry::Box(0.8, 1.523, 0.91),
      "open_table", drake::multibody::CoulombFriction<double>(0.3, 0.3));
  plant.RegisterVisualGeometry(
      plant.world_body(), X_WTable, drake::geometry::Box(0.8, 1.523, 0.91),
      "open_table_visual", Eigen::Vector4d(0.95, 0.95, 0.95, 1.0));
  dairlib::oim::AddXarmActuators(params, &plant);
  plant.Finalize();
  dairlib::oim::ValidateXarmPlant(plant, params);
  auto* contact_monitor = builder.AddSystem<XarmObjectContactMonitor>(
      plant,
      plant.GetBodyByName(params.robot.end_effector_body).index(),
      plant.GetBodyByName(params.object.body).index(),
      params.robot.end_effector_point,
      params.controller.pusher_radius,
      params.controller.contact_activation_tolerance,
      params.simulation.time_step);
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_monitor->get_input_port(0));
  builder.Connect(plant.get_state_output_port(),
                  contact_monitor->get_input_port(1));
  std::unique_ptr<drake::lcm::DrakeLcm> drake_lcm;
  if (FLAGS_lcm) {
    drake_lcm = std::make_unique<drake::lcm::DrakeLcm>(FLAGS_lcm_url);
    // Channel name is derived, not configured, so the canonical config
    // SHA-256 stays pinned.
    contact_monitor->EnableLcmPublishing(
        drake_lcm.get(), params.lcm.object_state_channel + "_CONTACT");
    auto* lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
        drake_lcm.get());
    auto* command_sub = builder.AddSystem(
        drake::systems::lcm::LcmSubscriberSystem::Make<
            dairlib::lcmt_robot_input>(params.lcm.robot_input_channel, lcm));
    auto* command_receiver =
        builder.AddSystem<dairlib::systems::RobotInputReceiver>(plant);
    auto* command_values =
        builder.AddSystem<dairlib::systems::SubvectorPassThrough<double>>(
            command_receiver->get_output_port().size(), 0,
            plant.num_actuators());
    builder.Connect(*command_sub, *command_receiver);
    builder.Connect(command_receiver->get_output_port(),
                    command_values->get_input_port());
    builder.Connect(command_values->get_output_port(),
                    plant.get_actuation_input_port());
    auto* state_sender = builder.AddSystem<XarmStateSender>(plant, params);
    auto* state_pub = builder.AddSystem(
        drake::systems::lcm::LcmPublisherSystem::Make<
            dairlib::lcmt_robot_output>(params.lcm.robot_state_channel, lcm,
            1.0 / params.simulation.robot_publish_rate));
    builder.Connect(plant.get_state_output_port(), state_sender->get_input_port());
    builder.Connect(state_sender->get_output_port(), state_pub->get_input_port());
    auto* object_sender = builder.AddSystem<dairlib::systems::ObjectStateSender>(
        plant, true, object_models[0]);
    auto* object_pub = builder.AddSystem(
        drake::systems::lcm::LcmPublisherSystem::Make<
            dairlib::lcmt_object_state>(params.lcm.object_state_channel, lcm,
            1.0 / params.simulation.object_publish_rate));
    builder.Connect(plant.get_state_output_port(object_models[0]),
                    object_sender->get_input_port_state());
    builder.Connect(object_sender->get_output_port(), object_pub->get_input_port());
  }
  if (params.simulation.visualize) drake::visualization::AddDefaultVisualization(&builder);
  drake::systems::VectorLogSink<double>* state_logger = nullptr;
  if (!FLAGS_record_dir.empty()) {
    if (!(FLAGS_record_period > 0.0)) {
      throw std::runtime_error("record_period must be positive");
    }
    state_logger = drake::systems::LogVectorOutput(
        plant.get_state_output_port(), &builder, FLAGS_record_period);
  }
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());
  dairlib::oim::SetXarmHome(params, plant, &plant_context);
  if (!FLAGS_robot_start_positions_override.empty()) {
    std::vector<double> positions;
    std::stringstream stream(FLAGS_robot_start_positions_override);
    std::string token;
    while (std::getline(stream, token, ',')) positions.push_back(std::stod(token));
    if (positions.size() != params.robot.controlled_joints.size()) {
      throw std::runtime_error(
          "robot_start_positions_override must contain one value per "
          "controlled joint");
    }
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
      plant.GetJointByName(params.robot.controlled_joints[i])
          .SetPositions(&plant_context,
                        Eigen::VectorXd::Constant(1, positions[i]));
    }
  }
  if (std::isfinite(FLAGS_object_start_yaw_override)) {
    const auto& block = plant.GetBodyByName(params.object.body);
    auto X_WBlock = plant.EvalBodyPoseInWorld(plant_context, block);
    X_WBlock.set_rotation(drake::math::RotationMatrixd::MakeZRotation(
        FLAGS_object_start_yaw_override));
    plant.SetFreeBodyPose(&plant_context, block, X_WBlock);
  }
  Eigen::VectorXd q0(params.robot.controlled_joints.size());
  for (int i = 0; i < q0.size(); ++i) {
    q0[i] = plant.GetJointByName(params.robot.controlled_joints[i])
                .GetPositions(plant_context)[0];
  }
  if (FLAGS_gravity_hold && !FLAGS_lcm) {
    const Eigen::MatrixXd B = plant.MakeActuationMatrix();
    const Eigen::VectorXd gravity =
        plant.CalcGravityGeneralizedForces(plant_context);
    plant.get_actuation_input_port().FixValue(&plant_context,
                                               -B.transpose() * gravity);
  }
  simulator.set_target_realtime_rate(params.simulation.realtime_rate);
  simulator.Initialize();
  if (FLAGS_duration > 0.0) simulator.AdvanceTo(FLAGS_duration);
  if (state_logger != nullptr) {
    const std::filesystem::path record_dir(FLAGS_record_dir);
    if (std::filesystem::exists(record_dir)) {
      throw std::runtime_error(
          "record_dir already exists; refusing to overwrite " +
          record_dir.string());
    }
    std::filesystem::create_directories(record_dir);
    std::ofstream summary(record_dir / "summary.json");
    std::ofstream steps(record_dir / "steps.jsonl");
    if (!summary || !steps) {
      throw std::runtime_error("failed to create renderer telemetry files");
    }
    summary << "{\n"
            << "  \"scene\": \"open_table\",\n"
            << "  \"source\": \"native C++ xarm6_sim\",\n"
            << "  \"config\": \"" << FLAGS_config << "\",\n"
            << "  \"duration\": " << FLAGS_duration << ",\n"
            << "  \"record_period\": " << FLAGS_record_period << ",\n"
            << "  \"contact_period\": "
            << params.simulation.time_step << ",\n"
            << "  \"pusher_radius\": "
            << params.controller.pusher_radius << ",\n"
            << "  \"contact_activation_tolerance\": "
            << params.controller.contact_activation_tolerance << "\n"
            << "}\n";
    steps << std::setprecision(17);
    const auto& log = state_logger->FindLog(simulator.get_context());
    const Eigen::VectorXd final_state =
        plant.GetPositionsAndVelocities(plant_context);
    const auto& block = plant.GetBodyByName(params.object.body);
    const auto& ee = plant.GetBodyByName(params.robot.end_effector_body);
    for (int sample = 0; sample < log.num_samples(); ++sample) {
      plant.SetPositionsAndVelocities(&plant_context, log.data().col(sample));
      const auto X_WBlock = plant.EvalBodyPoseInWorld(plant_context, block);
      const Eigen::Matrix3d R_WBlock = X_WBlock.rotation().matrix();
      const double yaw = std::atan2(R_WBlock(1, 0), R_WBlock(0, 0));
      const Eigen::Vector3d p_WBlock = X_WBlock.translation();
      const Eigen::Vector3d p_WTip =
          plant.EvalBodyPoseInWorld(plant_context, ee) *
          params.robot.end_effector_point;
      const auto X_WEe = plant.EvalBodyPoseInWorld(plant_context, ee);
      const Eigen::Vector3d p_WCapsuleStart =
          X_WEe * Eigen::Vector3d(
              0.0, 0.0, params.controller.pusher_radius);
      const Eigen::Vector3d p_WCapsuleEnd =
          X_WEe * Eigen::Vector3d(
              0.0, 0.0, params.robot.end_effector_point.z() -
                                params.controller.pusher_radius);
      const double position_error =
          (p_WBlock.head<2>() - params.object.goal_pose.head<2>()).norm();
      const double orientation_error = std::abs(std::remainder(
          params.object.goal_pose.z() - yaw, 6.28318530717958647692));
      steps << "{\"step\":" << sample
            << ",\"sim_time\":" << log.sample_times()[sample]
            << ",\"arm_positions\":[";
      for (int joint_index = 0;
           joint_index < static_cast<int>(params.robot.controlled_joints.size());
           ++joint_index) {
        if (joint_index > 0) steps << ',';
        steps << plant.GetJointByName(params.robot.controlled_joints[joint_index])
                     .GetPositions(plant_context)[0];
      }
      steps << "],\"object_pose\":[" << p_WBlock.x() << ','
            << p_WBlock.y() << ',' << yaw << "]"
            << ",\"object_position_W\":[" << p_WBlock.x() << ','
            << p_WBlock.y() << ',' << p_WBlock.z() << "]"
            << ",\"goal_pose\":[" << params.object.goal_pose.x() << ','
            << params.object.goal_pose.y() << ','
            << params.object.goal_pose.z() << "]"
            << ",\"tip_position\":[" << p_WTip.x() << ',' << p_WTip.y()
            << ',' << p_WTip.z() << "]"
            << ",\"capsule_start_W\":[" << p_WCapsuleStart.x() << ','
            << p_WCapsuleStart.y() << ',' << p_WCapsuleStart.z() << "]"
            << ",\"capsule_end_W\":[" << p_WCapsuleEnd.x() << ','
            << p_WCapsuleEnd.y() << ',' << p_WCapsuleEnd.z() << "]"
            << ",\"position_error\":" << position_error
            << ",\"orientation_error\":" << orientation_error << "}\n";
    }
    contact_monitor->WriteTelemetry(record_dir / "contacts.jsonl");
    plant.SetPositionsAndVelocities(&plant_context, final_state);
    std::cout << "renderer_telemetry_dir=" << record_dir
              << " samples=" << log.num_samples() << std::endl;
  }
  Eigen::VectorXd q1(q0.size());
  for (int i = 0; i < q1.size(); ++i) {
    q1[i] = plant.GetJointByName(params.robot.controlled_joints[i])
                .GetPositions(plant_context)[0];
  }
  const auto& ee = plant.GetBodyByName(params.robot.end_effector_body);
  const Eigen::Vector3d p_WTip =
      plant.EvalBodyPoseInWorld(plant_context, ee) *
      params.robot.end_effector_point;
  const auto X_WEe = plant.EvalBodyPoseInWorld(plant_context, ee);
  const Eigen::Vector3d p_WCapsuleStart =
      X_WEe * Eigen::Vector3d(
          0.0, 0.0, params.controller.pusher_radius);
  const Eigen::Vector3d p_WCapsuleEnd =
      X_WEe * Eigen::Vector3d(
          0.0, 0.0,
          params.robot.end_effector_point.z() -
              params.controller.pusher_radius);
  const auto& block = plant.GetBodyByName(params.object.body);
  const auto X_WBlock = plant.EvalBodyPoseInWorld(plant_context, block);
  const Eigen::Vector3d p_WBlock = X_WBlock.translation();
  const Eigen::Matrix3d R_WBlock = X_WBlock.rotation().matrix();
  const double block_yaw = std::atan2(R_WBlock(1, 0), R_WBlock(0, 0));
  const double translation_error =
      (p_WBlock.head<2>() - params.object.goal_pose.head<2>()).norm();
  const double orientation_error = std::abs(std::remainder(
      params.object.goal_pose.z() - block_yaw, 6.28318530717958647692));
  const double initial_yaw = std::isfinite(FLAGS_object_start_yaw_override)
      ? FLAGS_object_start_yaw_override
      : params.object.start_pose.z();
  const double initial_orientation_error = std::abs(std::remainder(
      params.object.goal_pose.z() - initial_yaw,
      6.28318530717958647692));
  const double yaw_error_reduction =
      initial_orientation_error - orientation_error;
  const double lateral_drift =
      std::abs(p_WBlock.x() - params.object.start_pose.x());
  const bool rotational_progress = yaw_error_reduction > 0.0;
  const bool lateral_drift_rejected =
      lateral_drift <= params.controller.lateral_drift_tolerance;
  const bool scenario_success =
      translation_error <= params.task.translation_tolerance &&
      orientation_error <= params.task.orientation_tolerance;
  std::cout << "xarm_joint_drift_inf=" << (q1 - q0).lpNorm<Eigen::Infinity>()
            << "\nxarm_joint_positions=" << q1.transpose()
            << "\nend_effector_point_W=" << p_WTip.transpose()
            << "\nend_effector_capsule_start_W="
            << p_WCapsuleStart.transpose()
            << "\nend_effector_capsule_end_W="
            << p_WCapsuleEnd.transpose()
            << "\nblock_position_W=" << p_WBlock.transpose() << std::endl;
  std::cout << "block_yaw_W=" << block_yaw
            << "\nobject_translation_error=" << translation_error
            << "\nobject_orientation_error=" << orientation_error
            << "\nobject_yaw_error_reduction=" << yaw_error_reduction
            << "\nobject_lateral_drift=" << lateral_drift
            << "\nrotational_progress="
            << (rotational_progress ? "PASS" : "FAIL")
            << "\nlateral_drift_rejection="
            << (lateral_drift_rejected ? "PASS" : "FAIL")
            << "\nopen_table_success=" << (scenario_success ? "PASS" : "FAIL")
            << std::endl;
  contact_monitor->PrintSummary();
  return 0;
}
