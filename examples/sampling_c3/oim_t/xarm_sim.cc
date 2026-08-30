#include <iostream>

#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm_process_common.h"
#include "systems/robot_lcm_systems.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_double(duration, 0.0, "Seconds to simulate; zero performs a startup check");
DEFINE_bool(gravity_hold, true,
            "Apply the source model's gravcomp=1 semantics to the xArm joints");
DEFINE_bool(lcm, false, "Exchange robot command/state and T state over LCM");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");

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
      message->effort_names[i] = params_.robot.controlled_joints[i] + "_actuator";
    }
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  const OimTParams params_;
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
  std::unique_ptr<drake::lcm::DrakeLcm> drake_lcm;
  if (FLAGS_lcm) {
    drake_lcm = std::make_unique<drake::lcm::DrakeLcm>(FLAGS_lcm_url);
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
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());
  dairlib::oim::SetXarmHome(params, plant, &plant_context);
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
  Eigen::VectorXd q1(q0.size());
  for (int i = 0; i < q1.size(); ++i) {
    q1[i] = plant.GetJointByName(params.robot.controlled_joints[i])
                .GetPositions(plant_context)[0];
  }
  const auto& ee = plant.GetBodyByName(params.robot.end_effector_body);
  const Eigen::Vector3d p_WTip =
      plant.EvalBodyPoseInWorld(plant_context, ee) *
      params.robot.end_effector_point;
  const auto& block = plant.GetBodyByName(params.object.body);
  const Eigen::Vector3d p_WBlock =
      plant.EvalBodyPoseInWorld(plant_context, block).translation();
  std::cout << "xarm_joint_drift_inf=" << (q1 - q0).lpNorm<Eigen::Infinity>()
            << "\nend_effector_point_W=" << p_WTip.transpose()
            << "\nblock_position_W=" << p_WBlock.transpose() << std::endl;
  return 0;
}
