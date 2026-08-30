#include <algorithm>
#include <memory>

#include <dairlib/lcmt_robot_input.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm_process_common.h"
#include "lcm/lcm_trajectory.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");

namespace dairlib::oim {

class SafeTaskTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  SafeTaskTrajectorySource(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()),
        end_effector_(plant.GetBodyByName(params.robot.end_effector_body)),
        point_(params.robot.end_effector_point) {
    state_port_ = this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators())).get_index();
    message_port_ = this->DeclareAbstractInputPort(
        "task_space_trajectory",
        drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();
    drake::trajectories::PiecewisePolynomial<double> initial(
        Eigen::Vector3d::Zero());
    drake::trajectories::Trajectory<double>& model = initial;
    output_port_ = this->DeclareAbstractOutputPort(
        "safe_end_effector_position_target", model,
        &SafeTaskTrajectorySource::CalcTrajectory).get_index();
  }

  const drake::systems::OutputPort<double>& trajectory_output() const {
    return this->get_output_port(output_port_);
  }

 private:
  void CalcTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const {
    auto* result = dynamic_cast<
        drake::trajectories::PiecewisePolynomial<double>*>(output);
    const auto* message =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, message_port_);
    if (message != nullptr && message->utime > 0 &&
        message->saved_traj.num_trajectories > 0) {
      const LcmTrajectory trajectories(message->saved_traj);
      const auto& target =
          trajectories.GetTrajectory("end_effector_position_target");
      if (target.datapoints.rows() != 3 || target.datapoints.cols() < 2) {
        throw std::runtime_error("OSC target must contain at least two 3D samples");
      }
      *result = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          target.time_vector, target.datapoints);
      return;
    }
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, state_port_));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    const Eigen::Vector3d hold =
        plant_.EvalBodyPoseInWorld(*context_, end_effector_) * point_;
    *result = drake::trajectories::PiecewisePolynomial<double>(hold);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::RigidBody<double>& end_effector_;
  Eigen::Vector3d point_;
  drake::systems::InputPortIndex state_port_, message_port_;
  drake::systems::OutputPortIndex output_port_;
};

class XarmVelocityServoBridge final
    : public drake::systems::LeafSystem<double> {
 public:
  XarmVelocityServoBridge(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()),
        effort_limits_(params.robot.effort_limits),
        velocity_limits_(params.robot.velocity_limits),
        servo_gains_(params.robot.velocity_servo_gains) {
    this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators()));
    this->DeclareVectorInputPort(
        "osc_torque", systems::TimestampedVector<double>(plant.num_actuators()));
    this->DeclareVectorOutputPort(
        "xarm_torque", systems::TimestampedVector<double>(plant.num_actuators()),
        &XarmVelocityServoBridge::CalcTorque);
  }

 private:
  void CalcTorque(const drake::systems::Context<double>& context,
                  systems::TimestampedVector<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, 0));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    const Eigen::VectorXd gravity_compensation =
        -plant_.MakeActuationMatrix().transpose() *
        plant_.CalcGravityGeneralizedForces(*context_);
    const auto* osc_torque =
        dynamic_cast<const systems::TimestampedVector<double>*>(
            this->EvalVectorInput(context, 1));
    // DAIRLab OSC returns total inverse-dynamics torque. Remove its gravity
    // component before passing through the source actuator clamp, then add
    // gravity back outside that clamp to reproduce MuJoCo gravcomp ordering.
    const Eigen::VectorXd desired_servo_torque =
        osc_torque->get_data() - gravity_compensation;

    // Drake imports this MJCF actuator as a torque input. Reconstruct the
    // source MuJoCo velocity actuator exactly around the upstream desired
    // servo torque:
    //   qdot_cmd = clamp(qdot + tau_desired / kv, +/- qdot_limit)
    //   tau_servo = clamp(kv * (qdot_cmd - qdot), +/- effort_limit)
    // Gravity compensation is a passive MuJoCo body force and therefore sits
    // outside the actuator force clamp. Joint-4 stiffness is already a Drake
    // passive force element and is intentionally not added here.
    const Eigen::VectorXd velocity = plant_.GetVelocities(*context_);
    Eigen::VectorXd servo_torque(plant_.num_actuators());
    for (int i = 0; i < servo_torque.size(); ++i) {
      const double velocity_command = std::clamp(
          velocity[i] + desired_servo_torque[i] / servo_gains_[i],
          -velocity_limits_[i], velocity_limits_[i]);
      servo_torque[i] = std::clamp(
          servo_gains_[i] * (velocity_command - velocity[i]),
          -effort_limits_[i], effort_limits_[i]);
    }
    const Eigen::VectorXd torque = servo_torque + gravity_compensation;
    output->SetDataVector(torque);
    output->set_timestamp(state->get_timestamp());
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  Eigen::VectorXd effort_limits_;
  Eigen::VectorXd velocity_limits_;
  Eigen::VectorXd servo_gains_;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = LoadAndValidateConfig(FLAGS_config);
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ValidateXarmPlant(plant, params);
  auto plant_context = plant.CreateDefaultContext();

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  drake::systems::DiagramBuilder<double> builder;
  auto* state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          dairlib::lcmt_robot_output>(params.lcm.robot_state_channel, &lcm));
  auto* state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto* trajectory_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          dairlib::lcmt_timestamped_saved_traj>(
          params.lcm.tracking_trajectory_channel, &lcm));
  auto* trajectory_source =
      builder.AddSystem<SafeTaskTrajectorySource>(plant, params);
  auto* osc =
      builder.AddSystem<systems::controllers::OperationalSpaceControl>(
          plant, plant_context.get(), false);
  auto tracking = std::make_unique<
      systems::controllers::TransTaskSpaceTrackingData>(
      "end_effector_position_target",
      params.controller.task_space_kp.asDiagonal(),
      params.controller.task_space_kd.asDiagonal(),
      Eigen::Matrix3d::Identity(), plant, plant);
  tracking->AddPointToTrack(params.robot.end_effector_body,
                            params.robot.end_effector_point);
  osc->AddTrackingData(std::move(tracking));
  // Preserve DAIRLab's shared Sampling-C3 acceleration regularization:
  // w_accel=1e-5 multiplied by W_accel=0.01 I.
  osc->SetAccelerationCostWeights(
      1.0e-7 * Eigen::MatrixXd::Identity(plant.num_velocities(),
                                         plant.num_velocities()));
  osc->Build();
  auto* servo_bridge =
      builder.AddSystem<XarmVelocityServoBridge>(plant, params);
  auto* command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto* command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          params.lcm.robot_input_channel, &lcm,
          drake::systems::TriggerTypeSet({drake::systems::TriggerType::kForced})));
  builder.Connect(*state_sub, *state_receiver);
  builder.Connect(trajectory_sub->get_output_port(),
                  trajectory_source->get_input_port(1));
  builder.Connect(state_receiver->get_output_port(),
                  trajectory_source->get_input_port(0));
  builder.Connect(trajectory_source->trajectory_output(),
                  osc->get_input_port_tracking_data(
                      "end_effector_position_target"));
  builder.Connect(state_receiver->get_output_port(),
                  osc->get_input_port_robot_output());
  builder.Connect(state_receiver->get_output_port(),
                  servo_bridge->get_input_port(0));
  builder.Connect(osc->get_output_port_osc_command(),
                  servo_bridge->get_input_port(1));
  builder.Connect(servo_bridge->get_output_port(),
                  command_sender->get_input_port());
  builder.Connect(command_sender->get_output_port(), command_pub->get_input_port());

  std::shared_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
  diagram->set_name("oim_xarm_dairlab_osc_controller");
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, diagram, state_receiver, params.lcm.robot_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib::oim

int main(int argc, char* argv[]) { return dairlib::oim::DoMain(argc, argv); }
