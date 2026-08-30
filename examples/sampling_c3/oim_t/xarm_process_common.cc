#include "examples/sampling_c3/oim_t/xarm_process_common.h"

#include <stdexcept>

#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/revolute_spring.h>

namespace dairlib::oim {
namespace {

void DemandSize(const Eigen::VectorXd& value, int expected,
                const std::string& name) {
  if (value.size() != expected) {
    throw std::runtime_error(name + " must contain " +
                             std::to_string(expected) + " values");
  }
}

}  // namespace

OimTParams LoadAndValidateConfig(const std::string& path) {
  OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(path);
  const int joints = params.robot.controlled_joints.size();
  if (joints != 5) throw std::runtime_error("OIM xArm requires five joints");
  DemandSize(params.robot.home_positions, joints, "home_positions");
  DemandSize(params.robot.effort_limits, joints, "effort_limits");
  DemandSize(params.robot.velocity_limits, joints, "velocity_limits");
  DemandSize(params.robot.velocity_servo_gains, joints,
             "velocity_servo_gains");
  DemandSize(params.robot.passive_stiffness, joints, "passive_stiffness");
  if (params.simulation.time_step != params.task.execution_time_step) {
    throw std::runtime_error("simulation and execution time steps differ");
  }
  if (params.controller.task_space_plan_step_limit <= 0.0) {
    throw std::runtime_error("task_space_plan_step_limit must be positive");
  }
  if (params.controller.pusher_radius <= 0.0 ||
      params.controller.approach_clearance < 0.0 ||
      params.controller.contact_activation_tolerance <= 0.0) {
    throw std::runtime_error("invalid pusher approach/contact parameters");
  }
  return params;
}

void AddXarmActuators(const OimTParams& params,
                      drake::multibody::MultibodyPlant<double>* plant) {
  // Drake's pinned MJCF parser does not import MuJoCo velocity actuators.
  // Recreate one torque input per controlled joint; the controller remains
  // responsible for enforcing the YAML effort and velocity limits.
  const int num_joints = params.robot.controlled_joints.size();
  for (int i = 0; i < num_joints; ++i) {
    const std::string& name = params.robot.controlled_joints[i];
    auto& joint = plant->GetMutableJointByName<
        drake::multibody::RevoluteJoint>(name);
    joint.set_velocity_limits(
        Eigen::VectorXd::Constant(1, -params.robot.velocity_limits[i]),
        Eigen::VectorXd::Constant(1, params.robot.velocity_limits[i]));
    plant->AddJointActuator(name + "_actuator", joint,
                            params.robot.effort_limits[i]);
  }
  for (int i = 0; i < params.robot.passive_stiffness.size(); ++i) {
    if (params.robot.passive_stiffness[i] == 0.0) continue;
    const auto& joint = dynamic_cast<
        const drake::multibody::RevoluteJoint<double>&>(
        plant->GetJointByName(params.robot.controlled_joints[i]));
    plant->AddForceElement<drake::multibody::RevoluteSpring>(
        joint, 0.0, params.robot.passive_stiffness[i]);
  }
}

void ValidateXarmPlant(const drake::multibody::MultibodyPlant<double>& plant,
                       const OimTParams& params) {
  for (const std::string& name : params.robot.controlled_joints) {
    if (!plant.HasJointNamed(name)) {
      throw std::runtime_error("xArm model is missing joint " + name);
    }
  }
  if (!plant.HasBodyNamed(params.robot.end_effector_body)) {
    throw std::runtime_error("xArm model is missing end-effector body " +
                             params.robot.end_effector_body);
  }
  if (plant.num_actuators() !=
      static_cast<int>(params.robot.controlled_joints.size())) {
    throw std::runtime_error("xArm model must expose exactly five actuators");
  }
  for (int i = 0; i < plant.num_actuators(); ++i) {
    const auto& actuator = plant.GetJointActuatorByName(
        params.robot.controlled_joints[i] + "_actuator");
    if (actuator.effort_limit() !=
        params.robot.effort_limits[i]) {
      throw std::runtime_error("xArm actuator effort limit mismatch");
    }
  }
}

void SetXarmHome(const OimTParams& params,
                 const drake::multibody::MultibodyPlant<double>& plant,
                 drake::systems::Context<double>* context) {
  for (int i = 0; i < params.robot.home_positions.size(); ++i) {
    const auto& joint = plant.GetJointByName(params.robot.controlled_joints[i]);
    joint.SetPositions(context,
                       (Eigen::VectorXd(1) << params.robot.home_positions[i]).finished());
  }
}

}  // namespace dairlib::oim
