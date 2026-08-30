#include "examples/sampling_c3/oim_t/xarm_process_common.h"

#include <stdexcept>

#include <drake/common/yaml/yaml_io.h>

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
  if (params.simulation.time_step != params.task.execution_time_step) {
    throw std::runtime_error("simulation and execution time steps differ");
  }
  return params;
}

void AddXarmActuators(const OimTParams& params,
                      drake::multibody::MultibodyPlant<double>* plant) {
  // Drake's pinned MJCF parser does not import MuJoCo velocity actuators.
  // Recreate one torque input per controlled joint; the controller remains
  // responsible for enforcing the YAML effort and velocity limits.
  for (const std::string& name : params.robot.controlled_joints) {
    plant->AddJointActuator(name + "_actuator", plant->GetJointByName(name));
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
