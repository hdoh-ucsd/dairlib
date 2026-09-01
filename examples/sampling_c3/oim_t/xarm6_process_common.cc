#include "examples/sampling_c3/oim_t/xarm6_process_common.h"

#include <array>
#include <set>
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
  if (joints != 6) throw std::runtime_error("OIM xArm6 requires six joints");
  const std::array<std::string, 6> expected_joints = {
      "xarm6_joint1", "xarm6_joint2", "xarm6_joint3",
      "xarm6_joint4", "xarm6_joint5", "xarm6_joint6"};
  for (int i = 0; i < joints; ++i) {
    if (params.robot.controlled_joints[i] != expected_joints[i]) {
      throw std::runtime_error(
          "controlled_joints must contain ordered xarm6_joint1..6");
    }
  }
  if (std::set<std::string>(params.robot.controlled_joints.begin(),
                            params.robot.controlled_joints.end()).size() !=
      params.robot.controlled_joints.size()) {
    throw std::runtime_error("controlled_joints must be unique");
  }
  DemandSize(params.robot.home_positions, joints, "home_positions");
  DemandSize(params.robot.effort_limits, joints, "effort_limits");
  DemandSize(params.robot.velocity_limits, joints, "velocity_limits");
  DemandSize(params.robot.velocity_servo_gains, joints,
             "velocity_servo_gains");
  DemandSize(params.robot.passive_stiffness, joints, "passive_stiffness");
  if (!params.robot.home_positions.allFinite() ||
      !params.robot.effort_limits.allFinite() ||
      !params.robot.velocity_limits.allFinite() ||
      !params.robot.velocity_servo_gains.allFinite() ||
      !params.robot.passive_stiffness.allFinite() ||
      (params.robot.effort_limits.array() <= 0.0).any() ||
      (params.robot.velocity_limits.array() <= 0.0).any() ||
      (params.robot.velocity_servo_gains.array() <= 0.0).any() ||
      (params.robot.passive_stiffness.array() < 0.0).any()) {
    throw std::runtime_error("invalid xArm6 joint parameters");
  }
  if (params.scenario_name != "open_table") {
    throw std::runtime_error("native xArm6 processes require open_table");
  }
  if (params.simulation.time_step != params.task.execution_time_step) {
    throw std::runtime_error("simulation and execution time steps differ");
  }
  if (params.object.mass <= 0.0 || params.object.planar_moment_inertia <= 0.0) {
    throw std::runtime_error("object mass and planar inertia must be positive");
  }
  if (params.controller.task_space_plan_step_limit <= 0.0) {
    throw std::runtime_error("task_space_plan_step_limit must be positive");
  }
  if (params.controller.approach_command_step_limit <= 0.0) {
    throw std::runtime_error("approach_command_step_limit must be positive");
  }
  if (params.controller.descent_command_step_limit <= 0.0) {
    throw std::runtime_error("descent_command_step_limit must be positive");
  }
  if (params.controller.descent_posture_kp <= 0.0 ||
      params.controller.descent_posture_kd <= 0.0 ||
      params.controller.descent_posture_weight <= 0.0 ||
      params.controller.reposition_posture_kp <= 0.0 ||
      params.controller.reposition_posture_kd <= 0.0 ||
      params.controller.descent_diff_ik_centering_gain <= 0.0 ||
      params.controller.descent_diff_ik_max_velocity <= 0.0) {
    throw std::runtime_error(
        "posture and differential-IK parameters must be positive");
  }
  if (params.controller.pusher_radius <= 0.0 ||
      params.controller.approach_clearance < 0.0 ||
      params.controller.descent_clearance <=
          params.controller.approach_clearance ||
      params.controller.approach_planar_tolerance <= 0.0 ||
      params.controller.contact_activation_tolerance <= 0.0) {
    throw std::runtime_error(
        "invalid pusher approach/contact parameters (descent clearance must "
        "exceed approach clearance)");
  }
  if (params.controller.object_yaw_cost_weight <= 0.0 ||
      params.controller.yaw_selection_weight <= 0.0 ||
      params.controller.lateral_drift_weight <= 0.0 ||
      params.controller.lateral_drift_tolerance <= 0.0 ||
      params.controller.minimum_contact_normal_step <= 0.0 ||
      params.controller.minimum_contact_normal_step >
          params.controller.task_space_plan_step_limit) {
    throw std::runtime_error("invalid rotational/lateral planning parameters");
  }
  if (params.controller.reposition_speed <= 0.0 ||
      params.controller.reposition_waypoint_height <=
          params.object.resting_height ||
      params.controller.successor_minimum_contact_steps <= 0 ||
      params.controller.successor_progress_window_steps <
          params.controller.successor_minimum_contact_steps ||
      params.controller.successor_minimum_yaw_progress <= 0.0 ||
      params.controller.successor_minimum_translation_progress <= 0.0 ||
      params.controller.successor_cost_hysteresis_fraction < 0.0 ||
      params.controller.successor_cost_hysteresis_fraction >= 1.0) {
    throw std::runtime_error("invalid successor-face repositioning parameters");
  }
  const auto& full = params.full_sampling_c3plus;
  if (full.contact_model != "anitescu" || full.horizon <= 0 ||
      full.admm_iterations <= 0 || full.num_threads <= 0 ||
      full.delta_option != 1 ||
      full.rho_scale <= 0.0 || full.gamma <= 0.0 ||
      full.qp_projection_alpha <= 0.0 ||
      full.qp_projection_scaling <= 0.0 ||
      full.num_friction_directions < 2 ||
      full.pusher_ground_friction < 0.0 ||
      full.pusher_object_friction < 0.0 ||
      full.object_ground_friction < 0.0 ||
      full.state_cost_diagonal.size() != 19 ||
      full.input_cost_diagonal.size() != 3 ||
      (full.state_cost_diagonal.array() < 0.0).any() ||
      (full.input_cost_diagonal.array() < 0.0).any() ||
      full.state_cost_scale <= 0.0 || full.input_cost_scale <= 0.0 ||
      full.consensus_cost_scale <= 0.0 ||
      full.projection_cost_scale <= 0.0 ||
      full.consensus_lambda_weight <= 0.0 ||
      full.consensus_eta_weight <= 0.0 ||
      full.projection_lambda_weight <= 0.0 ||
      full.projection_eta_weight <= 0.0 ||
      full.dynamics_residual_tolerance <= 0.0 ||
      full.equality_residual_tolerance <= 0.0 ||
      full.nonnegative_residual_tolerance <= 0.0 ||
      full.complementarity_residual_tolerance <= 0.0 ||
      full.consensus_residual_tolerance <= 0.0) {
    throw std::runtime_error("invalid full Sampling-C3+ parameters");
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
    const auto& joint = plant.GetJointByName(name);
    if (joint.num_positions() != 1 || joint.num_velocities() != 1 ||
        dynamic_cast<const drake::multibody::RevoluteJoint<double>*>(
            &joint) == nullptr) {
      throw std::runtime_error(
          "xArm6 controlled joints must be one-DOF revolute joints");
    }
  }
  if (!plant.HasBodyNamed(params.robot.end_effector_body)) {
    throw std::runtime_error("xArm model is missing end-effector body " +
                             params.robot.end_effector_body);
  }
  if (plant.num_actuators() !=
      static_cast<int>(params.robot.controlled_joints.size())) {
    throw std::runtime_error(
        "xArm model actuator count must match the configured six joints");
  }
  for (int i = 0; i < plant.num_actuators(); ++i) {
    const auto& actuator = plant.GetJointActuatorByName(
        params.robot.controlled_joints[i] + "_actuator");
    if (actuator.joint().name() != params.robot.controlled_joints[i]) {
      throw std::runtime_error("xArm actuator-to-joint mapping mismatch");
    }
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
