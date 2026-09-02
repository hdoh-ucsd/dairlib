#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/yaml/yaml_read_archive.h>

struct OimXarmRobotParams {
  std::string model, model_instance, end_effector_body;
  Eigen::Vector3d end_effector_point;
  std::vector<std::string> controlled_joints;
  Eigen::VectorXd home_positions, effort_limits, velocity_limits;
  Eigen::VectorXd velocity_servo_gains;
  Eigen::VectorXd passive_stiffness;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(model)); a->Visit(DRAKE_NVP(model_instance));
    a->Visit(DRAKE_NVP(end_effector_body));
    a->Visit(DRAKE_NVP(end_effector_point));
    a->Visit(DRAKE_NVP(controlled_joints));
    a->Visit(DRAKE_NVP(home_positions)); a->Visit(DRAKE_NVP(effort_limits));
    a->Visit(DRAKE_NVP(velocity_limits));
    a->Visit(DRAKE_NVP(velocity_servo_gains));
    a->Visit(DRAKE_NVP(passive_stiffness));
  }
};

struct OimSimulationParams {
  std::string scene_model;
  double time_step{}, realtime_rate{}, actuator_delay{};
  double robot_publish_rate{}, object_publish_rate{};
  bool publish_efforts{}, visualize{};
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(scene_model)); a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(realtime_rate)); a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(publish_efforts)); a->Visit(DRAKE_NVP(visualize));
  }
};

struct OimObjectParams {
  std::string model, model_instance, body;
  Eigen::Vector3d start_pose, goal_pose;
  Eigen::Vector3d start_angular_velocity_W, start_linear_velocity_W;
  double resting_height{}, mass{}, planar_moment_inertia{};
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(model)); a->Visit(DRAKE_NVP(model_instance));
    a->Visit(DRAKE_NVP(body)); a->Visit(DRAKE_NVP(start_pose));
    a->Visit(DRAKE_NVP(goal_pose));
    a->Visit(DRAKE_NVP(start_angular_velocity_W));
    a->Visit(DRAKE_NVP(start_linear_velocity_W));
    a->Visit(DRAKE_NVP(resting_height));
    a->Visit(DRAKE_NVP(mass));
    a->Visit(DRAKE_NVP(planar_moment_inertia));
  }
};

struct OimTaskParams {
  std::string pose_frame;
  double translation_tolerance{}, orientation_tolerance{};
  int run_steps{};
  double planning_time_step{}, execution_time_step{};
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(pose_frame));
    a->Visit(DRAKE_NVP(translation_tolerance));
    a->Visit(DRAKE_NVP(orientation_tolerance)); a->Visit(DRAKE_NVP(run_steps));
    a->Visit(DRAKE_NVP(planning_time_step));
    a->Visit(DRAKE_NVP(execution_time_step));
  }
};

struct OimControllerParams {
  bool include_end_effector_orientation{};
  int control_loop_delay_ms{};
  Eigen::Vector3d task_space_kp, task_space_kd;
  double descent_posture_kp{}, descent_posture_kd{}, descent_posture_weight{};
  double reposition_posture_kp{}, reposition_posture_kd{};
  double descent_diff_ik_centering_gain{}, descent_diff_ik_max_velocity{};
  double task_space_plan_step_limit{}, approach_command_step_limit{};
  double descent_command_step_limit{};
  double pusher_radius{}, approach_clearance{}, descent_clearance{};
  double approach_planar_tolerance{};
  double contact_activation_tolerance{};
  double object_yaw_cost_weight{}, yaw_selection_weight{};
  double lateral_drift_weight{}, lateral_drift_tolerance{};
  double minimum_contact_normal_step{};
  double reposition_speed{}, reposition_waypoint_height{};
  int successor_minimum_contact_steps{}, successor_progress_window_steps{};
  int physical_contact_dwell_steps{};
  double successor_minimum_yaw_progress{};
  double successor_minimum_translation_progress{};
  double successor_cost_hysteresis_fraction{};
  std::string osqp_settings_file, osc_qp_settings_file;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(control_loop_delay_ms));
    a->Visit(DRAKE_NVP(task_space_kp));
    a->Visit(DRAKE_NVP(task_space_kd));
    a->Visit(DRAKE_NVP(descent_posture_kp));
    a->Visit(DRAKE_NVP(descent_posture_kd));
    a->Visit(DRAKE_NVP(descent_posture_weight));
    a->Visit(DRAKE_NVP(reposition_posture_kp));
    a->Visit(DRAKE_NVP(reposition_posture_kd));
    a->Visit(DRAKE_NVP(descent_diff_ik_centering_gain));
    a->Visit(DRAKE_NVP(descent_diff_ik_max_velocity));
    a->Visit(DRAKE_NVP(task_space_plan_step_limit));
    a->Visit(DRAKE_NVP(approach_command_step_limit));
    a->Visit(DRAKE_NVP(descent_command_step_limit));
    a->Visit(DRAKE_NVP(pusher_radius));
    a->Visit(DRAKE_NVP(approach_clearance));
    a->Visit(DRAKE_NVP(descent_clearance));
    a->Visit(DRAKE_NVP(approach_planar_tolerance));
    a->Visit(DRAKE_NVP(contact_activation_tolerance));
    a->Visit(DRAKE_NVP(object_yaw_cost_weight));
    a->Visit(DRAKE_NVP(yaw_selection_weight));
    a->Visit(DRAKE_NVP(lateral_drift_weight));
    a->Visit(DRAKE_NVP(lateral_drift_tolerance));
    a->Visit(DRAKE_NVP(minimum_contact_normal_step));
    a->Visit(DRAKE_NVP(reposition_speed));
    a->Visit(DRAKE_NVP(reposition_waypoint_height));
    a->Visit(DRAKE_NVP(successor_minimum_contact_steps));
    a->Visit(DRAKE_NVP(successor_progress_window_steps));
    a->Visit(DRAKE_NVP(successor_minimum_yaw_progress));
    a->Visit(DRAKE_NVP(successor_minimum_translation_progress));
    a->Visit(DRAKE_NVP(physical_contact_dwell_steps));
    a->Visit(DRAKE_NVP(successor_cost_hysteresis_fraction));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
  }
};

struct OimFullSamplingC3PlusParams {
  std::string contact_model;
  int horizon{}, admm_iterations{}, num_threads{};
  int delta_option{};
  bool warm_start{}, penalize_input_change{}, end_on_qp_step{}, scale_lcs{};
  double rho_scale{}, gamma{}, qp_projection_alpha{};
  double qp_projection_scaling{};
  int num_friction_directions{};
  double pusher_ground_friction{}, pusher_object_friction{};
  double object_ground_friction{};
  double state_cost_scale{}, input_cost_scale{};
  double consensus_cost_scale{}, projection_cost_scale{};
  Eigen::VectorXd state_cost_diagonal, input_cost_diagonal;
  double consensus_lambda_weight{}, consensus_eta_weight{};
  double projection_lambda_weight{}, projection_eta_weight{};
  double dynamics_residual_tolerance{}, equality_residual_tolerance{};
  double nonnegative_residual_tolerance{};
  double complementarity_residual_tolerance{};
  double consensus_residual_tolerance{};
  Eigen::VectorXd workspace_lower, workspace_upper;
  double workspace_margin{};
  Eigen::VectorXd input_lower, input_upper;
  Eigen::VectorXd ee_velocity_lower, ee_velocity_upper;
  int perimeter_sample_count{}, random_seed{}, mesh_sample_count{};
  int mesh_random_seed{}, num_outer_threads{};
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(horizon));
    a->Visit(DRAKE_NVP(admm_iterations));
    a->Visit(DRAKE_NVP(num_threads));
    a->Visit(DRAKE_NVP(delta_option));
    a->Visit(DRAKE_NVP(warm_start));
    a->Visit(DRAKE_NVP(penalize_input_change));
    a->Visit(DRAKE_NVP(end_on_qp_step));
    a->Visit(DRAKE_NVP(scale_lcs));
    a->Visit(DRAKE_NVP(rho_scale));
    a->Visit(DRAKE_NVP(gamma));
    a->Visit(DRAKE_NVP(qp_projection_alpha));
    a->Visit(DRAKE_NVP(qp_projection_scaling));
    a->Visit(DRAKE_NVP(num_friction_directions));
    a->Visit(DRAKE_NVP(pusher_ground_friction));
    a->Visit(DRAKE_NVP(pusher_object_friction));
    a->Visit(DRAKE_NVP(object_ground_friction));
    a->Visit(DRAKE_NVP(state_cost_scale));
    a->Visit(DRAKE_NVP(input_cost_scale));
    a->Visit(DRAKE_NVP(consensus_cost_scale));
    a->Visit(DRAKE_NVP(projection_cost_scale));
    a->Visit(DRAKE_NVP(state_cost_diagonal));
    a->Visit(DRAKE_NVP(input_cost_diagonal));
    a->Visit(DRAKE_NVP(consensus_lambda_weight));
    a->Visit(DRAKE_NVP(consensus_eta_weight));
    a->Visit(DRAKE_NVP(projection_lambda_weight));
    a->Visit(DRAKE_NVP(projection_eta_weight));
    a->Visit(DRAKE_NVP(dynamics_residual_tolerance));
    a->Visit(DRAKE_NVP(equality_residual_tolerance));
    a->Visit(DRAKE_NVP(nonnegative_residual_tolerance));
    a->Visit(DRAKE_NVP(complementarity_residual_tolerance));
    a->Visit(DRAKE_NVP(consensus_residual_tolerance));
    a->Visit(DRAKE_NVP(workspace_lower));
    a->Visit(DRAKE_NVP(workspace_upper));
    a->Visit(DRAKE_NVP(workspace_margin));
    a->Visit(DRAKE_NVP(input_lower));
    a->Visit(DRAKE_NVP(input_upper));
    a->Visit(DRAKE_NVP(ee_velocity_lower));
    a->Visit(DRAKE_NVP(ee_velocity_upper));
    a->Visit(DRAKE_NVP(perimeter_sample_count));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mesh_sample_count));
    a->Visit(DRAKE_NVP(mesh_random_seed));
    a->Visit(DRAKE_NVP(num_outer_threads));
  }
};

struct OimLcmParams {
  std::string robot_input_channel, robot_state_channel, object_state_channel;
  std::string tracking_trajectory_channel, osc_debug_channel;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(robot_input_channel));
    a->Visit(DRAKE_NVP(robot_state_channel));
    a->Visit(DRAKE_NVP(object_state_channel));
    a->Visit(DRAKE_NVP(tracking_trajectory_channel));
    a->Visit(DRAKE_NVP(osc_debug_channel));
  }
};

struct OimTParams {
  std::string scenario_name;
  OimXarmRobotParams robot;
  OimSimulationParams simulation;
  OimObjectParams object;
  OimTaskParams task;
  OimControllerParams controller;
  OimFullSamplingC3PlusParams full_sampling_c3plus;
  OimLcmParams lcm;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(scenario_name)); a->Visit(DRAKE_NVP(robot));
    a->Visit(DRAKE_NVP(simulation)); a->Visit(DRAKE_NVP(object));
    a->Visit(DRAKE_NVP(task)); a->Visit(DRAKE_NVP(controller));
    a->Visit(DRAKE_NVP(full_sampling_c3plus));
    a->Visit(DRAKE_NVP(lcm));
  }
};
