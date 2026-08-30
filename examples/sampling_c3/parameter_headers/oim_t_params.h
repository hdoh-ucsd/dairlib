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
  double resting_height{};
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(model)); a->Visit(DRAKE_NVP(model_instance));
    a->Visit(DRAKE_NVP(body)); a->Visit(DRAKE_NVP(start_pose));
    a->Visit(DRAKE_NVP(goal_pose)); a->Visit(DRAKE_NVP(resting_height));
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
  std::string sampling_c3_options_file, sampling_params_file;
  std::string reposition_params_file, progress_params_file, osc_params_file;
  std::string osqp_settings_file, osc_qp_settings_file;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(include_end_effector_orientation));
    a->Visit(DRAKE_NVP(control_loop_delay_ms));
    a->Visit(DRAKE_NVP(task_space_kp));
    a->Visit(DRAKE_NVP(task_space_kd));
    a->Visit(DRAKE_NVP(sampling_c3_options_file));
    a->Visit(DRAKE_NVP(sampling_params_file));
    a->Visit(DRAKE_NVP(reposition_params_file));
    a->Visit(DRAKE_NVP(progress_params_file));
    a->Visit(DRAKE_NVP(osc_params_file));
    a->Visit(DRAKE_NVP(osqp_settings_file));
    a->Visit(DRAKE_NVP(osc_qp_settings_file));
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
  OimLcmParams lcm;
  template <typename Archive> void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(scenario_name)); a->Visit(DRAKE_NVP(robot));
    a->Visit(DRAKE_NVP(simulation)); a->Visit(DRAKE_NVP(object));
    a->Visit(DRAKE_NVP(task)); a->Visit(DRAKE_NVP(controller));
    a->Visit(DRAKE_NVP(lcm));
  }
};
