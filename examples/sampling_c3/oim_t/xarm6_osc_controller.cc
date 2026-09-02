#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

#include <dairlib/lcmt_robot_input.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_quaternion.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm6_process_common.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");
DEFINE_bool(track_tip_orientation, true,
            "Condition xArm roll/pitch during task-space tracking");
DEFINE_bool(translation_only_descent, true,
            "Use translation-only OSC after the first downward command");
DEFINE_string(control_log, "",
              "Optional new CSV file for measured xArm state and control telemetry");
DEFINE_double(control_log_period, 0.02,
              "Minimum simulation seconds between xArm control CSV rows");

namespace dairlib::oim {

const dairlib::lcmt_trajectory_block* FindTrajectory(
    const dairlib::lcmt_timestamped_saved_traj* message,
    const std::string& name) {
  if (message == nullptr) return nullptr;
  const auto& saved = message->saved_traj;
  if (saved.num_trajectories !=
          static_cast<int>(saved.trajectory_names.size()) ||
      saved.num_trajectories !=
          static_cast<int>(saved.trajectories.size())) {
    throw std::runtime_error("malformed OSC saved-trajectory message");
  }
  for (int i = 0; i < saved.num_trajectories; ++i) {
    if (saved.trajectory_names[i] == name) {
      return &saved.trajectories[i];
    }
  }
  return nullptr;
}

// A task/force message describes a bounded transition, not an indefinitely
// extrapolated command. Drake trajectories extrapolate the closest segment
// beyond their end time, so an unrefreshed nonzero-slope target ramps away
// without bound (measured post-release runaway: tip driven to z = 1.14 m and
// joint 3 into its limit while the planner waited for tip quiescence).
// Append an explicit final hold knot so the terminal slope is zero, matching
// the existing collision-aware posture-trajectory hold.
drake::trajectories::PiecewisePolynomial<double>
FirstOrderHoldWithTerminalHold(const Eigen::VectorXd& time_vector,
                               const Eigen::MatrixXd& datapoints) {
  const int points = static_cast<int>(time_vector.size());
  Eigen::VectorXd held_times(points + 1);
  held_times.head(points) = time_vector;
  const double last_segment = time_vector[points - 1] -
      time_vector[points - 2];
  held_times[points] =
      time_vector[points - 1] + std::max(last_segment, 1.0e-3);
  Eigen::MatrixXd held_points(datapoints.rows(), points + 1);
  held_points.leftCols(points) = datapoints;
  held_points.rightCols(1) = datapoints.rightCols(1);
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      held_times, held_points);
}

Eigen::Vector3d FirstPositionTarget(
    const dairlib::lcmt_trajectory_block& target) {
  if (target.num_datatypes != 3 || target.num_points < 1 ||
      target.datapoints.size() != 3) {
    throw std::runtime_error("OSC target must contain 3D samples");
  }
  Eigen::Vector3d result;
  for (int row = 0; row < 3; ++row) {
    if (target.datapoints[row].size() !=
        static_cast<size_t>(target.num_points)) {
      throw std::runtime_error("malformed OSC target datapoints");
    }
    result[row] = target.datapoints[row][0];
  }
  return result;
}

Eigen::VectorXd ReadTrajectoryColumn(
    const dairlib::lcmt_trajectory_block& target, int expected_rows,
    int column) {
  if (target.num_datatypes != expected_rows || target.num_points < 1 ||
      column < 0 || column >= target.num_points ||
      target.datapoints.size() != static_cast<size_t>(expected_rows)) {
    throw std::runtime_error("OSC vector target has incompatible dimensions");
  }
  Eigen::VectorXd result(expected_rows);
  for (int row = 0; row < expected_rows; ++row) {
    if (target.datapoints[row].size() !=
        static_cast<size_t>(target.num_points)) {
      throw std::runtime_error("malformed OSC vector target datapoints");
    }
    result[row] = target.datapoints[row][column];
  }
  return result;
}

class DescentPostureTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  DescentPostureTrajectorySource(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()), params_(params),
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
        Eigen::VectorXd::Zero(params.robot.controlled_joints.size()));
    drake::trajectories::Trajectory<double>& model = initial;
    this->DeclareAbstractOutputPort(
        "descent_posture_target", model,
        &DescentPostureTrajectorySource::CalcTrajectory);
  }

 private:
  Eigen::VectorXd ReadControlledPositions() const {
    Eigen::VectorXd q(params_.robot.controlled_joints.size());
    for (int i = 0; i < q.size(); ++i) {
      q[i] = plant_.GetJointByName(params_.robot.controlled_joints[i])
                 .GetPositions(*context_)[0];
    }
    return q;
  }

  void CalcTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, state_port_));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    const Eigen::Vector3d tip =
        plant_.EvalBodyPoseInWorld(*context_, end_effector_) * point_;
    const auto* message =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, message_port_);
    Eigen::VectorXd desired = ReadControlledPositions();
    const auto* target =
        FindTrajectory(message, "end_effector_position_target");
    if (target != nullptr) {
      const Eigen::Vector3d target_position = FirstPositionTarget(*target);
      const bool downward = target_position.z() < tip.z() -
          params_.controller.contact_activation_tolerance;
      if (!descent_latched_ && downward) {
        nominal_posture_ = desired;
        descent_latched_ = true;
        std::cout << "descent_diff_ik_latched q="
                  << nominal_posture_.transpose() << std::endl;
      }
      if (descent_latched_) {
        const double dt = params_.task.planning_time_step;
        Eigen::Vector3d desired_velocity =
            (target_position - tip) / dt;
        if (desired_velocity.norm() >
            params_.controller.descent_diff_ik_max_velocity) {
          desired_velocity *=
              params_.controller.descent_diff_ik_max_velocity /
              desired_velocity.norm();
        }
        Eigen::MatrixXd J_translation(3, plant_.num_velocities());
        plant_.CalcJacobianTranslationalVelocity(
            *context_, drake::multibody::JacobianWrtVariable::kV,
            end_effector_.body_frame(), point_, plant_.world_frame(),
            plant_.world_frame(), &J_translation);
        Eigen::MatrixXd J = J_translation;
        Eigen::VectorXd desired_task_velocity = desired_velocity;
        const auto* axis_target =
            FindTrajectory(message, "end_effector_stick_axis_target");
        if (axis_target != nullptr) {
          Eigen::Vector3d desired_axis = FirstPositionTarget(*axis_target);
          desired_axis.normalize();
          const Eigen::Vector3d current_axis =
              plant_.EvalBodyPoseInWorld(*context_, end_effector_)
                  .rotation() * Eigen::Vector3d::UnitZ();
          Eigen::Vector3d desired_axis_velocity =
              (desired_axis - current_axis) / dt;
          if (desired_axis_velocity.norm() >
              params_.controller.descent_diff_ik_max_velocity) {
            desired_axis_velocity *=
                params_.controller.descent_diff_ik_max_velocity /
                desired_axis_velocity.norm();
          }
          Eigen::MatrixXd J_spatial(6, plant_.num_velocities());
          plant_.CalcJacobianSpatialVelocity(
              *context_, drake::multibody::JacobianWrtVariable::kV,
              end_effector_.body_frame(), Eigen::Vector3d::Zero(),
              plant_.world_frame(), plant_.world_frame(), &J_spatial);
          Eigen::Matrix3d axis_cross;
          axis_cross << 0.0, -current_axis.z(), current_axis.y(),
              current_axis.z(), 0.0, -current_axis.x(),
              -current_axis.y(), current_axis.x(), 0.0;
          J.resize(6, plant_.num_velocities());
          J.topRows(3) = J_translation;
          J.bottomRows(3) = -axis_cross * J_spatial.topRows(3);
          desired_task_velocity.resize(6);
          desired_task_velocity << desired_velocity, desired_axis_velocity;
        }
        drake::multibody::DifferentialInverseKinematicsParameters diff_ik(
            plant_.num_positions(), plant_.num_velocities());
        diff_ik.set_time_step(dt);
        diff_ik.set_nominal_joint_position(nominal_posture_);
        diff_ik.set_joint_centering_gain(
            params_.controller.descent_diff_ik_centering_gain *
            Eigen::MatrixXd::Identity(plant_.num_positions(),
                                      plant_.num_positions()));
        diff_ik.set_joint_position_limits(
            {plant_.GetPositionLowerLimits(), plant_.GetPositionUpperLimits()});
        diff_ik.set_joint_velocity_limits(
            {-params_.robot.velocity_limits, params_.robot.velocity_limits});
        const auto result =
            drake::multibody::DoDifferentialInverseKinematics(
                plant_.GetPositions(*context_),
                plant_.GetVelocities(*context_), desired_task_velocity, J,
                diff_ik);
        if (result.joint_velocities.has_value()) {
          desired += dt * *result.joint_velocities;
        } else if (message->utime != last_failure_utime_) {
          last_failure_utime_ = message->utime;
          std::cerr << "descent_diff_ik_no_solution utime=" << message->utime
                    << std::endl;
        }
      }
    }
    *dynamic_cast<drake::trajectories::PiecewisePolynomial<double>*>(output) =
        drake::trajectories::PiecewisePolynomial<double>(desired);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  const OimTParams& params_;
  const drake::multibody::RigidBody<double>& end_effector_;
  Eigen::Vector3d point_;
  drake::systems::InputPortIndex state_port_, message_port_;
  mutable bool descent_latched_{false};
  mutable Eigen::VectorXd nominal_posture_;
  mutable int64_t last_failure_utime_{-1};
};

class AdaptiveOrientationTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  AdaptiveOrientationTrajectorySource(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params, const Eigen::Quaterniond& home_orientation)
      : plant_(plant), context_(plant.CreateDefaultContext()),
        end_effector_(plant.GetBodyByName(params.robot.end_effector_body)),
        point_(params.robot.end_effector_point),
        home_orientation_(home_orientation),
        descent_trigger_tolerance_(
            params.controller.contact_activation_tolerance) {
    state_port_ = this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators())).get_index();
    message_port_ = this->DeclareAbstractInputPort(
        "task_space_trajectory",
        drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();
    drake::trajectories::PiecewiseQuaternionSlerp<double> initial(
        {0.0, 1.0}, {home_orientation, home_orientation});
    drake::trajectories::Trajectory<double>& model = initial;
    this->DeclareAbstractOutputPort(
        "end_effector_orientation_target", model,
        &AdaptiveOrientationTrajectorySource::CalcTrajectory);
  }

 private:
  void CalcTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, state_port_));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    const auto X_WEe =
        plant_.EvalBodyPoseInWorld(*context_, end_effector_);
    const Eigen::Vector3d tip = X_WEe * point_;
    Eigen::Quaterniond desired = home_orientation_;
    const auto* message =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, message_port_);
    const auto* target =
        FindTrajectory(message, "end_effector_position_target");
    if (target != nullptr) {
      const Eigen::Vector3d target_position = FirstPositionTarget(*target);
      if (!descent_latched_ &&
          target_position.z() < tip.z() - descent_trigger_tolerance_) {
        descent_orientation_ = Eigen::Quaterniond(X_WEe.rotation().matrix());
        descent_latched_ = true;
      }
    }
    if (descent_latched_) desired = descent_orientation_;
    drake::trajectories::PiecewiseQuaternionSlerp<double> trajectory(
        {0.0, 1.0}, {desired, desired});
    *dynamic_cast<drake::trajectories::PiecewiseQuaternionSlerp<double>*>(
        output) = trajectory;
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::RigidBody<double>& end_effector_;
  Eigen::Vector3d point_;
  Eigen::Quaterniond home_orientation_;
  double descent_trigger_tolerance_;
  mutable bool descent_latched_{false};
  mutable Eigen::Quaterniond descent_orientation_{Eigen::Quaterniond::Identity()};
  drake::systems::InputPortIndex state_port_, message_port_;
};

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
    const auto* target =
        FindTrajectory(message, "end_effector_position_target");
    if (target != nullptr) {
      if (target->num_datatypes != 3 || target->num_points < 2 ||
          target->time_vec.size() !=
              static_cast<size_t>(target->num_points) ||
          target->datapoints.size() != 3) {
        throw std::runtime_error("OSC target must contain at least two 3D samples");
      }
      Eigen::VectorXd time_vector = Eigen::VectorXd::Map(
          target->time_vec.data(), target->num_points);
      Eigen::MatrixXd datapoints(3, target->num_points);
      for (int row = 0; row < 3; ++row) {
        if (target->datapoints[row].size() !=
            static_cast<size_t>(target->num_points)) {
          throw std::runtime_error("malformed OSC target datapoints");
        }
        datapoints.row(row) = Eigen::VectorXd::Map(
            target->datapoints[row].data(), target->num_points);
      }
      *result = FirstOrderHoldWithTerminalHold(time_vector, datapoints);
      if (message->utime != last_target_utime_) {
        last_target_utime_ = message->utime;
        std::cout << "osc_target_received utime=" << message->utime
                  << " target_W=" << datapoints.col(0).transpose()
                  << std::endl;
      }
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
  mutable int64_t last_target_utime_{-1};
};

class SafeForceTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  SafeForceTrajectorySource() {
    message_port_ = this->DeclareAbstractInputPort(
        "task_space_trajectory",
        drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();
    drake::trajectories::PiecewisePolynomial<double> initial(
        Eigen::Vector3d::Zero());
    drake::trajectories::Trajectory<double>& model = initial;
    output_port_ = this->DeclareAbstractOutputPort(
        "safe_end_effector_force_target", model,
        &SafeForceTrajectorySource::CalcTrajectory).get_index();
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
    const auto* target =
        FindTrajectory(message, "end_effector_force_target");
    if (target == nullptr) {
      *result = drake::trajectories::PiecewisePolynomial<double>(
          Eigen::Vector3d::Zero());
      return;
    }
    if (target->num_datatypes != 3 || target->num_points < 2 ||
        target->time_vec.size() !=
            static_cast<size_t>(target->num_points) ||
        target->datapoints.size() != 3) {
      throw std::runtime_error(
          "OSC force target must contain at least two 3D samples");
    }
    Eigen::VectorXd time_vector = Eigen::VectorXd::Map(
        target->time_vec.data(), target->num_points);
    Eigen::MatrixXd datapoints(3, target->num_points);
    for (int row = 0; row < 3; ++row) {
      if (target->datapoints[row].size() !=
          static_cast<size_t>(target->num_points)) {
        throw std::runtime_error("malformed OSC force target datapoints");
      }
      datapoints.row(row) = Eigen::VectorXd::Map(
          target->datapoints[row].data(), target->num_points);
    }
    *result = FirstOrderHoldWithTerminalHold(time_vector, datapoints);
  }

  drake::systems::InputPortIndex message_port_;
  drake::systems::OutputPortIndex output_port_;
};

class CollisionAwarePostureTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  CollisionAwarePostureTrajectorySource(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()), params_(params) {
    state_port_ = this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators())).get_index();
    message_port_ = this->DeclareAbstractInputPort(
        "task_space_trajectory",
        drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();
    drake::trajectories::PiecewisePolynomial<double> initial(
        Eigen::VectorXd::Zero(params.robot.controlled_joints.size()));
    drake::trajectories::Trajectory<double>& model = initial;
    this->DeclareAbstractOutputPort(
        "collision_aware_reposition_posture_target", model,
        &CollisionAwarePostureTrajectorySource::CalcTrajectory);
  }

 private:
  Eigen::VectorXd ReadMeasuredPosture() const {
    Eigen::VectorXd q(params_.robot.controlled_joints.size());
    for (int i = 0; i < q.size(); ++i) {
      q[i] = plant_.GetJointByName(params_.robot.controlled_joints[i])
                 .GetPositions(*context_)[0];
    }
    return q;
  }

  void CalcTrajectory(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, state_port_));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    auto* result = dynamic_cast<
        drake::trajectories::PiecewisePolynomial<double>*>(output);
    const auto* message =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, message_port_);
    const auto* target = FindTrajectory(
        message, "collision_aware_reposition_posture_target");
    if (target == nullptr) {
      *result = drake::trajectories::PiecewisePolynomial<double>(
          ReadMeasuredPosture());
      return;
    }
    const int rows = params_.robot.controlled_joints.size();
    if (target->num_points < 2 ||
        target->time_vec.size() != static_cast<size_t>(target->num_points)) {
      throw std::runtime_error(
          "reposition posture target requires at least two timed knots");
    }
    Eigen::VectorXd time_vector = Eigen::VectorXd::Map(
        target->time_vec.data(), target->num_points);
    Eigen::MatrixXd datapoints(rows, target->num_points);
    for (int column = 0; column < target->num_points; ++column) {
      datapoints.col(column) =
          ReadTrajectoryColumn(*target, rows, column);
    }
    // A reposition message describes a bounded transition, not an
    // indefinitely extrapolated joint velocity.  Add an explicit final hold
    // segment so FirstOrderHold has zero terminal slope if the planner waits
    // at contact without publishing another posture message.
    Eigen::VectorXd held_time_vector(time_vector.size() + 1);
    held_time_vector.head(time_vector.size()) = time_vector;
    held_time_vector[time_vector.size()] =
        time_vector[time_vector.size() - 1] +
        params_.task.planning_time_step;
    Eigen::MatrixXd held_datapoints(rows, target->num_points + 1);
    held_datapoints.leftCols(target->num_points) = datapoints;
    held_datapoints.rightCols(1) = datapoints.rightCols(1);
    *result = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
        held_time_vector, held_datapoints);
    if (message->utime != last_target_utime_) {
      last_target_utime_ = message->utime;
      const Eigen::VectorXd measured = ReadMeasuredPosture();
      std::cout << "collision_aware_posture_received utime="
                << message->utime << " publisher=\""
                << message->saved_traj.metadata.description << "\" q_target="
                << datapoints.rightCols(1).transpose()
                << " q_measured=" << measured.transpose()
                << " posture_error_inf="
                << (datapoints.rightCols(1) - measured)
                       .lpNorm<Eigen::Infinity>()
                << std::endl;
    }
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  const OimTParams& params_;
  drake::systems::InputPortIndex state_port_, message_port_;
  mutable int64_t last_target_utime_{-1};
};

class OscPhaseMux final : public drake::systems::LeafSystem<double> {
 public:
  OscPhaseMux(const drake::multibody::MultibodyPlant<double>& plant,
              const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()), params_(params),
        end_effector_(plant.GetBodyByName(params.robot.end_effector_body)),
        point_(params.robot.end_effector_point),
        descent_trigger_tolerance_(
            params.controller.contact_activation_tolerance) {
    this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators()));
    this->DeclareAbstractInputPort(
        "task_space_trajectory",
        drake::Value<dairlib::lcmt_timestamped_saved_traj>{});
    this->DeclareVectorInputPort(
        "conditioned_osc", systems::TimestampedVector<double>(
                               plant.num_actuators()));
    this->DeclareVectorInputPort(
        "translation_only_osc", systems::TimestampedVector<double>(
                                  plant.num_actuators()));
    this->DeclareVectorInputPort(
        "collision_aware_posture_osc", systems::TimestampedVector<double>(
                                           plant.num_actuators()));
    this->DeclareVectorOutputPort(
        "selected_osc", systems::TimestampedVector<double>(
                            plant.num_actuators()),
        &OscPhaseMux::SelectTorque);
  }

 private:
  void SelectTorque(const drake::systems::Context<double>& context,
                    systems::TimestampedVector<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, 0));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    const Eigen::Vector3d tip =
        plant_.EvalBodyPoseInWorld(*context_, end_effector_) * point_;
    const auto* message =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(context, 1);
    const auto* target =
        FindTrajectory(message, "end_effector_position_target");
    if (target != nullptr) {
      const Eigen::Vector3d target_position = FirstPositionTarget(*target);
      translation_only_latched_ = translation_only_latched_ ||
          target_position.z() < tip.z() - descent_trigger_tolerance_;
    }
    const bool reposition_posture_active = FindTrajectory(
        message, "collision_aware_reposition_posture_target") != nullptr;
    const auto* selected =
        dynamic_cast<const systems::TimestampedVector<double>*>(
            this->EvalVectorInput(
                context,
                reposition_posture_active
                    ? 4
                    : (FLAGS_translation_only_descent &&
                               translation_only_latched_
                           ? 3
                           : 2)));
    Eigen::VectorXd selected_torque = selected->get_data();
    if (reposition_posture_active) {
      const auto* posture_target = FindTrajectory(
          message, "collision_aware_reposition_posture_target");
      const int joints = params_.robot.controlled_joints.size();
      const Eigen::VectorXd start_q = ReadTrajectoryColumn(
          *posture_target, joints, 0);
      const Eigen::VectorXd target_q = ReadTrajectoryColumn(
          *posture_target, joints, posture_target->num_points - 1);
      const double command_duration =
          posture_target->time_vec.back() - posture_target->time_vec.front();
      if (!std::isfinite(command_duration) || command_duration <= 0.0) {
        throw std::runtime_error(
            "collision-aware posture command duration must be positive");
      }
      Eigen::VectorXd desired_v =
          (target_q - start_q) / command_duration;
      for (int i = 0; i < joints; ++i) {
        desired_v[i] = std::clamp(
            desired_v[i], -params_.robot.velocity_limits[i],
            params_.robot.velocity_limits[i]);
        const auto& joint = dynamic_cast<
            const drake::multibody::RevoluteJoint<double>&>(
            plant_.GetJointByName(params_.robot.controlled_joints[i]));
        selected_torque[i] +=
            joint.GetDampingVector(*context_)[0] * desired_v[i];
      }
    }
    if (reposition_posture_active && message != nullptr &&
        message->utime - last_reposition_torque_log_utime_ >= 500000) {
      last_reposition_torque_log_utime_ = message->utime;
      std::cout << "collision_aware_posture_torque utime="
                << message->utime << " selected="
                << selected_torque.transpose() << std::endl;
    }
    output->SetDataVector(selected_torque);
    output->set_timestamp(state->get_timestamp());
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  const OimTParams& params_;
  const drake::multibody::RigidBody<double>& end_effector_;
  Eigen::Vector3d point_;
  double descent_trigger_tolerance_;
  // Once the staged planner begins descent it never returns to planar
  // acquisition.  Mirror that monotonic contract here so convergence near the
  // target cannot re-enable orientation conditioning and disturb x/y.
  mutable bool translation_only_latched_{false};
  mutable int64_t last_reposition_torque_log_utime_{-500000};
};

class Xarm6WristRollTrajectorySource final
    : public drake::systems::LeafSystem<double> {
 public:
  explicit Xarm6WristRollTrajectorySource(double home_position)
      : home_position_(home_position) {
    drake::trajectories::PiecewisePolynomial<double> initial(
        Eigen::VectorXd::Constant(1, home_position_));
    drake::trajectories::Trajectory<double>& model = initial;
    this->DeclareAbstractOutputPort(
        "xarm6_wrist_roll_target", model,
        &Xarm6WristRollTrajectorySource::CalcTrajectory);
  }

 private:
  void CalcTrajectory(
      const drake::systems::Context<double>&,
      drake::trajectories::Trajectory<double>* output) const {
    auto* polynomial = dynamic_cast<
        drake::trajectories::PiecewisePolynomial<double>*>(output);
    if (polynomial == nullptr) {
      throw std::runtime_error(
          "xArm6 wrist-roll target output has the wrong trajectory type");
    }
    *polynomial = drake::trajectories::PiecewisePolynomial<double>(
        Eigen::VectorXd::Constant(1, home_position_));
  }

  double home_position_{};
};

class XarmVelocityServoBridge final
    : public drake::systems::LeafSystem<double> {
 public:
  XarmVelocityServoBridge(
      const drake::multibody::MultibodyPlant<double>& plant,
      const OimTParams& params, const std::string& control_log_path,
      double control_log_period)
      : plant_(plant), context_(plant.CreateDefaultContext()),
        effort_limits_(params.robot.effort_limits),
        velocity_limits_(params.robot.velocity_limits),
        servo_gains_(params.robot.velocity_servo_gains),
        joint_names_(params.robot.controlled_joints),
        control_log_period_(control_log_period) {
    if (!control_log_path.empty()) {
      if (!(control_log_period_ > 0.0)) {
        throw std::runtime_error("control_log_period must be positive");
      }
      if (std::filesystem::exists(control_log_path)) {
        throw std::runtime_error(
            "control_log already exists; refusing to overwrite " +
            control_log_path);
      }
      control_log_.open(control_log_path);
      if (!control_log_) {
        throw std::runtime_error("failed to create control_log " +
                                 control_log_path);
      }
      control_log_ << std::setprecision(17) << std::unitbuf;
      control_log_ << "time_s";
      for (const auto& name : joint_names_) control_log_ << ",q_" << name;
      for (const auto& name : joint_names_) control_log_ << ",v_" << name;
      for (const auto& name : joint_names_) control_log_ << ",tau_osc_" << name;
      for (const auto& name : joint_names_)
        control_log_ << ",tau_servo_desired_" << name;
      for (const auto& name : joint_names_)
        control_log_ << ",qdot_command_" << name;
      for (const auto& name : joint_names_)
        control_log_ << ",tau_servo_clamped_" << name;
      for (const auto& name : joint_names_)
        control_log_ << ",tau_gravity_" << name;
      for (const auto& name : joint_names_)
        control_log_ << ",tau_command_" << name;
      control_log_ << '\n';
    }
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
    Eigen::VectorXd velocity_command(plant_.num_actuators());
    Eigen::VectorXd servo_torque(plant_.num_actuators());
    for (int i = 0; i < servo_torque.size(); ++i) {
      velocity_command[i] = std::clamp(
          velocity[i] + desired_servo_torque[i] / servo_gains_[i],
          -velocity_limits_[i], velocity_limits_[i]);
      servo_torque[i] = std::clamp(
          servo_gains_[i] * (velocity_command[i] - velocity[i]),
          -effort_limits_[i], effort_limits_[i]);
    }
    const Eigen::VectorXd torque = servo_torque + gravity_compensation;
    const double timestamp = state->get_timestamp();
    if (control_log_ &&
        timestamp >= last_control_log_time_ + control_log_period_ - 1e-12) {
      last_control_log_time_ = timestamp;
      const Eigen::VectorXd position = plant_.GetPositions(*context_);
      control_log_ << timestamp;
      const auto append = [this](const Eigen::VectorXd& values) {
        for (const double value : values) control_log_ << ',' << value;
      };
      append(position);
      append(velocity);
      append(osc_torque->get_data());
      append(desired_servo_torque);
      append(velocity_command);
      append(servo_torque);
      append(gravity_compensation);
      append(torque);
      control_log_ << '\n';
    }
    output->SetDataVector(torque);
    output->set_timestamp(timestamp);
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  Eigen::VectorXd effort_limits_;
  Eigen::VectorXd velocity_limits_;
  Eigen::VectorXd servo_gains_;
  std::vector<std::string> joint_names_;
  double control_log_period_;
  mutable std::ofstream control_log_;
  mutable double last_control_log_time_{-
      std::numeric_limits<double>::infinity()};
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
  SetXarmHome(params, plant, plant_context.get());

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
  auto* force_source = builder.AddSystem<SafeForceTrajectorySource>();
  auto* descent_posture_source =
      builder.AddSystem<DescentPostureTrajectorySource>(plant, params);
  auto* reposition_posture_source =
      builder.AddSystem<CollisionAwarePostureTrajectorySource>(plant, params);
  auto* osc =
      builder.AddSystem<systems::controllers::OperationalSpaceControl>(
          plant, plant_context.get(), false);
  auto* translation_only_osc =
      builder.AddSystem<systems::controllers::OperationalSpaceControl>(
          plant, plant_context.get(), false);
  auto* reposition_posture_osc =
      builder.AddSystem<systems::controllers::OperationalSpaceControl>(
          plant, plant_context.get(), false);
  osc->set_name("conditioned_xarm_osc");
  translation_only_osc->set_name("translation_only_xarm_osc");
  reposition_posture_osc->set_name("collision_aware_posture_xarm_osc");
  auto tracking = std::make_unique<
      systems::controllers::TransTaskSpaceTrackingData>(
      "end_effector_position_target",
      params.controller.task_space_kp.asDiagonal(),
      params.controller.task_space_kd.asDiagonal(),
      Eigen::Matrix3d::Identity(), plant, plant);
  tracking->AddPointToTrack(params.robot.end_effector_body,
                            params.robot.end_effector_point);
  const Eigen::Vector3d task_acceleration_limits =
      params.controller.task_space_kp.cwiseProduct(
          Eigen::Vector3d::Constant(
              params.controller.approach_command_step_limit));
  tracking->SetCmdAccelerationBounds(
      -task_acceleration_limits, task_acceleration_limits);
  osc->AddTrackingData(std::move(tracking));
  // The wrist roll is the last controlled joint by convention: the joint
  // whose axis coincides with the axisymmetric stick (xarm6_joint6 on the
  // xArm6, panda_joint7 on the Franka Panda).
  const int wrist_roll_index =
      static_cast<int>(params.robot.controlled_joints.size()) - 1;
  auto* wrist_roll_source =
      builder.AddSystem<Xarm6WristRollTrajectorySource>(
          params.robot.home_positions[wrist_roll_index]);
  auto wrist_roll_tracking = std::make_unique<
      systems::controllers::JointSpaceTrackingData>(
      "xarm6_wrist_roll_target",
      params.controller.descent_posture_kp * Eigen::MatrixXd::Identity(1, 1),
      params.controller.descent_posture_kd * Eigen::MatrixXd::Identity(1, 1),
      params.controller.descent_posture_weight *
          Eigen::MatrixXd::Identity(1, 1),
      plant, plant);
  wrist_roll_tracking->AddJointsToTrack(
      {params.robot.controlled_joints[wrist_roll_index]},
      {params.robot.controlled_joints[wrist_roll_index] + "dot"});
  osc->AddTrackingData(std::move(wrist_roll_tracking));
  auto translation_only_tracking = std::make_unique<
      systems::controllers::TransTaskSpaceTrackingData>(
      "end_effector_position_target",
      params.controller.task_space_kp.asDiagonal(),
      params.controller.task_space_kd.asDiagonal(),
      Eigen::Matrix3d::Identity(), plant, plant);
  translation_only_tracking->AddPointToTrack(
      params.robot.end_effector_body, params.robot.end_effector_point);
  translation_only_tracking->SetCmdAccelerationBounds(
      -task_acceleration_limits, task_acceleration_limits);
  translation_only_osc->AddTrackingData(std::move(translation_only_tracking));
  const int num_controlled_joints = params.robot.controlled_joints.size();
  auto descent_posture_tracking = std::make_unique<
      systems::controllers::JointSpaceTrackingData>(
      "descent_posture_target",
      params.controller.reposition_posture_kp *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      params.controller.reposition_posture_kd *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      params.controller.descent_posture_weight *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      plant, plant);
  std::vector<std::string> controlled_velocities;
  controlled_velocities.reserve(num_controlled_joints);
  for (const auto& joint : params.robot.controlled_joints) {
    controlled_velocities.push_back(joint + "dot");
  }
  descent_posture_tracking->AddJointsToTrack(
      params.robot.controlled_joints, controlled_velocities);
  translation_only_osc->AddTrackingData(std::move(descent_posture_tracking));
  auto reposition_posture_tracking = std::make_unique<
      systems::controllers::JointSpaceTrackingData>(
      "collision_aware_reposition_posture_target",
      params.controller.reposition_posture_kp *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      params.controller.reposition_posture_kd *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      params.controller.descent_posture_weight *
          Eigen::MatrixXd::Identity(num_controlled_joints,
                                    num_controlled_joints),
      plant, plant);
  reposition_posture_tracking->AddJointsToTrack(
      params.robot.controlled_joints, controlled_velocities);
  reposition_posture_osc->AddTrackingData(
      std::move(reposition_posture_tracking));
  // Collision-aware repositioning supplies a differential-IK posture and the
  // Cartesian waypoint from which that posture was constructed.  Track both:
  // posture alone can settle with a small joint-space error that is still a
  // large tip error near a singular or poorly conditioned configuration.
  auto reposition_translation_tracking = std::make_unique<
      systems::controllers::TransTaskSpaceTrackingData>(
      "collision_aware_reposition_position_target",
      params.controller.task_space_kp.asDiagonal(),
      params.controller.task_space_kd.asDiagonal(),
      Eigen::Matrix3d::Identity(), plant, plant);
  reposition_translation_tracking->AddPointToTrack(
      params.robot.end_effector_body, params.robot.end_effector_point);
  reposition_translation_tracking->SetCmdAccelerationBounds(
      -task_acceleration_limits, task_acceleration_limits);
  reposition_posture_osc->AddTrackingData(
      std::move(reposition_translation_tracking));
  auto make_force_tracking = [&]() {
    return std::make_unique<
        systems::controllers::ExternalForceTrackingData>(
        "end_effector_force", Eigen::Matrix3d::Identity(), plant, plant,
        params.robot.end_effector_body, params.robot.end_effector_point);
  };
  osc->AddForceTrackingData(make_force_tracking());
  translation_only_osc->AddForceTrackingData(make_force_tracking());
  reposition_posture_osc->AddForceTrackingData(make_force_tracking());
  // The xArm differential-IK reference weights tilt by 0.35 relative to
  // translation; OSC uses a quadratic weight, hence 0.35^2 = 0.1225.
  Eigen::Matrix3d orientation_weight =
      0.1225 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d orientation_kp =
      800.0 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d orientation_kd =
      40.0 * Eigen::Matrix3d::Identity();
  // Joint 6 now exposes wrist roll, but the pushing capsule remains
  // axis-symmetric.  Regulate that redundant coordinate through the
  // six-joint posture task while the Cartesian orientation task conditions
  // only the stick axis.
  orientation_weight(2, 2) = 0.0;
  orientation_kp(2, 2) = 0.0;
  orientation_kd(2, 2) = 0.0;
  AdaptiveOrientationTrajectorySource* orientation_source = nullptr;
  const Eigen::Quaterniond home_orientation(
      plant.EvalBodyPoseInWorld(
          *plant_context,
          plant.GetBodyByName(params.robot.end_effector_body)).rotation().matrix());
  if (FLAGS_track_tip_orientation) {
    auto orientation_tracking = std::make_unique<
        systems::controllers::RotTaskSpaceTrackingData>(
        "end_effector_orientation_target", orientation_kp, orientation_kd,
        orientation_weight, plant, plant);
    orientation_tracking->AddFrameToTrack(params.robot.end_effector_body);
    osc->AddTrackingData(std::move(orientation_tracking));
    orientation_source =
        builder.AddSystem<AdaptiveOrientationTrajectorySource>(
            plant, params, home_orientation);
  }
  // Preserve DAIRLab's shared Sampling-C3 acceleration regularization:
  // w_accel=1e-5 multiplied by W_accel=0.01 I.
  osc->SetAccelerationCostWeights(
      1.0e-7 * Eigen::MatrixXd::Identity(plant.num_velocities(),
                                         plant.num_velocities()));
  translation_only_osc->SetAccelerationCostWeights(
      1.0e-7 * Eigen::MatrixXd::Identity(plant.num_velocities(),
                                         plant.num_velocities()));
  reposition_posture_osc->SetAccelerationCostWeights(
      1.0e-7 * Eigen::MatrixXd::Identity(plant.num_velocities(),
                                         plant.num_velocities()));
  osc->Build();
  translation_only_osc->Build();
  reposition_posture_osc->Build();
  auto* osc_phase_mux = builder.AddSystem<OscPhaseMux>(plant, params);
  auto* servo_bridge =
      builder.AddSystem<XarmVelocityServoBridge>(
          plant, params, FLAGS_control_log, FLAGS_control_log_period);
  auto* command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto* command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          params.lcm.robot_input_channel, &lcm,
          drake::systems::TriggerTypeSet({drake::systems::TriggerType::kForced})));
  builder.Connect(*state_sub, *state_receiver);
  builder.Connect(trajectory_sub->get_output_port(),
                  trajectory_source->get_input_port(1));
  builder.Connect(trajectory_sub->get_output_port(),
                  force_source->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(),
                  trajectory_source->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(),
                  descent_posture_source->get_input_port(0));
  builder.Connect(trajectory_sub->get_output_port(),
                  descent_posture_source->get_input_port(1));
  builder.Connect(state_receiver->get_output_port(),
                  reposition_posture_source->get_input_port(0));
  builder.Connect(trajectory_sub->get_output_port(),
                  reposition_posture_source->get_input_port(1));
  builder.Connect(trajectory_source->trajectory_output(),
                  osc->get_input_port_tracking_data(
                      "end_effector_position_target"));
  builder.Connect(wrist_roll_source->get_output_port(),
                  osc->get_input_port_tracking_data(
                      "xarm6_wrist_roll_target"));
  builder.Connect(trajectory_source->trajectory_output(),
                  translation_only_osc->get_input_port_tracking_data(
                      "end_effector_position_target"));
  builder.Connect(descent_posture_source->get_output_port(),
                  translation_only_osc->get_input_port_tracking_data(
                      "descent_posture_target"));
  builder.Connect(reposition_posture_source->get_output_port(),
                  reposition_posture_osc->get_input_port_tracking_data(
                      "collision_aware_reposition_posture_target"));
  builder.Connect(trajectory_source->trajectory_output(),
                  reposition_posture_osc->get_input_port_tracking_data(
                      "collision_aware_reposition_position_target"));
  builder.Connect(force_source->trajectory_output(),
                  osc->get_input_port_tracking_data(
                      "end_effector_force"));
  builder.Connect(force_source->trajectory_output(),
                  translation_only_osc->get_input_port_tracking_data(
                      "end_effector_force"));
  builder.Connect(force_source->trajectory_output(),
                  reposition_posture_osc->get_input_port_tracking_data(
                      "end_effector_force"));
  if (orientation_source != nullptr) {
    builder.Connect(state_receiver->get_output_port(),
                    orientation_source->get_input_port(0));
    builder.Connect(trajectory_sub->get_output_port(),
                    orientation_source->get_input_port(1));
    builder.Connect(orientation_source->get_output_port(),
                    osc->get_input_port_tracking_data(
                        "end_effector_orientation_target"));
  }
  builder.Connect(state_receiver->get_output_port(),
                  osc->get_input_port_robot_output());
  builder.Connect(state_receiver->get_output_port(),
                  translation_only_osc->get_input_port_robot_output());
  builder.Connect(state_receiver->get_output_port(),
                  reposition_posture_osc->get_input_port_robot_output());
  builder.Connect(state_receiver->get_output_port(),
                  osc_phase_mux->get_input_port(0));
  builder.Connect(trajectory_sub->get_output_port(),
                  osc_phase_mux->get_input_port(1));
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_phase_mux->get_input_port(2));
  builder.Connect(translation_only_osc->get_output_port_osc_command(),
                  osc_phase_mux->get_input_port(3));
  builder.Connect(reposition_posture_osc->get_output_port_osc_command(),
                  osc_phase_mux->get_input_port(4));
  builder.Connect(state_receiver->get_output_port(),
                  servo_bridge->get_input_port(0));
  builder.Connect(osc_phase_mux->get_output_port(),
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
