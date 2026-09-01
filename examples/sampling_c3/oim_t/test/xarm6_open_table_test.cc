#include <gtest/gtest.h>

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "examples/sampling_c3/oim_t/xarm6_process_common.h"

namespace dairlib::oim {
namespace {

constexpr char kConfig[] =
    "examples/sampling_c3/oim_t/parameters/oim_t.yaml";

TEST(Xarm6OpenTableTest, CanonicalConfigurationIsSixDimensional) {
  const OimTParams params = LoadAndValidateConfig(kConfig);
  EXPECT_EQ(params.scenario_name, "open_table");
  ASSERT_EQ(params.robot.controlled_joints.size(), 6);
  EXPECT_EQ(params.robot.controlled_joints.back(), "xarm6_joint6");
  EXPECT_EQ(params.robot.home_positions.size(), 6);
  EXPECT_EQ(params.robot.effort_limits.size(), 6);
  EXPECT_EQ(params.robot.velocity_limits.size(), 6);
  EXPECT_EQ(params.robot.velocity_servo_gains.size(), 6);
  EXPECT_EQ(params.robot.passive_stiffness.size(), 6);
  EXPECT_DOUBLE_EQ(params.robot.effort_limits[5], 20.0);
  EXPECT_DOUBLE_EQ(params.robot.velocity_limits[5], 0.5);
}

TEST(Xarm6OpenTableTest, PlantActuationAndWristAxisAreConsistent) {
  const OimTParams params = LoadAndValidateConfig(kConfig);
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ASSERT_NO_THROW(ValidateXarmPlant(plant, params));
  EXPECT_EQ(plant.num_positions(), 6);
  EXPECT_EQ(plant.num_velocities(), 6);
  EXPECT_EQ(plant.num_actuators(), 6);
  EXPECT_EQ(plant.MakeActuationMatrix().rows(), 6);
  EXPECT_EQ(plant.MakeActuationMatrix().cols(), 6);
  EXPECT_EQ(plant.MakeActuationMatrix().fullPivLu().rank(), 6);

  auto context = plant.CreateDefaultContext();
  SetXarmHome(params, plant, context.get());
  const auto& end_effector =
      plant.GetBodyByName(params.robot.end_effector_body);
  const auto X_WEe_home =
      plant.EvalBodyPoseInWorld(*context, end_effector);
  const Eigen::Vector3d tip_home =
      X_WEe_home * params.robot.end_effector_point;
  const Eigen::Vector3d axis_home =
      X_WEe_home.rotation() * Eigen::Vector3d::UnitZ();

  plant.GetJointByName("xarm6_joint6")
      .SetPositions(context.get(), Eigen::VectorXd::Constant(1, 0.5));
  const auto X_WEe_rolled =
      plant.EvalBodyPoseInWorld(*context, end_effector);
  const Eigen::Vector3d tip_rolled =
      X_WEe_rolled * params.robot.end_effector_point;
  const Eigen::Vector3d axis_rolled =
      X_WEe_rolled.rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(tip_rolled.isApprox(tip_home, 1.0e-12));
  EXPECT_TRUE(axis_rolled.isApprox(axis_home, 1.0e-12));
  EXPECT_FALSE(X_WEe_rolled.rotation().matrix().isApprox(
      X_WEe_home.rotation().matrix(), 1.0e-6));
}

}  // namespace
}  // namespace dairlib::oim
