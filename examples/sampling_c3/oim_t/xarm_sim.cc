#include <iostream>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm_process_common.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_double(duration, 0.0, "Seconds to simulate; zero performs a startup check");
DEFINE_bool(gravity_hold, true,
            "Apply the source model's gravcomp=1 semantics to the xArm joints");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = dairlib::oim::LoadAndValidateConfig(FLAGS_config);
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, params.simulation.time_step);
  drake::multibody::Parser(&plant, &scene_graph).AddModels(params.simulation.scene_model);
  dairlib::oim::AddXarmActuators(params, &plant);
  plant.Finalize();
  dairlib::oim::ValidateXarmPlant(plant, params);
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
  if (FLAGS_gravity_hold) {
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
