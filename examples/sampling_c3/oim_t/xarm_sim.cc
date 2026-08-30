#include <limits>

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
  simulator.set_target_realtime_rate(params.simulation.realtime_rate);
  simulator.Initialize();
  if (FLAGS_duration > 0.0) simulator.AdvanceTo(FLAGS_duration);
  return 0;
}
