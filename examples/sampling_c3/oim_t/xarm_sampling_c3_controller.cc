#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm_process_common.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = dairlib::oim::LoadAndValidateConfig(FLAGS_config);
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  dairlib::oim::AddXarmActuators(params, &plant);
  plant.Finalize();
  dairlib::oim::ValidateXarmPlant(plant, params);
  if (params.object.start_pose.isApprox(params.object.goal_pose)) {
    throw std::runtime_error("OIM-T start and goal must differ");
  }
  // The existing Sampling-C3+ solver remains the planner implementation;
  // this executable owns the xArm/OIM configuration boundary and must not
  // import the legacy Franka task-level YAMLs.
  return 0;
}
