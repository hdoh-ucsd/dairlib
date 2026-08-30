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
  auto context = plant.CreateDefaultContext();
  dairlib::oim::SetXarmHome(params, plant, context.get());
  // This process boundary deliberately validates the xArm execution plant
  // without inheriting Panda joint objectives or seven-DOF assumptions.
  return 0;
}
