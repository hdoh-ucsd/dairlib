#pragma once

#include <string>

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/sampling_c3/parameter_headers/oim_t_params.h"

namespace dairlib::oim {

OimTParams LoadAndValidateConfig(const std::string& path);
void AddXarmActuators(const OimTParams& params,
                      drake::multibody::MultibodyPlant<double>* plant);
void ValidateXarmPlant(const drake::multibody::MultibodyPlant<double>& plant,
                       const OimTParams& params);
void SetXarmHome(const OimTParams& params,
                 const drake::multibody::MultibodyPlant<double>& plant,
                 drake::systems::Context<double>* context);

}  // namespace dairlib::oim
