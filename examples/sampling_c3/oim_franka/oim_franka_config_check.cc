#include <stdexcept>
#include <string>

#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_franka/franka_process_common.h"

DEFINE_string(config, "examples/sampling_c3/oim_franka/parameters/oim_franka.yaml",
              "Canonical OIM-T configuration to validate");

namespace {

void DemandSize(const Eigen::VectorXd& value, int expected,
                const std::string& name) {
  if (value.size() != expected) {
    throw std::runtime_error(name + " must contain " +
                             std::to_string(expected) + " values");
  }
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = dairlib::oim::LoadAndValidateConfig(FLAGS_config);
  const int joints = static_cast<int>(params.robot.controlled_joints.size());
  DemandSize(params.robot.passive_stiffness, joints, "passive_stiffness");
  if (params.object.model_instance != params.object.body) {
    throw std::runtime_error("initial OIM-T import requires model instance and body names to match");
  }
  if (params.full_sampling_c3plus.contact_model != "anitescu" ||
      params.full_sampling_c3plus.horizon != 5 ||
      params.full_sampling_c3plus.admm_iterations != 3 ||
      params.full_sampling_c3plus.num_friction_directions != 2) {
    throw std::runtime_error(
        "full Sampling-C3+ must retain the canonical Push-T structure");
  }
  DemandSize(params.full_sampling_c3plus.state_cost_diagonal, 19,
             "full_sampling_c3plus.state_cost_diagonal");
  DemandSize(params.full_sampling_c3plus.input_cost_diagonal, 3,
             "full_sampling_c3plus.input_cost_diagonal");
  DemandSize(params.full_sampling_c3plus.workspace_lower, 3,
             "full_sampling_c3plus.workspace_lower");
  DemandSize(params.full_sampling_c3plus.workspace_upper, 3,
             "full_sampling_c3plus.workspace_upper");
  DemandSize(params.full_sampling_c3plus.input_lower, 3,
             "full_sampling_c3plus.input_lower");
  DemandSize(params.full_sampling_c3plus.input_upper, 3,
             "full_sampling_c3plus.input_upper");
  DemandSize(params.full_sampling_c3plus.ee_velocity_lower, 3,
             "full_sampling_c3plus.ee_velocity_lower");
  DemandSize(params.full_sampling_c3plus.ee_velocity_upper, 3,
             "full_sampling_c3plus.ee_velocity_upper");
  if ((params.full_sampling_c3plus.workspace_lower.array() >=
       params.full_sampling_c3plus.workspace_upper.array()).any() ||
      (params.full_sampling_c3plus.input_lower.array() >=
       params.full_sampling_c3plus.input_upper.array()).any() ||
      (params.full_sampling_c3plus.ee_velocity_lower.array() >=
       params.full_sampling_c3plus.ee_velocity_upper.array()).any()) {
    throw std::runtime_error("full Sampling-C3+ bounds are invalid");
  }
  if (params.full_sampling_c3plus.perimeter_sample_count <= 0 ||
      params.full_sampling_c3plus.random_seed < 0 ||
      params.full_sampling_c3plus.mesh_sample_count <= 0 ||
      params.full_sampling_c3plus.mesh_random_seed < 0) {
    throw std::runtime_error("full Sampling-C3+ sampling settings are invalid");
  }
  if (params.full_sampling_c3plus.num_outer_threads <= 0) {
    throw std::runtime_error("full Sampling-C3+ outer thread count is invalid");
  }
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) { return DoMain(argc, argv); }
