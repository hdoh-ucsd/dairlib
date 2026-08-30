#include <stdexcept>
#include <string>

#include <drake/common/yaml/yaml_io.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/parameter_headers/oim_t_params.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
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
  const OimTParams params = drake::yaml::LoadYamlFile<OimTParams>(FLAGS_config);
  const int joints = static_cast<int>(params.robot.controlled_joints.size());
  if (joints != 5) throw std::runtime_error("OIM xArm must have five controlled joints");
  DemandSize(params.robot.home_positions, joints, "home_positions");
  DemandSize(params.robot.effort_limits, joints, "effort_limits");
  DemandSize(params.robot.velocity_limits, joints, "velocity_limits");
  DemandSize(params.robot.velocity_servo_gains, joints,
             "velocity_servo_gains");
  if (params.task.execution_time_step != params.simulation.time_step) {
    throw std::runtime_error("task and simulation execution time steps differ");
  }
  if (params.object.model_instance != params.object.body) {
    throw std::runtime_error("initial OIM-T import requires model instance and body names to match");
  }
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) { return DoMain(argc, argv); }

