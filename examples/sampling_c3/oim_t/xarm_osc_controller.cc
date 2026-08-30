#include <algorithm>
#include <memory>

#include <dairlib/lcmt_robot_input.hpp>
#include <dairlib/lcmt_robot_output.hpp>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <gflags/gflags.h>

#include "examples/sampling_c3/oim_t/xarm_process_common.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"

DEFINE_string(config, "examples/sampling_c3/oim_t/parameters/oim_t.yaml",
              "Canonical OIM-T configuration");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0", "LCM URL");

namespace dairlib::oim {

class XarmGravityController final : public drake::systems::LeafSystem<double> {
 public:
  XarmGravityController(const drake::multibody::MultibodyPlant<double>& plant,
                        const OimTParams& params)
      : plant_(plant), context_(plant.CreateDefaultContext()),
        effort_limits_(params.robot.effort_limits) {
    this->DeclareVectorInputPort(
        "xarm_state", systems::OutputVector<double>(
                          plant.num_positions(), plant.num_velocities(),
                          plant.num_actuators()));
    this->DeclareVectorOutputPort(
        "xarm_torque", systems::TimestampedVector<double>(plant.num_actuators()),
        &XarmGravityController::CalcTorque);
  }

 private:
  void CalcTorque(const drake::systems::Context<double>& context,
                  systems::TimestampedVector<double>* output) const {
    const auto* state = dynamic_cast<const systems::OutputVector<double>*>(
        this->EvalVectorInput(context, 0));
    plant_.SetPositionsAndVelocities(context_.get(), state->GetState());
    Eigen::VectorXd torque = -plant_.MakeActuationMatrix().transpose() *
        plant_.CalcGravityGeneralizedForces(*context_);
    for (int i = 0; i < torque.size(); ++i) {
      torque[i] = std::clamp(torque[i], -effort_limits_[i], effort_limits_[i]);
    }
    output->SetDataVector(torque);
    output->set_timestamp(state->get_timestamp());
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable std::unique_ptr<drake::systems::Context<double>> context_;
  Eigen::VectorXd effort_limits_;
};

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const OimTParams params = LoadAndValidateConfig(FLAGS_config);
  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModels(params.robot.model);
  AddXarmActuators(params, &plant);
  plant.Finalize();
  ValidateXarmPlant(plant, params);

  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);
  drake::systems::DiagramBuilder<double> builder;
  auto* state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<
          dairlib::lcmt_robot_output>(params.lcm.robot_state_channel, &lcm));
  auto* state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto* controller = builder.AddSystem<XarmGravityController>(plant, params);
  auto* command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);
  auto* command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          params.lcm.robot_input_channel, &lcm,
          drake::systems::TriggerTypeSet({drake::systems::TriggerType::kForced})));
  builder.Connect(*state_sub, *state_receiver);
  builder.Connect(state_receiver->get_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), command_sender->get_input_port());
  builder.Connect(command_sender->get_output_port(), command_pub->get_input_port());

  std::shared_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
  diagram->set_name("oim_xarm_gravity_controller");
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, diagram, state_receiver, params.lcm.robot_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib::oim

int main(int argc, char* argv[]) { return dairlib::oim::DoMain(argc, argv); }
