# Native OIM-T xArm C3+ architecture

`parameters/oim_t.yaml` is the only task-level source of truth. The legacy
`sim_params.yaml`, `goal_params.yaml`, and
`sampling_c3_controller_params.yaml` are not inputs to the xArm processes.

```text
                       parameters/oim_t.yaml
                 (robot + sim + task + routing)
                                  |
              +-------------------+-------------------+
              |                   |                   |
              v                   v                   v
         xarm_sim          xarm_osc_controller   xarm_sampling_c3_controller
              |                   ^                   |
              | XARM_STATE       | XARM_INPUT        | OIM_XARM_TRAJECTORY
              +------------------>|<------------------+
              |
              +---- OIM_T_STATE --------------------->+

 xArm execution plant        reduced-order planner plant
 --------------------        ---------------------------
 five controlled joints      floating point end effector
 rigid wrist/tool             one floating OIM T
 physical table/contact       synthetic support contacts
 2 ms integration             C3+ planning time step
```

## Ownership boundary

- OIM owns the xArm model, T geometry, start and goal, table, time steps, and
  success tolerances.
- DAIRLab owns Sampling-C3+ mode selection, LCS construction, consensus solve,
  trajectory messages, and OSC infrastructure.
- `xarm_sim` owns physical integration and publishes xArm and T state.
- `xarm_sampling_c3_controller` converts xArm/T observations into the existing
  reduced-order C3+ state and publishes Cartesian trajectories.
- `xarm_osc_controller` converts those trajectories to five actuator torques.
  It must not retain Panda joint names, seven-DOF assumptions, or Panda limits.

## Validation gates

1. The canonical YAML loads and all vector dimensions agree.
2. The xArm MJCF loads with five explicitly recreated actuators.
3. Start and goal match OIM without the historical DAIRLab frame warp.
4. State messages contain five positions, five velocities, and five efforts.
5. OSC holds home and respects the configured per-joint effort box.
6. Sampling-C3+ reaches its first solve without a workspace assertion.
7. A deterministic short rollout records commit, dirty state, YAML checksum,
   command, seed, dependency versions, and output location.
8. Only after gates 1-7 pass may the Python port consume this YAML.

## Current status

The clean branch now contains the three named native process entry points and
the narrowly vendored open-table OIM-T model. Each entry point loads the one
canonical YAML, imports the xArm plant, recreates the five actuators omitted by
the pinned Drake MJCF parser, and rejects a plant that violates the five-joint
contract. `xarm_sim` additionally initializes and advances the physical scene.

The compatibility layer restores the ignored joint-4 spring, configured joint
velocity and actuator effort limits, and xArm gravity compensation. A one-second
static hold has zero measured joint drift and leaves the T at its configured
rest height.

The first closed-loop transport gate is also connected. `xarm_sim --lcm=true`
publishes only the five selected xArm joints on `XARM_STATE_SIMULATION`, accepts
five named actuator torques on `XARM_INPUT_SIMULATION`, and publishes the native
Drake T state separately. `xarm_osc_controller` consumes each state message,
recomputes gravity torque from that observed state, clips it to the configured
effort box, and sends it back to simulation. A two-second run completed with
0.000769446 rad maximum joint drift. The T is now a dedicated SDF model and the
table is registered natively in Drake, avoiding the MJCF parser's broken
multi-floating-body model-instance boundary.

The processes intentionally do not disguise the Franka binaries as xArm
implementations. The pinned parser still reports unsupported features in the
robot MJCF. The native T/table path removes the scenario-level collision-group
and explicit-pair warnings, but rolling/torsional friction remains outside
Drake's point-contact model and must be treated as a model difference.

The next gate adds translational task-space tracking using the unchanged
DAIRLab gains `Kp = 200 I` and `Kd = 20 I`. The controller evaluates the
configured tip point, computes its translational Jacobian, adds
`J' (Kp e - Kd v)` to gravity compensation, and clips the result to the xArm
effort box. In a five-second three-process test, a +10 mm x request produced a
+7.604 mm tip displacement.

`xarm_sampling_c3_controller --first_solve_only` completes one native C3+
solve over a one-contact linearization. The contact gap uses the exact OIM T
minimum-y boundary (-79.4 mm), the measured 5.55 mm pusher radius, physical
T mass 0.1 kg, configured start/goal, five-knot horizon, 0.05 s planning step,
and three ADMM iterations.

`xarm_sampling_c3_controller --live_sampled_plan=true` is the next closed-loop
checkpoint. It consumes the live xArm tip and native T pose, transforms eight
samples on the exact two-box T boundary into world coordinates, solves one
one-contact C3+ candidate per sample, selects the finite candidate with the
smallest predicted object-position error, and publishes a step-limited pusher
target to the task-space tracker. `--live_control_steps` repeats that operation
from refreshed state at `--live_step_period_ms`. This is not yet a complete
contact-acquisition policy: the separated one-contact LCS has no tangential
alignment objective, and its complementarity force is inactive while the
normal gap is positive. It is also not yet a rotational or multi-contact
pushing policy.
