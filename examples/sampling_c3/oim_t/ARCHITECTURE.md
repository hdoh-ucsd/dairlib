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

The clean branch and canonical schema exist. The three native executables and
vendored model targets remain to be implemented. Existing Franka binaries are
unchanged and are not aliases for the xArm processes.

