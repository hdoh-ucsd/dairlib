# Native xArm6 controller migration — 2026-08-31

## Scope and preserved baseline

This change replaces the OIM native arm's welded wrist-roll model with a true
six-joint xArm6. It does not alter the C3/C3+ horizon, ADMM settings, contact
parameters, task tolerances, OSC Cartesian gains, random seeds, or the default
`reduced_exact_t` planner mode. Earlier five-DOF result logs remain immutable.

Base commit: `f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce`

The sixth joint definition follows UFACTORY's xArm6 URDF: revolute wrist roll,
axis `[0, 0, 1]`, range `[-2 pi, 2 pi]`, and effort limit `20 N m`. The OIM
safety ceiling remains `0.5 rad/s`, matching the other five joints. The new
servo gain is `200`, matching the existing wrist actuator group; the `20 N m`
effort clamp remains authoritative.

## Fix plan and gates

| Gate | Requirement | Result |
|---|---|---|
| 1 | MJCF exposes `xarm6_joint6` and the native layer recreates six actuators | PASS |
| 2 | Canonical YAML contains six names, homes, effort/velocity limits, servo gains, and passive stiffness entries | PASS |
| 3 | Config and plant validation reject non-six-joint xArm6 layouts | PASS |
| 4 | Simulator publishes and accepts six-dimensional state/effort messages | PASS |
| 5 | OSC matrices, posture targets, servo bridge, and trajectory datatypes are dimensioned from all six joints | PASS |
| 6 | Primary OSC actively regulates wrist roll; descent and repositioning track all six joints | PASS |
| 7 | Four native executables build and focused Full Sampling-C3+ regression passes | PASS |
| 8 | Three-process Cartesian smoke test moves the tip while wrist roll remains bounded | PASS |
| 9 | Re-run collision-aware acquisition and measured contact gates with six-DOF receipts | TODO |
| 10 | Resume Full Sampling-C3+ task cycles and unchanged-tolerance `open_table` terminal rollout | TODO |

## Validation

Build and focused test:

```sh
bazel build --jobs=8 \
  //examples/sampling_c3/oim_t:oim_t_config_check \
  //examples/sampling_c3/oim_t:xarm6_sim \
  //examples/sampling_c3/oim_t:xarm6_osc_controller \
  //examples/sampling_c3/oim_t:xarm6_sampling_c3_controller
bazel test --test_output=errors \
  //examples/sampling_c3/oim_t:xarm6_full_sampling_c3plus_test
```

Results:

```text
four native targets:             PASS
configuration validation:        PASS
focused Full Sampling-C3+ test:  PASS
startup joint positions:         [0, -pi/4, -pi/4, 0, pi/2, 0]
startup joint drift infinity:    0
startup tip W:                   [0.254967, -0.000911, 0.334228] m
```

Closed-loop `+10 mm` x smoke result:

```text
result directory:
  /root/push_anything_ADMM/results/xarm6_wrist_control_smoke_ABzshN
planner / simulator status:      0 / 0
OSC status:                      124 (expected external timeout after sim)
final tip W:                     [0.262916, -0.000941057, 0.333666] m
measured x motion:               +7.949 mm
final joint 6:                   -6.293e-06 rad
maximum joint displacement:      0.030072 rad
physical contact episodes:       0
```

## Controller behavior

The primary OSC retains Cartesian translation and stick-axis tracking. A weak
one-joint posture task regulates wrist roll to its configured home value.
Descent and collision-aware repositioning already construct their joint-space
targets from `controlled_joints`, so they now track six coordinates without a
parallel controller implementation.

Joint 6 rotates about the axisymmetric pushing stick. It adds a real actuator
and removes a configuration/model mismatch, but it does not by itself change
the pusher capsule axis or solve a contact geometry failure. The next gate must
therefore re-measure IK, whole-capsule clearance, and contact response rather
than treating six actuators as proof of `open_table` success.
