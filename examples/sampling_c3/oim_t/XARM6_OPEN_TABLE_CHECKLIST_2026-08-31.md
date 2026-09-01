# xArm6 `open_table` execution checklist — 2026-08-31

This checklist audits the native C++ path at source commit
`f5c53479c6b7bb9e0a5d58d29af2c6c2b10f8bce` with a dirty worktree containing
the cumulative OIM/xArm implementation.  No solver setting, dynamics/contact
parameter, gain, horizon, random seed, task tolerance, or success criterion was
changed during this audit.

Canonical configuration:
`examples/sampling_c3/oim_t/parameters/oim_t.yaml`, SHA-256
`bc1b710fe705cc1b05850f9426b000c987c40c0767d69cf020fc94a13898d68e`.

## Code and model checklist

- [x] The MJCF has six revolute joints and six actuators. Joint 6 is the
  axisymmetric-stick wrist roll.
- [x] YAML joint names, home positions, effort limits, velocity limits, servo
  gains, and passive stiffness all have six ordered finite entries.
- [x] Configuration validation requires `open_table`, exact ordered joint names,
  positive actuator limits/gains, and nonnegative passive stiffness.
- [x] Plant validation requires one-position/one-velocity revolute joints and
  verifies that every actuator owns its configured joint.
- [x] Drake simulation publishes and accepts all six arm channels. Pusher
  capsule telemetry uses the configured radius and endpoint.
- [x] OSC tracks the configured stick endpoint, conditions the stick axis, and
  regulates joint 6 by name instead of by a hard-coded array index.
- [x] Differential IK, collision-aware posture IK, and whole-capsule swept
  checks use dynamic six-joint vectors.
- [x] The robot-only xArm6 plant has 6 positions, 6 velocities, 6 actuators,
  and a full-rank 6-by-6 actuation matrix.
- [x] A regression proves joint-6 roll changes flange orientation without
  changing the axisymmetric pusher tip or axis.
- [x] Release/lift waypoints use position-only IK; collision-checked traverse
  owns reorientation. This prevents joint-limited orientation motion from
  carrying the tip away from a release waypoint.
- [x] Collision-aware posture trajectories now append a zero-slope terminal
  segment. A completed command no longer extrapolates joint velocity during a
  contact dwell.
- [x] Release noise remains non-descending inside the existing 3 mm activation
  band, while a larger lift overshoot can be corrected by the existing
  collision-checked IK path.
- [x] Full Sampling-C3+ reports the unchanged global `open_table` translation
  and orientation acceptance explicitly; a productive local cycle is no
  longer sufficient for terminal success.
- [x] Rejected live contact IK and other local-cycle failures publish a fresh
  measured-state task/posture hold before raising an error.

## Verification checklist

- [x] All four native processes build.
- [x] `xarm6_open_table_test` passes.
- [x] `xarm6_full_sampling_c3plus_test` passes.
- [x] Configuration validation passes.
- [x] Reduced exact-T first-solve and deterministic contact witnesses remain
  unchanged.
- [x] A reduced 2,000-update LCM run preserves six-dimensional state/command
  handoff and stable joint-6 roll.
- [x] A full 2,000-update run completes collision-aware acquisition, first
  contact, measured lateral rejection, corrective-face contact, release,
  measured lateral recovery, progress resampling, a third physical contact,
  1,000-step productive dwell, and safe terminal hold.
- [x] The productive cycle moved the object by 8.3007 mm in goal-directed y and
  reduced yaw error by 0.14446 rad while keeping final lateral drift at
  2.28601 mm (5 mm limit).
- [ ] Terminal `open_table` succeeds. The measured terminal error after one
  cycle is 0.790798 m and 2.98282 rad, outside the unchanged 0.05 m and 0.10 rad
  tolerances.
- [x] Receding Sampling-C3+ repeats from each measured object pose until
  terminal acceptance, a failed acceptance gate, or the execution budget. A
  measured run proved productive cycle 1, inter-cycle release, and cycle-2
  resampling/execution.
- [x] Candidate arbitration includes the live measured xArm6 posture/contact
  IK and whole-capsule receipt, falls through after static or physical
  execution failure, and quarantines the failed sample until the object pose
  changes.
- [x] Lateral recovery after a later receding cycle has a live-reachable
  corrective face. Live replay now matches physical bounded Cartesian
  stepping, and a measured run proved second-cycle contact, wrong-polarity
  rejection, corrective-face contact, clear, and 2.82219 mm recovered drift.
- [x] A nonproductive but safely corrected contact continues into a fresh
  measured-state solve instead of terminating the receding loop. One run
  proved four productive cycles and three recovery-only continuations.
- [x] A sample whose measured response exits the lateral corridor enters the
  pose-local live-execution quarantine.
- [x] Replenish the sampled progress buffer after a later-pose physical
  traverse rejection. Four explicit deterministic seed families provide 200
  candidates; two consecutive measured replans retained executable contacts.
- [x] Fall through to the next ranked recovery candidate when static replay
  passes but measured physical execution rejects final-contact IK. The
  measured retry changed from `mesh_crossbar_left_seeded_9` to
  `crossbar_left_seeded_4` and recovered lateral drift to 1.65903 mm.
- [x] Reject a corrective contact whose measured response worsens lateral
  error or has the wrong measured polarity, release it, and replan before it
  consumes the remaining execution budget.
- [x] Condition measured six-axis object velocity only at the planar LCS
  boundary (`wz`, `vx`, `vy` retained; `wx`, `wy`, `vz` constrained), while
  preserving the raw spatial velocity in experiment receipts.
- [x] Defer a cycle when the unchanged 1,000-step physical dwell cannot fit in
  the remaining execution budget, publish a measured hold, and return a
  classified exit code.
- [x] Bound every physical contact-engagement subtarget by the existing 2 mm
  minimum contact-normal step.
- [x] Reconstruct retry contact geometry from the current measured object pose,
  release the failed face, and fall through to the next ranked physical
  candidate.
- [x] Reject wrong-polarity and contact-loss responses during both initial and
  receding corrective dwell.
- [x] Make vertical release completion z-only and hold measured x/y on upward
  substeps, so settling drift cannot turn a lift into an inward scrape or a
  stale-horizontal-target deadlock.
- [x] Execute release/lift/anchor motion with bounded local differential IK.
  Every predicted joint step must reduce Cartesian error and pass the actual
  interpolated whole-capsule/table receipt.
- [x] Preserve the measured pusher axis during position-only release phases;
  the subsequent verticalization phase alone owns shaft rotation.
- [x] Route every post-contact progress and recovery reposition above the full
  T and through the neutral workspace before verticalization. A 5,000-step
  physical receipt proves three progress and three recovery anchor routes without
  fall-through.
- [x] Make every accepted full contact cycle reduce the measured global
  terminal-error metric. Predicted and measured receipts require translation
  and orientation to be nonregressive and at least one to meet its existing
  minimum-progress threshold. A 20,000-update physical rollout produced seven
  accepted Pareto-descent cycles and rejected four cycles whose yaw regressed.
- [x] Condition candidate ranking on accumulated measured translation,
  orientation, and lateral response for the current face/pose neighborhood.
  Physical evidence shows 42 nearby candidates demoted after two incompatible
  observations, selection of an unseen stem contact, a complete productive
  dwell, and the resulting compatible/unseen/incompatible partition.
- [x] Preserve an executable, live-IK-reachable candidate after a large
  measured yaw transition. Contact-feasible replenishment remains subject to
  live IK and whole-capsule execution, and high-anchor verticalization has a
  guarded home-seed fallback. An 8,000-update physical receipt completed five
  post-yaw replans and 20 progress live-IK passes without candidate exhaustion.
- [x] Admit progress only from a settled planar T, inside a 2 mm measured
  lateral reserve, and execute the longest full-Sampling-C3+ prefix predicted
  to remain inside the unchanged 5 mm corridor. A physical receipt completed
  four replans and three productive cycles after the prior zero-intersection
  failure.
- [x] Replenish contact-feasible candidates after downstream prefix/contact/
  response filtering, not only after an empty workspace batch. The physical
  branch activated twice, recovered 200 candidates, and selected seven then
  four plans that retained mandatory live IK and capsule checks.
- [ ] Reserve measured acquisition and recovery overhead before admitting a
  new 1,000-step contact dwell. The latest rollout admitted a cycle with 1,291
  updates remaining and exhausted the 8,000-step budget during recovery.

## Canonical evidence

Full-stack command used a private LCM URL and the unchanged 2,000-update budget:

```sh
xarm6_osc_controller --lcm_url='udpm://239.255.76.67:7783?ttl=0'
xarm6_sim --lcm_url='udpm://239.255.76.67:7783?ttl=0' \
  --duration=65 --lcm=true --gravity_hold=false
xarm6_sampling_c3_controller \
  --lcm_url='udpm://239.255.76.67:7783?ttl=0' \
  --planner_mode=full_sampling_c3plus --live_sampled_plan=true \
  --full_execution_steps=2000
```

Raw immutable working output:
`/root/push_anything_ADMM/results/xarm6_lift_overshoot_2000_gVHf8w`.
The simulator exited 0, the planner exited 0 before the explicit global
terminal gate was added, and the OSC was intentionally ended by its 75-second
process timeout. The simulator's independent `open_table_success=FAIL` is the
authoritative terminal result.

Post-terminal-gate failure-path evidence:
`/root/push_anything_ADMM/results/xarm6_terminal_gate_2000_gz8a6g`. This run
reached the progress-contact phase but correctly rejected a non-finite live IK
step. It motivated the measured-state hold-before-throw fix and the remaining
live-IK-conditioned candidate-fallback item above.

Controller-item evidence:

- `/root/push_anything_ADMM/results/xarm6_receding_recovery_5000_r3Nw0W`
  proves cycle 1 (`receding_cycle=PASS`), inter-cycle release
  (`receding_release=PASS`), and measured cycle-2 resampling.
- `/root/push_anything_ADMM/results/xarm6_release_before_recovery_5000_Z8ge3L`
  proves physical candidate quarantine/fall-through, release-before-recovery,
  live-IK-conditioned contact-only recovery selection, corrective contact, and
  corrective clear.
- `/root/push_anything_ADMM/results/xarm6_controller_items_final_3500_cnDSxy`
  is the final post-fix run. It safely releases on measured lateral rejection,
  rejects all 72 unreachable recovery candidates at live contact IK, publishes
  a measured-state hold, and preserves the unchanged terminal failure.
- `/root/push_anything_ADMM/results/xarm6_bounded_live_replay_5000_j3s88r`
  proves the bounded live-replay fix, a live-reachable second-cycle contact,
  wrong-polarity rejection, corrective-face contact, and 2.82219 mm recovery.
- `/root/push_anything_ADMM/results/xarm6_recovery_continuation_5000_5YFAmK`
  proves four productive cycles, three recovery-only continuations, and 18
  physical contact episodes before the external timeout.
- `/root/push_anything_ADMM/results/xarm6_response_quarantine_3000_b4UbB1`
  proves measured-response quarantine and exposes later-pose candidate
  exhaustion after a live traverse failure.
- `/root/push_anything_ADMM/results/xarm6_progress_replenishment_2000_lAHPvQ`
  proves 200-candidate progress replenishment and two nonempty consecutive
  replans; its next failure is measured recovery final-contact IK fall-through.
- `/root/push_anything_ADMM/results/xarm6_recovery_physical_fallthrough_repeat_2000_Jt4OVF`
  proves physical final-contact failure quarantine, measured-state replay,
  successor candidate acquisition, and 1.65903 mm recovered lateral drift.
- `/root/push_anything_ADMM/results/xarm6_bounded_contact_engagement_physical_5000_QHGyZU`
  proves bounded engagement, six productive cycles, and a complete 1,000-step
  physical dwell.
- `/root/push_anything_ADMM/results/xarm6_velocity_fallback_physical_20000_CeISun`
  proves 14 connected productive cycles and the measured/raw velocity receipt.
- `/root/push_anything_ADMM/results/xarm6_vertical_completion_physical_20000_CroKyA`
  proves the z-only vertical release gate and 11 cycles; its late unstable
  neutral transit is retained as negative evidence.
- `/root/push_anything_ADMM/results/xarm6_axis_consistent_anchor_physical_5000_h0Rn3w`
  proves two axis-consistent neutral-anchor transits and safe budget deferral.
- `/root/push_anything_ADMM/results/xarm6_local_diffik_anchor_physical_5000_Hw1PbG`
  proves six cycles using bounded local differential-IK release/lift steps.
- `/root/push_anything_ADMM/results/xarm6_local_diffik_terminal_physical_20000_Z2R2Ua`
  is retained negative evidence for direct far-reach verticalization and the
  physical fall-through it can cause.
- `/root/push_anything_ADMM/results/xarm6_mandatory_neutral_anchor_physical_5000_tGkCM7`
  proves mandatory high neutral routing for progress and recovery; the final
  tip remained above the table at z=0.241177 m when the budget expired.
- `/root/push_anything_ADMM/results/xarm6_corridor_prefix_gate_8000_9BC4hk`
  proves settled/reserve admission, corridor-safe receding prefixes, four
  progress replans, and three productive physical cycles.
- `/root/push_anything_ADMM/results/xarm6_postfilter_replenishment_8000_kj2Oc9`
  proves two post-filter contact-feasible replenishments and records the next
  measured-overhead-aware cycle-budget failure at update 8,000.

Repository-wide Python baseline after the changes: 398 passed, 34 failed,
matching the pre-existing dirty-tree baseline. Focused native Bazel tests:
4 passed.

Environment: Drake 1.51.1, Bazel 8.4.0, g++ 13.3.0. The next implementation
gate is measured-response-conditioned candidate ranking across receding
contact cycles; it is controller research work, not a missing xArm6
joint/model connection.
