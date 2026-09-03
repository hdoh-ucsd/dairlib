# Object/Goal Ablation of the Reference Sampling-C3+ Stack

**Date:** 2026-09-02 · **Branch:** `c3plus-object-ablation` · **Stack:** native C++
`franka_sim` + `franka_osc_controller` + `franka_sampling_c3_controller`
(Push-T demo wiring, sphere end effector r = 0.0195 m, Push-T costs untouched)

## Question

The OIM open_table task (small 0.1 kg T, long translation + π flip) does not
transfer to the reference C3+ method. Which object property is responsible —
the **goal**, the **shape/scale (inertia)**, or the **mass**?

## Method

Every experiment is a config/model variant only (`push_t_exp*` demo dirs); no
solver, cost, or controller code is modified. All experiments share one task:
start (0.5, 0.30, yaw 0) → goal (0.5, −0.30, yaw π), success thresholds
0.02 m / 0.1 rad (the reference stack's own). Trials terminate at first
success or a wall cap; metrics use the schema of
`aggregate_trial_stats` (steps = 10 Hz object-state samples; censored at the
cap for failures).

Two repairs were required to make the *stock* reference demo run at all
(commits `ed6c6310`, `c21819f6`):

1. The checked-in push_t parameter files had drifted ~30 keys behind the
   current C++ structs (reconciled from the `anything` demo's files).
2. The repos→C3 mode gate was geometrically unsatisfiable for push_t:
   `ee_z_close` requires EE z < `z_height + c3_min_clearance` = 0.006 m, but
   the floor (z = −0.0145) plus the sphere radius (0.0195) bounds the EE
   center to z ≥ ~0.005, a 1 mm window the tracking never enters — and
   `gen_planar_samples` projected reposition targets to z = −0.004, *inside*
   the floor. Measured: 4,522/4,522 controller steps in repositioning mode,
   zero C3-mode entries, object displacement < 1 µm in 116 s. Fix (config
   only): `ee_z_close: false` (push_t-era semantics). After the fix the stock
   control makes contact and pushes (12–77 C3-mode entries per run).

## Demo directory names

The original exp numbering is kept in result artifacts; the repo demo dirs
now carry task-specific names:

| Old | New | Task |
|---|---|---|
| `push_t_exp1` | `push_t_nativeT_oimgoal` | native 1 kg T, OIM-style 0.6 m + π goal |
| `push_t_exp2` | `push_t_oimdimsT_1kg` | OIM-dimension T at 1 kg, OIM-style goal |
| `push_t_exp3m05` | `push_t_nativeT_m005` | native-shape T at 0.05 kg, OIM-style goal |

(`push_t_exp0` control, `push_t_exp4` rule-Q, and the `push_t_exp5*`
light-OIM-dims pair keep their names; `--demo_name` accepts any `push_t*`.)

## Objects

| Variant | Boxes (m) | Mass | Izz vs native |
|---|---|---|---|
| native `push_t` | 2 × 0.16×0.04×0.04 (bbox 0.24×0.16×0.04) | 1.0 kg | 1× |
| `push_t_oimscale` (exp2) | OIM dims: 0.089×0.0198×0.0596 + 0.0198×0.0794×0.0596 | 1.0 kg | inertia from boxes |
| `push_t_m05` (exp3) | native shape | **0.05 kg** | ×0.05 |
| `push_t_Tx` | world-x scaled ×0.371 (→ OIM x-extent 0.089) | 1.0 kg | recomputed |
| `push_t_Ty` | world-y scaled ×0.620 (→ OIM y-extent 0.0992) | 1.0 kg | recomputed |
| `push_t_Tz` | world-z scaled ×1.49 (→ OIM height 0.0596) | 1.0 kg | recomputed |

## Results

### Single 600 s-wall screening runs (~195 s sim)

| Run | Final pos err (m) | Final ang err (rad) | Success |
|---|---|---|---|
| exp0 control (stock goal: 0.19 m / 0.74 rad) | 0.159 | 0.012 | ✗ (rotation done, translation unfinished) |
| exp1 native T + OIM goal | 0.356 | 0.275 | ✗ (large progress: ~0.24 m, 2.87 rad) |
| exp2 OIM-dimension T | 0.532 | 2.219 | ✗ (worst) |
| exp3 0.05 kg T | 0.024 | 0.051 | **✓ t = 69 s** |

### Success-terminated trial statistics

`c3ab_<name>_trials.jsonl` ledgers; aggregates in `c3ab_<name>_trials_result.json`.

**exp3 — 0.05 kg native-shape T (n = 5, cap ≈ 650–720 s sim):**

```json
{"n_trials": 5, "success_rate": 0.4,
 "pos_err_mean": 0.2644, "pos_err_std": 0.2266,
 "pos_err_mean_success": 0.0186, "pos_err_std_success": 0.0003,
 "theta_err_mean": 0.8620, "theta_err_std": 1.0651,
 "theta_err_mean_success": 0.0326, "theta_err_std_success": 0.0188,
 "mean_execution_time": 427.9, "std_execution_time": 303.5,
 "mean_steps_to_goal": 4279.4, "std_steps_to_goal": 3034.6,
 "mean_steps_to_goal_success": 576.5}
```

Successes at 47.7 s and 67.6 s sim; two near-miss failures (~0.32–0.35 m,
0.6–0.7 rad remaining, still progressing at cap) and one stalled draw.

**exp1 — native 1 kg T + OIM goal (n = 5, cap ≈ 1300–1420 s sim): 0/5.**

```json
{"n_trials": 5, "success_rate": 0.0,
 "pos_err_mean": 0.382, "pos_err_std": 0.046,
 "theta_err_mean": 1.092, "theta_err_std": 0.781,
 "mean_execution_time": 1328.6, "mean_steps_to_goal": 13285.8}
```

Answer to "does the native T succeed on OIM goals?": **no** — every trial
progresses (two finish the rotation, θ ≈ 0.20), and all five stall at a
consistent 0.34–0.46 m translation residual around mid-path (y ≈ 0).

**Axis-matched variants (n = 3 each, 1 kg, cap ≈ 600 s sim) — all 0/3:**

| Variant | pos_err mean±std | theta_err mean±std | worst draw |
|---|---|---|---|
| T_x (×0.371) | 0.499 ± 0.082 | 2.138 ± 0.915 | 0.600 / 3.142 (zero progress) |
| T_y (×0.620) | 0.406 ± 0.031 | **1.179 ± 0.418** | 0.441 / 1.663 |
| T_z (×1.49)  | 0.451 ± 0.020 | 1.982 ± 0.149 | 0.468 / 2.096 |

## Principled Q/R generating rule — derivation validated, intervention refuted

A generating rule was proposed: Q_obj ∝ blkdiag(I_obj, m·I₃) (so
q_θ/q_pos = ρ_g²), q/r ∝ m^α (α∈[1,2] by force headroom), R fixed from
u_max, q_pos from task tolerance.

**Structural validation.** Native Push-T: ρ_g = 0.069 m, ρ_g² = 0.00477 →
commensurate quaternion weight = q_pos·ρ_g² = 200×0.00477 = **0.95 ≈ 1**,
exactly as the rule derives; the shipped
`q_quaternion_dependent_weight: 1000` is a deliberate ×1049 orientation
"task statement". The OIM-geometry T (both 1 kg and the true 0.1 kg — they
are geometrically similar) has ρ_g = 0.0352 m, ρ_g² = 0.00124, giving a
rule-preserving rescale of quat 1000 → **260** and object angular-velocity
weight 0.05 → **0.013**. Mass: exp3 (0.05 kg, unchanged w_Q, 2/5 success)
empirically supports α → 0 when force headroom is large; R untouched.

**Matched-protocol test (exp4 vs exp2 control; OIM-dims T, n = 3 each,
≈ 710 s sim, only the two weights differ):**

| Config | pos_err | theta_err |
|---|---|---|
| exp2 control (native Q, quat 1000) | 0.436 ± 0.029 | **0.545 ± 0.082** |
| exp4 (rule Q, quat 260, ω 0.013)   | 0.457 ± 0.057 | 1.112 ± 0.725 |

The rescale did **not** help: rotation got worse and less consistent, and
translation was unchanged. (The 2.22 rad exp2 screening figure that
motivated the test was a short-window draw artifact — hence the matched
control.) Reading: the ×1049 overweight is load-bearing rotation *drive*
at every scale, not native-scale tuning noise; and rotation is not the
binding constraint anyway — **translation is**: native-Q small T, rule-Q
small T, and the native big T all stall at ≈ 0.34–0.46 m residual on the
0.6 m goal.

## exp5 — OIM-dimension T at 0.05 kg: the goal is reachable

The fully axis-aligned OIM-geometry T (0.089×0.0992×0.0596) at 0.05 kg
(masses/inertias ×0.05), success-terminated trials, n = 3 per Q config:

| Config | Successes | success times (sim s) | failed draw |
|---|---|---|---|
| `push_t_exp5rule` (rule Q: quat 260, ω 0.013) | **3/3** | 57.5, 95.5, 97.2 | — |
| `push_t_exp5nat` (native Q: quat 1000, ω 0.05) | 2/3 | 46.4, 97.5 | 0.47 m / 1.34 rad |

First configuration in the study where the OIM *geometry* reaches the goal
— and it does so reliably. Attribution: **mass is the primary rescuer**
(the native-Q control also mostly succeeds at 0.05 kg, where the same
geometry was 0/6 at 1 kg across both Q configs). The rule-derived Q is
equal-or-better at light mass (3/3 vs 2/3; n too small for significance)
— the opposite sign of its effect at 1 kg — consistent with the rule's own
caveat: the commensurate weights are right when force headroom is large,
while heavy objects need the extra orientation drive of the ×1049 task
statement.

Implication for the true OIM T (0.1 kg, same geometry): it sits between
the 0.05 kg (works) and 1 kg (fails) regimes, much closer to 0.05 kg; the
reference stack with either Q is likely capable, with the rule-derived Q
the principled default.

## Settled: the true OIM T, and the gyration-commensurate Q

**Terminology.** The "rule Q" is the **scaledQ** (short for the
gyration-commensurate Q; `scaledQ` in demo names, historical ledgers say
`commQ`/`ruleQ`): object orientation weight scaled so
q_θ/q_pos = ρ_g² relative to the tuned native baseline
(quat 1000 → 260, object angular-velocity 0.05 → 0.013 for the OIM
geometry; w_Q and R unchanged). Demo dirs renamed accordingly
(`push_t_oimdimsT_1kg_scaledQ`, `push_t_oimdimsT_m005_{nativeQ,scaledQ}`,
`push_t_oimT_m01_{nativeQ,scaledQ}`).

**The settling experiment.** The *true* OIM T — exact OIM dimensions AND
exact OIM mass 0.1 kg (link masses 0.0529/0.0471, inertias ×0.1) — on the
OIM-style goal, n = 5 success-terminated trials per Q arm:

| Arm | success_rate | success times (sim s) | success errors (m / rad) | mean_steps_to_goal_success |
|---|---|---|---|---|
| **commensurateQ** | **4/5** | 56.0, 63.5, 82.8, 121.3 | 0.0155 ± 0.0034 / 0.0385 ± 0.0222 | 809 |
| nativeQ | 3/5 | 78.3, 82.9, 98.7 | 0.0138 ± 0.0066 / 0.0503 ± 0.0300 | 866 |

Full-schema aggregates: `c3ab_oimT_m01_{commQ,natQ}_trials_result.json`.

**Finding.** The reference C3+ stack solves the true OIM T task reliably at
its real mass; the earlier total failure of the OIM geometry was a 1 kg
artifact. Between the Q designs, success speed is indistinguishable
(≈ 56–121 s either way); the difference is **reliability**. Pooled over the
two light-mass OIM-geometry objects (0.05 + 0.1 kg):

| Arm | pooled successes |
|---|---|
| commensurateQ | **7/8** |
| nativeQ | 5/8 |

Each nativeQ failure is a hard stall; commensurateQ had one. Combined with
the 1 kg matched control (where commensurateQ *hurt* rotation:
θ 1.11 ± 0.72 vs 0.55 ± 0.08), the picture is a clean interaction that the
generating rule itself predicts via its force-headroom caveat:

- **ample force headroom (light object):** commensurate weights suffice and
  remove the over-driven-rotation stall mode — equal speed, better
  reliability; use **commensurateQ**.
- **scarce headroom (heavy object):** the ×1049 orientation task-statement
  overweight is load-bearing drive; keep **nativeQ** — though neither Q
  succeeds there, as translation throughput binds first.

n = 3–5 per cell: directions are consistent across three object masses,
but individual comparisons are not statistically significant.

### Failure post-mortems (true OIM T, n=5 arms)

Per-trial time/steps to goal (10 Hz steps): scaledQ successes 56.0 s/560,
63.5 s/635, 82.8 s/828, 121.3 s/1213; nativeQ successes 78.3 s/783,
82.9 s/829, 98.7 s/987.

- **scaledQ trial 3 — inner-workspace trap (geometric, not Q):** the first
  push drove the T diagonally to (0.273, −0.03), radius 0.276 from the
  base. Pushing it back toward the goal requires the EE on the base side
  of the object at radius ≈ 0.21 — inside the `robot_radius_limits`
  inner bound of 0.25 — so no admissible contact sample exists; the
  planner repositioned 172 times while the object crept < 2 mm and
  0.02 rad in 10 minutes. Mitigations live in sampling/workspace config
  (inner radius, or keeping early pushes off the base direction), not Q.
- **nativeQ trial 3 — mid-path translation stall:** rotation complete
  (θ 0.086) at healthy radius (x = 0.425) but y-translation stopped at
  −0.03 of −0.30 — the same translation-throughput signature as the 1 kg
  native T. nativeQ trial 4 never composed a push sequence.

## Conclusions (final)

1. **The OIM-style goal is not a blocker.** Every capable object makes
   consistent progress on the 0.6 m + π task; the true OIM T completes it
   in 56–121 s of sim time.
2. **Mass dominates.** Lighter is strictly easier: the 0.05 kg native-shape
   T succeeds where the 1 kg native T never does (0/5, stalling ≈ 0.35 m
   short on translation), and the OIM geometry flips from 0/6 at 1 kg to
   7/8 pooled at 0.05–0.1 kg.
3. **The early "geometry is the blocker" reading was a mass confound.**
   exp2 and the axis-matched Tx/Ty/Tz variants shrank the T while keeping
   1 kg — up to ~10× the true OIM density — so per-feature friction load,
   not shape, drove their failures. At its true 0.1 kg mass the exact OIM
   geometry is reliably solvable by the unmodified reference stack.
4. **Q design is a second-order reliability effect with a clean
   force-headroom interaction.** Success speeds are indistinguishable
   between native Q and the gyration-commensurate Q; at light mass the
   commensurate weights remove hard-stall draws (pooled 7/8 vs 5/8), while
   at 1 kg they hurt rotation (θ 1.11 ± 0.72 vs 0.55 ± 0.08). Use
   commensurateQ when force headroom is ample; keep the native
   orientation-drive overweight when it is not.
5. **For heavy objects on long goals, translation throughput binds first**
   (≈ 0.4 m residual across all 1 kg configs and both Q designs); no Q
   rebalance addresses it. That — push cadence and reposition overhead —
   is the frontier if 1 kg-class objects ever matter.
6. **Recommended OIM configuration:** `push_t_oimT_m01_scaledQ` —
   true OIM T (0.1 kg) with the gyration-commensurate Q — 4/5 success,
   809 mean steps-to-goal on successes.

## Artifacts & reproduction

- Results + videos (organized 2026-09-03): everything under
  `D:\projects\ERL\push_anything_ADMM\results\c3ab_ablation_study\` and
  `/root/push_anything_ADMM/results/c3ab_ablation_study/`; feature-named
  video collection (45 clips) inside as `c3ab_video_collection_20260903/`.
  Superseded pre-z-gate-fix iterations were erased; the stock-baseline
  receipt `c3ab_exp0_stock_baseline/` is retained as bug evidence.
- Ledgers (`.jsonl` + full-schema `_result.json`):
  `c3ab_{exp1,exp3,expTx,expTy,expTz,exp2_ctrl,exp4_ruleQ,exp5nat,exp5rule,
  oimT_m01_natQ,oimT_m01_commQ}_trials`.
- Harness: success-terminated trial runner (`run_c3plus_trials.sh`; three
  binaries per lane on isolated LCM ports, parallel lanes), pydrake-LCM
  recorder (`record_full_state.py`), replay renderer
  (`render_c3plus_run.py`); copies live with the session receipts.
- One trial of the recommended configuration:
  `franka_{osc_controller,sampling_c3_controller,sim}
  --demo_name=push_t_oimT_m01_scaledQ
  --lcm_url=udpm://239.255.76.67:7991?ttl=0`.

*Report from the 2026-09-02/03 ablation sessions. Nondeterminism caveat:
the stack varies run-to-run (threading/timing), so all n are independent
draws, not seeds; per-cell n = 3–5 — directions are consistent across
masses, individual comparisons not statistically significant.*
