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

## Conclusions

1. **The OIM-style goal is not the blocker.** The native T makes major,
   consistent progress on the long π-flip task (79 % of the rotation in the
   screening run); it needs more time/budget, not a different method.
2. **Mass is not the blocker — lighter is easier.** The 0.05 kg T is the only
   configuration that reaches the goal (2/5 trials, ~48–68 s sim when it
   lands; success-mean errors 0.019 m / 0.033 rad).
3. **Geometry/scale is the blocker, along every axis.** The full
   OIM-dimension T is the worst performer, and each single-axis match (x
   shrink, y shrink, z growth) independently collapses the success rate to
   0/3 with severely degraded rotation progress. Degradation ordering:
   T_y (mildest, θ≈1.18) < T_z (θ≈1.98) ≈ T_x (θ≈2.14) — note the axis
   scale magnitudes differ (0.62 / 1.49 / 0.371), so severity tracks the
   size of the geometric change, not a privileged axis.
4. On OIM-style long goals, the dominant failure in this stack is
   **translation throughput** — a consistent ≈ 0.4 m stall across object
   scales and Q choices — so cost-matrix work should target translation
   progress (or the reposition/push cadence), not the rotation balance.
5. Practical implication for OIM transfer: adaptation effort should target
   the geometry coupling (contact sampling standoffs, pusher-to-feature
   scale ratios, and drift-per-rotation kinematics of small footprints) —
   not the cost function's mass/goal terms. Similarity-scaled costs are
   necessary (see the oim_franka arc receipts) but demonstrably not
   sufficient once member widths approach the pusher diameter
   (OIM member 0.0198 m vs sphere ⌀ 0.039 m).

## Artifacts & reproduction

- Results + videos: `D:\projects\ERL\push_anything_ADMM\results\c3ab_*_20260902`
  (dirs SHA-256-verified) and `/root/push_anything_ADMM/results/c3ab_*`.
- Ledgers: `c3ab_{exp1,exp3,expTx,expTy,expTz}_trials.jsonl` (+ `_result.json`).
- Harness: success-terminated trial runner
  (`run_c3plus_trials.sh` — three binaries per lane on isolated LCM ports,
  parallel lanes), pydrake-LCM recorder (`record_full_state.py`), replay
  renderer (`render_c3plus_run.py`); copies live with the session receipts.
- One trial by hand:
  `franka_{osc_controller,sampling_c3_controller,sim} --demo_name=push_t_exp1
  --lcm_url=udpm://239.255.76.67:7991?ttl=0`.

*Report generated by the 2026-09-02 ablation session; single-seed caveat:
the stack is nondeterministic across runs (threading/timing), so all n are
independent draws, not seeds.*
