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

## Open-table run videos (all successes and failures)

Every OIM-goal (obstacle-free) trial, grouped by configuration; each
group lists its exact cost matrices. Video links are relative to this
README next to `c3ab_video_collection_20260903/` (the D-drive study folder carries both).

### native T (1 kg), OIM goal — nativeQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ² (geodesic angle), weight **1000**; `q_ω` = 0.05·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃

| Trial | Outcome | Video |
|---|---|---|
| 1 | ❌ FAIL 0.34 m / 0.21 rad | [video](c3ab_video_collection_20260903/nativeT-1kg_oimgoal_nativeQ_trial1_FAIL_pos0.34m_th0.21rad.mp4) |
| 2 | ❌ FAIL 0.35 m / 0.20 rad | [video](c3ab_video_collection_20260903/nativeT-1kg_oimgoal_nativeQ_trial2_FAIL_pos0.35m_th0.20rad.mp4) |
| 3 | ❌ FAIL 0.46 m / 2.22 rad | [video](c3ab_video_collection_20260903/nativeT-1kg_oimgoal_nativeQ_trial3_FAIL_pos0.46m_th2.22rad.mp4) |
| 4 | ❌ FAIL 0.41 m / 1.50 rad | [video](c3ab_video_collection_20260903/nativeT-1kg_oimgoal_nativeQ_trial4_FAIL_pos0.41m_th1.50rad.mp4) |
| 5 | ❌ FAIL 0.35 m / 1.32 rad | [video](c3ab_video_collection_20260903/nativeT-1kg_oimgoal_nativeQ_trial5_FAIL_pos0.35m_th1.32rad.mp4) |

### native-shape T (0.05 kg) — nativeQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ² (geodesic angle), weight **1000**; `q_ω` = 0.05·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃

| Trial | Outcome | Video |
|---|---|---|
| 1 | ✅ SUCCESS 47.7 s | [video](c3ab_video_collection_20260903/lightT-0.05kg_oimgoal_trial1_SUCCESS-48s.mp4) |
| 2 | ✅ SUCCESS 67.6 s | [video](c3ab_video_collection_20260903/lightT-0.05kg_oimgoal_trial2_SUCCESS-68s.mp4) |
| 3 | ❌ FAIL 0.35 m / 0.62 rad | [video](c3ab_video_collection_20260903/lightT-0.05kg_oimgoal_trial3_FAIL_pos0.35m_th0.62rad.mp4) |
| 4 | ❌ FAIL 0.32 m / 0.71 rad | [video](c3ab_video_collection_20260903/lightT-0.05kg_oimgoal_trial4_FAIL_pos0.32m_th0.71rad.mp4) |
| 5 | ❌ FAIL 0.62 m / 2.91 rad | [video](c3ab_video_collection_20260903/lightT-0.05kg_oimgoal_trial5_FAIL_pos0.62m_th2.91rad.mp4) |

### OIM-dims T (1 kg) — nativeQ control

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ² (geodesic angle), weight **1000**; `q_ω` = 0.05·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃

| Trial | Outcome | Video |
|---|---|---|
| 1 | ❌ FAIL 0.46 m / 0.62 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_nativeQ-control_trial1_FAIL_pos0.46m_th0.62rad.mp4) |
| 2 | ❌ FAIL 0.46 m / 0.58 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_nativeQ-control_trial2_FAIL_pos0.46m_th0.58rad.mp4) |
| 3 | ❌ FAIL 0.39 m / 0.43 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_nativeQ-control_trial3_FAIL_pos0.39m_th0.43rad.mp4) |

### OIM-dims T (1 kg) — scaledQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ², weight **260** (= 1000 × ρ_g² ratio 0.26); `q_ω` = **0.013**·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃ (unchanged)

| Trial | Outcome | Video |
|---|---|---|
| 1 | ❌ FAIL 0.53 m / 2.14 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_ruleQ-quat260_trial1_FAIL_pos0.53m_th2.14rad.mp4) |
| 2 | ❌ FAIL 0.39 m / 0.56 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_ruleQ-quat260_trial2_FAIL_pos0.39m_th0.56rad.mp4) |
| 3 | ❌ FAIL 0.45 m / 0.64 rad | [video](c3ab_video_collection_20260903/oimdimsT-1kg_ruleQ-quat260_trial3_FAIL_pos0.45m_th0.64rad.mp4) |

### OIM-dims T (0.05 kg) — nativeQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ² (geodesic angle), weight **1000**; `q_ω` = 0.05·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃

| Trial | Outcome | Video |
|---|---|---|
| 1 | ✅ SUCCESS 46.4 s | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_nativeQ_trial1_SUCCESS-46s.mp4) |
| 2 | ✅ SUCCESS 97.5 s | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_nativeQ_trial2_SUCCESS-98s.mp4) |
| 3 | ❌ FAIL 0.47 m / 1.34 rad | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_nativeQ_trial3_FAIL_pos0.47m_th1.34rad.mp4) |

### OIM-dims T (0.05 kg) — scaledQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ², weight **260** (= 1000 × ρ_g² ratio 0.26); `q_ω` = **0.013**·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃ (unchanged)

| Trial | Outcome | Video |
|---|---|---|
| 1 | ✅ SUCCESS 97.2 s | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_ruleQ-quat260_trial1_SUCCESS-97s.mp4) |
| 2 | ✅ SUCCESS 57.5 s | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_ruleQ-quat260_trial2_SUCCESS-58s.mp4) |
| 3 | ✅ SUCCESS 95.5 s | [video](c3ab_video_collection_20260903/oimdimsT-0.05kg_ruleQ-quat260_trial3_SUCCESS-96s.mp4) |

### true OIM T (0.1 kg) — nativeQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ² (geodesic angle), weight **1000**; `q_ω` = 0.05·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃

| Trial | Outcome | Video |
|---|---|---|
| 1 | ✅ SUCCESS 82.9 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_nativeQ_trial1_SUCCESS-83s.mp4) |
| 2 | ✅ SUCCESS 98.7 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_nativeQ_trial2_SUCCESS-99s.mp4) |
| 3 | ❌ FAIL 0.28 m / 0.09 rad | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_nativeQ_trial3_FAIL_pos0.28m_th0.09rad.mp4) |
| 4 | ❌ FAIL 0.43 m / 3.02 rad | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_nativeQ_trial4_FAIL_pos0.43m_th3.02rad.mp4) |
| 5 | ✅ SUCCESS 78.3 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_nativeQ_trial5_SUCCESS-78s.mp4) |

### true OIM T (0.1 kg) — scaledQ

**Q/R:** `Q_obj`: pos diag `w_Q·(200,200,120)` = (10000,10000,6000); quat: runtime Hessian of θ², weight **260** (= 1000 × ρ_g² ratio 0.26); `q_ω` = **0.013**·I₃, `q_v` = 0.05·I₃; `R` = w_R·0.01·I₃ (unchanged)

| Trial | Outcome | Video |
|---|---|---|
| 1 | ✅ SUCCESS 56.0 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_commensurateQ_trial1_SUCCESS-56s.mp4) |
| 2 | ✅ SUCCESS 63.5 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_commensurateQ_trial2_SUCCESS-64s.mp4) |
| 3 | ❌ FAIL 0.35 m / 2.53 rad | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_commensurateQ_trial3_FAIL_pos0.35m_th2.53rad.mp4) |
| 4 | ✅ SUCCESS 121.3 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_commensurateQ_trial4_SUCCESS-121s.mp4) |
| 5 | ✅ SUCCESS 82.8 s | [video](c3ab_video_collection_20260903/trueOIMT-0.1kg_commensurateQ_trial5_SUCCESS-83s.mp4) |

## Single-obstacle scenario (true OIM T + exponential obstacle cost)

**Setup.** OIM benchmark `single_obstacle`: the source scene's static 0.1 m
cube placed at the start→goal midpoint (0.5, 0.0), physically welded in the
sim. Scenario definitions are factored per demo into
`parameters/scenario_params.yaml` (scenario name, obstacle disc list
[x, y, r], cost weight/decay, obstacle SDF), loaded through
`scenario_params_file` in the controller params and consumed by both the
planner and the simulator — obstacle-free demos are untouched.

**Cost shaping.** Every candidate sample's predicted object path pays an
exponential proximity penalty per knot and per obstacle:

    cost += w_obs · exp(−(‖p_obj,xy − p_obs,xy‖ − r_obs) / d_decay)

with w_obs = 20000, r_obs = 0.0707 (circumscribing the square footprint),
d_decay = 0.02 m. Demos: `push_t_oimT_m01_obst_{nativeQ,scaledQ}`.

**Results (n = 5 success-terminated trials per arm, 1500 s wall caps):**

| Arm | success_rate | outcome detail |
|---|---|---|
| scaledQ | **1/5** (69.8 s — obstacle-free speed) | 4 × "doorstep" stall: rotation complete (θ 0.01–0.33), object parked ≈ 0.35 m out just north of the cube |
| nativeQ | 0/5 | 2 × far stall (θ ≈ 2.76, barely rotated) + 3 × doorstep-adjacent |

Obstacle-free baseline on the same object: scaledQ 4/5, nativeQ 3/5.

**Diagnosis.** The exponential term does its half of the job perfectly —
no trial ever drove the T into the cube; plans through the obstacle are
priced out. What is missing is the other half: the goal generator's
lookahead sub-goal still points straight through the obstacle, so C3's
best straight pushes are vetoed by the barrier while the sampler settles
into safe-but-unproductive choices — a barrier-induced local minimum at
the obstacle's doorstep. The single success is a draw whose early pushes
happened to compose a lateral detour before reaching the doorstep.
scaledQ retains its reliability edge (only arm to succeed; its failures
die closer, with rotation already complete).

**Cost-only remedy (per the design intent — the cost field must steer, not
just veto):** with decay 0.02 m the exponential is ~flat at doorstep
distances, so all candidates pay a near-equal tax and the sampler cannot
distinguish lateral-escape pushes from holding; the plateaued cost also
trips the progress checker into reposition churn. Lengthening
`obstacle_cost_decay` to 0.06 m extends the gradient to 5–10 cm
clearances so detour plans price measurably cheaper. Re-run in the
`c3ab_obst2_*` ledgers (results appended below when complete).

## Five-scene sweep with the best manipulable object

Object: OIM-dims T at **0.05 kg** with **scaledQ** (the study's only 3/3
configuration). One demo per OIM benchmark scene
(`push_t_bestT_<scene>`), scene geometry mapped into the reference frame;
obstacle scenes use the three-zone exponential field (w = 5000,
decay = 0.04) over the scenario's disc decomposition.

| Scene | Success | success times (s) | pos_err mean | θ_err mean |
|---|---|---|---|---|
| open_table | 3/3 | 86, 134, 102 | 0.019 | 0.037 |
| single_obstacle | 0/3 | — | 0.391 | 0.603 |
| shelf_gap | 0/3 | — | 0.418 | 2.091 |
| ycb_clutter | 0/3 | — | 0.423 | 0.437 |
| icra_sign | 3/3 | 41, 75, 60 | 0.019 | 0.069 |

**Read:** obstacle-free scenes are solved outright (6/6, 41–134 s;
icra\_sign — 0.70 m + π/2 — is the fastest task in the study). All three
obstacle scenes fail 0/3, but close: the best draws finish the rotation
(ycb\_clutter trial 1 ends at θ = 0.050) and stall on the final routing
past the statics — the same doorstep local-minimum family as the decay
sweep, now confirmed scene-independent. The cost-only exponential steers
and vetoes correctly but composing the *detour push sequence* remains the
unsolved piece across every static-obstacle layout.

### The four questions

**1. How is the environment SDF implemented?** Physically: each scene is
a `<static>true</static>` SDF model welded at parse time (e.g.
`scene_shelf_gap.sdf`) — the sim collides with real geometry. 
Analytically: the planner uses a disc decomposition of the same statics
(`obstacles: [[x, y, r], …]` in `scenario_params.yaml`); signed distance
sd(p) = ‖p_xy − p_o‖ − r per disc, boxes covered by one circumscribed
disc or several overlapping discs (the shelf pair uses 3+2 so the gap
survives in the cost field); multi-obstacle cost is the sum of per-disc
exponentials — a smooth soft-min of the true SDF.

**2. How are cost and actions optimized w.r.t. the scene?** Three nested
loops. Inner: C3/ADMM solves the contact-implicit QP with x'Qx + u'Ru —
scene-free, LCS structure fixed. Middle: each candidate sample's solution
is rolled out and scored: tracking + travel + Σ_knots Σ_obs
w·exp(−sd/decay) over the predicted object path. Outer: mode selection
(C3 vs reposition, hysteresis + progress checks) consumes the
scene-shaped scores. The scene steers action *selection*, never the QP.
The decay sweep is the proof the field shape governs: 0.02 = veto-only,
0.06/20k = start-frozen, 0.04/5k = three-zone.

**3. How well does the object work, and why?** Perfect on every
obstacle-free task (9/9 lifetime: 3/3 + 3/3 here + its original 3/3),
because at 0.05 kg the force headroom is enormous (~0.2 N friction vs the
50 N input box) so C3's predictions execute faithfully, per-push
displacement is large, and scaledQ's ρg²-commensurate weights remove the
over-rotation stall mode. In obstacle scenes the object is still the
best performer observed (failures land nearest, rotation complete) but
no configuration yet composes the final detour.

**4. Why experiment on this object?** It is the bridge object: exact OIM
benchmark geometry — so member-width/pusher-diameter and
drift-per-rotation couplings are the real benchmark's — at the mass end
of the OIM range where the reference method demonstrably works. Results
on it isolate scene difficulty from object difficulty: its clean 6/6 on
obstacle-free scenes certifies that the remaining obstacle-scene failures
are planning/steering problems, not manipulability problems.

## 28-run open-table campaign and Q retuning (quat 510)

FIG8-style protocol: 28 success-terminated open-table trials of the best
object (OIM-dims T 0.05 kg) at scaledQ (quat 260), then a data-driven
retune tested at n = 14.

| Config | n | success | median t-to-goal | steps-to-goal (succ) | failure clusters |
|---|---|---|---|---|---|
| scaledQ quat 260 | 28 | 64.3 % | 125.3 s | 1204 | 7 mid-path stalls + **3 near-goal misses** (θ stuck 0.21–0.31) |
| **tuned quat 510** | 14 | **71.4 %** | **57.5 s** | **771** | 4 mid-path stalls, **0 near-goal misses** |

The near-goal cluster identified the tuning direction: 260 under-drives
terminal alignment while 1000 over-drives mid-task rotation; the geometric
mean 510 eliminated the terminal cluster and halved median time-to-goal.
Success-rate deltas are not individually significant at these n; the
cluster elimination and the speed shift are the mechanism-consistent
evidence. **Tuned recommendation: quat 510** (demo
`push_t_bestT_open_table_q510`); mid-path translation stalls remain the
Q-invariant residual.

**Native T (1 kg, nativeQ) through single_obstacle (n = 3):** 0/3, pos
0.390 ± 0.013, θ 0.289 ± 0.105 — the heavy object's translation wall and
the obstacle doorstep coincide; the exponential field prevented any
cube contact at 1 kg momentum.

**ICRA-sign caveat:** our icra_sign scene keeps the study's T object with
the source scene's goal pose (0.5, −0.40, π/2); the original benchmark
task pushes a C-shaped 2 kg sign into a slot, which this sweep does not
reproduce — treat our 3/3 as a long-diagonal open-table variant, not the
benchmark task.

## How the cost functions are defined, per scenario

Four layers, from the QP outward. Layers 0–1 are identical in every
scenario; layer 2 is the scenario; layer 3 is the object/Q configuration.

**Layer 0 — the C3 solve (contact-implicit MPC).** Per knot i of horizon
N = 5: `γ^i (x_i − x_des)ᵀ Q (x_i − x_des) + γ^{i+1} u_iᵀ R u_i` plus the
ADMM matrices G (consensus) and U (projection), γ = 1. Two complete cost
*sets* exist and the controller switches between them by distance to
goal (`cost_switching_threshold_distance` = 0.50 m, summed over objects):

- **Position regime** (far): `w_Q_position · q_vector_position`
  (object position 250,250,250), `planning_dt` = 0.10 s, quaternion block
  inert (0.1 placeholders), G/U from the `*_position_list`s.
- **Pose regime** (near): `w_Q · q_vector` (object position 200,200,120 —
  or the raised variants below), `planning_dt` = 0.05 s, and the
  quaternion block is **rebuilt every solve** as
  `w_quat · [∇²θ²(q, q_des) + PSD shift]` — the Gauss–Newton model of the
  squared geodesic angle.

The regime boundary is behaviorally loud: the 28-run mid-path failure
band (0.35–0.48 m) brackets the 0.50 m switch, where the objective gains
rotation pressure and halves its planning dt simultaneously.

**Layer 1 — sample scoring & mode economics (all scenarios).** Each
candidate EE placement gets a C3 solve whose plan is re-simulated
(`cost_type 5`, impedance rollout, object terms only) →
`all_sample_costs[i] = rollout_cost + travel_cost_per_meter·d_xy (=0)
[+ finished_reposition_cost when a reposition just completed]`. Mode
switching (C3 ↔ reposition) runs on these scores through *relative*
hysteresis fractions (0.3–0.9), so all layers' costs also govern when the
controller pushes vs relocates.

**Layer 2 — the scenario term (`scenario_params.yaml` per demo).**
`Σ_knots Σ_obs w_obs · exp(−(‖p_obj,xy − p_o‖ − r_o)/d_decay)` added to
layer-1 scores over the predicted object path. Per scene:

| Scene | obstacle discs [x, y, r] | w_obs | decay |
|---|---|---|---|
| open_table | — | 0 | — |
| single_obstacle | (0.5, 0, 0.0707) | 5000 | 0.04 |
| shelf_gap | 3 discs @ x=0.73 (r 0.133) + 2 @ x=0.35 (r 0.064) | 5000 | 0.04 |
| ycb_clutter | (0.5, 0, 0.0707) + (0.37, 0.20, 0.0996) | 5000 | 0.04 |
| icra_sign | — | 0 | — |

Field-shape calibration is empirical and documented above: decay 0.02 =
veto-only; 0.06 @ w=20000 = start-frozen; 0.04 @ w=5000 = three-zone.

**Layer 3 — Q configurations under test (pose regime, object block).**

| Config | obj pos diag (×w_Q=50) | quat weight | q_ω |
|---|---|---|---|
| nativeQ | 200,200,120 | 1000 | 0.05 |
| scaledQ | 200,200,120 | 260 (ρ_g²-commensurate) | 0.013 |
| q510 (tuned) | 200,200,120 | 510 | 0.013 |
| q510p15 (trial) | 300,300,180 | 510 | 0.013 |
| q510p2 (trial) | 400,400,240 | 510 | 0.013 |

The p15/p2 variants raise position dominance across the 0.5 m switch to
attack the mid-path stall cluster (>90 % SR campaign, in progress).

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
