# dairlib
Warning! This is very much "development-level" code and is provided as-is. APIs are likely to be unstable and, while we hope for the documentation to be thorough and accurate, we make no guarantees.

## Current Continuous Integration Status
* `main` branch build and unit tests (Ubuntu Jammy 22.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_jammy&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* `main` branch build and unit tests (Ubuntu Focal 24.04): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=build_focal&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
* Experimental build against Drake's `master` branch (Jammy): [![Build Status](https://api.cirrus-ci.com/github/DAIRLab/dairlib.svg?task=drake_master_build&script=test)](https://cirrus-ci.com/github/DAIRLab/dairlib)
## Complete Build Instructions

### Download dairlib
1. Clone `dairlib` into the your workspace, e.g. "my-workspace/dairlib".
```
git clone https://github.com/DAIRLab/dairlib.git
```

2. Download and setup SNOPT

dairlib, by default, assumes that users have access to SNOPT(https://web.stanford.edu/group/SOL/snopt.htm), though it is not required. **If you do not have SNOPT**, you will need to edit `.bazelrc` and change `build --define=WITH_SNOPT=ON` to `build --define=WITH_SNOPT=OFF`

For users at Penn, download SNOPT (https://www.seas.upenn.edu/~posa/snopt/snopt7.6.tar.gz) and add the following line to your `~/.bashrc`
```
export SNOPT_PATH=<the directory you downloaded to>/snopt7.6.tar.gz
```

There is no need to extract the tar.

### Build Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details). There are two ways to use Drake within dairlib:

#### Option 1: use pegged revision (Note - These steps may need repeated if switching to a branch with a different pegged revision of drake).

In `dairlib/install`, run the `install_prereqs_ubuntu.sh`. Our build process does not currently support MacOS, though it has in the past and likely will in the future.

This option is recommended for users who are not currently editing any source code in Drake itself.

#### Option 2: source install of Drake
If you would like to use your own local install of Drake, likely because you are modifying it, when you build with Bazel you will need to use `bazel build --override_module=drake=/home/user/my-workspace/drake <package you are building>` (using the appropriate directory for your own install). There is no need to build Drake.

### IDE setup
JetBrains IDEs have worked well for us and are available for free to students. For C++ development using the CLion Bazel plugin, see https://drake.mit.edu/clion.html and replace `drake` with `dairlib` in the "Setting up Drake in CLion" section. 

### Other dependencies
These dependencies are necessary for some advanced visualization and process management. Many examples will work without a full installation of Director or libbot, but (for lab members), these are ultimately recommended. 

#### LCM and libbot
Install a local copy of `lcm` and `libbot2` using `sudo apt install lcm libbot2`. The prerequisites installation (option 1.a) should add the proper apt repo for these.

#### ROS
To integrate with ROS (tested on ROS Noetic with 20.04), the following steps are required.
1. Install ROS http://wiki.ros.org/ROS/Installation
2. Do not forget to setup your environment. For instance, add these lines to `~/.bashrc`
```
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash 
```
3. Install additional dependencies
```
sudo apt install python3-rosinstall-generator python-catkin-tools python3-vcstool
```
4. Build the ROS workspace using catkin. From `dairlib/`,
```
sudo ./tools/workspace/ros/compile_ros_workspace.sh
```
5. Set the environment variable `DAIRLIB_WITH_ROS` to `ON`. For instance, add to `~/.bashrc`
```
export DAIRLIB_WITH_ROS=ON
```

#### Invariant-EKF
State Estimation for Cassie is done using contact-aided invariant-EKF. `invariant-ekf` is an external repository forked from Ross Hartley's repository of the same name. By default, a pegged version of this forked repository is used i.e. the `bazel` branch of DAIR lab's fork of `invariant-ekf` is automatically downloaded and used. However, to make changes to the files, the [DAIR Lab's fork of invariant-ekf](https://github.com/DAIRLab/invariant-ekf/tree/bazel "DAIR Lab's fork of invariant-ekf") can be cloned as a local repository.

To use local version of `invariant-ekf`, set the environment variable `DAIRLIB_LOCAL_INEKF_PATH`, e.g.
```
export DAIRLIB_LOCAL_INEKF_PATH=/home/user/my-workspace/invariant-ekf
```

### Notes for macOS
1. Be sure to have Xcode 9.0 or later installed with Command Line Tools. If you receive a `clang: error: cannot specify -o when generating multiple output files` message during the build process, re-run `install_prereqs.sh`, and be sure that it runs fully before termination, as this will reconfigure Xcode to work with Drake.


### Build dairlib
Build what you want via Bazel. From `dairlib`,  `bazel build ...` will build the entire project. Drake will be built as an external dependency.
- If you run into ram/cpu limits while building, you can cap the number of threads bazel will use (here we choose `8`) by either:
    - adding `build --jobs=8` to `.bazelrc` 
    - using the `jobs` flag when calling `build` (e.g. `bazel build [target] --jobs=8`)

## Included Modules
A list of included modules

### Object-Informed-Manipulation (OIM xArm6 port)

The C++ xArm6 port connects the OIM-T spatial model, Full Sampling-C3+ contact
planner, collision-aware task-space acquisition, OSC, and Drake simulation.
Sources live in
[`examples/sampling_c3/oim_t/`](examples/sampling_c3/oim_t/); the canonical
configuration is
[`parameters/oim_t.yaml`](examples/sampling_c3/oim_t/parameters/oim_t.yaml).
The implementation and experiment history are documented in
[`FULL_SAMPLING_C3PLUS_2026-08-31.md`](examples/sampling_c3/oim_t/FULL_SAMPLING_C3PLUS_2026-08-31.md)
and the per-gate ledgers listed below.

#### Task: `open_table`

An xArm6 with a vertical pushing stick must push a planar T block across an
open table from its start pose to a goal pose. The goal variables are the
object's planar SE(2) pose in the `oim_world` frame:

```text
g = (x_g, y_g, θ_g) = (0.381, -0.400, 3.1416)      # object.goal_pose
x^o = (x, y, θ)                                     # measured T pose (yaw from quaternion)
```

Success is a terminal tolerance check on both goal variables simultaneously
(`task.translation_tolerance`, `task.orientation_tolerance`):

```text
e_p = || (x, y) - (x_g, y_g) ||_2         <  0.05 m
e_θ = | wrap(θ - θ_g) |                   <  0.10 rad
```

The orientation error is computed in three steps
(`xarm6_full_sampling_c3plus.cc:92-107`): the measured quaternion
`q_WO = (w, x, y, z)` is normalized and reduced to its heading with the
standard ZYX yaw formula `θ = atan2(2(wz + xy), 1 − 2(y² + z²))`; the raw
difference `Δ = θ − θ_g` is formed; and `Δ` is wrapped with

```text
wrap(Δ) = atan2(sin Δ, cos Δ)   ∈ (−π, π]
```

The raw `Δ` is meaningless as a distance because yaw lives on the circle S¹:
`θ` and `θ + 2π` are the same physical heading, so `Δ` can be off by any
multiple of `2π` depending on the `atan2` branch. Passing `Δ` through
`sin`/`cos` erases every multiple of `2π`, and `atan2` rebuilds the unique
representative in `(−π, π]` — the shortest signed arc from `θ_g` to `θ`, with
no modulo edge cases and `|wrap(Δ)| ≤ π` always.

This matters for `open_table` specifically because `θ_g = 3.1416 ≈ π` sits on
the branch cut: a nearly-converged T can be measured at `θ = +3.10` on one
tick and `θ = −3.10` on the next. Unwrapped, the second reads
`|Δ| = 6.24 rad` — a false failure; wrapped, both read `e_θ ≈ 0.04 rad` and
pass the 0.10 rad gate. Roll and pitch are excluded from `e_θ` by design (a
toppled T is caught by the settle check's tilt angle
`ψ = acos((R_WO ẑ)·ẑ)`), and the same wrap is used for per-cycle yaw-progress
accounting (`xarm6_full_sampling_c3plus.cc:836-846`). The task therefore
requires roughly 0.8 m of translation plus a genuine ~π reorientation of
the T.

#### What we optimize

Each control cycle solves a finite-horizon contact-implicit MPC problem with
C3+ (ADMM over consensus copies) on a locally linearized Linear
Complementarity System. The state is `x ∈ R^19` (pusher position, object
quaternion, object position, then velocities), the input is `u ∈ R^3`
(Cartesian pusher force), contact forces are `λ ∈ R^20`, and the horizon is
`N = 5`:

```text
minimize    Σ_{t=0..N} (x_t - x_d)ᵀ Q (x_t - x_d)  +  Σ_{t=0..N-1} u_tᵀ R u_t

subject to  x_{t+1} = A x_t + B u_t + D λ_t + d          (LCS dynamics)
            0 ≤ λ_t ⊥ E x_t + F λ_t + H u_t + c ≥ 0     (complementarity)
```

The desired state `x_d` encodes the `open_table` goal variables directly: the
object-position slots hold `(x_g, y_g, resting_height)` and the
object-quaternion slots hold `q(θ_g)`, a pure yaw rotation built from
`goal_pose.z()` (`xarm6_full_sampling_c3plus.cc:1597-1605`). The cost
matrices are assembled in `RunSolveAtSampledPusher`
(`xarm6_full_sampling_c3plus.cc:1579-1596`):

```text
Q = state_cost_scale · diag(state_cost_diagonal)
  = 50 · diag(0.01, 0.01, 0.01,          # pusher position
              0.1, 0.1, 0.1, 0.1,        # object quaternion  → orientation goal
              200, 200, 120,             # object position    → translation goal
              5, 5, 5,  0.05 ×6)         # velocities
R = 1.0 · diag(0.01, 0.01, 0.01)
```

so translation error is weighted at an effective 10,000 per m² on object x/y
and orientation enters through the quaternion-error terms. ADMM additionally
carries consensus and projection penalties `G = 0.01·diag(g)` and
`U = 0.26·diag(u)` on the `λ`/`η` copies (weights 2/1 and 20/1); these enforce
the complementarity structure and are not part of the task objective.

Around that inner QP, three more objectives shape the behavior:

- **Sample ranking.** Candidate pusher placements are each solved and then
  scored by forward-simulating the plan through the LCS and accumulating the
  same quadratic error, `Σ eᵀ Q e` plus a terminal term
  (`dynamic_rollout_cost`, `xarm6_full_sampling_c3plus.cc:1740-1834`); the
  minimum-cost candidate is executed. A separate
  `object_yaw_cost_weight: 50.0` biases goal/sample selection toward yaw
  progress.
- **Acquisition IK.** Repositioning to a sampled contact solves an inverse
  kinematics problem with a position constraint on the stick tip and the
  restored source tilt objective `w_tilt · (1 - cos ψ)`, `w_tilt = 80`, which
  keeps the stick vertical inside the feasibility band
  (`xarm6_sampling_c3_controller.cc:120`, `:391-400`).
- **OSC tracking.** The 500 Hz operational-space controller tracks the
  selected trajectory with Cartesian gains `kp = 200`, `kd = 20` and a
  0.01-weight joint-posture regularizer.

In short: the *task* asks for `e_p < 0.05 m` and `e_θ < 0.10 rad` on the T's
planar pose; the *optimizer* minimizes a quadratic penalty on exactly those
two errors (plus small pusher/velocity/effort regularization) subject to
contact-implicit dynamics, and every sampled contact location competes on the
same objective.

#### Franka arm mathematics (the reference path the xArm6 port inherits)

The `franka_*` executables in `examples/sampling_c3/` are the DAIRLab
Sampling-C3 reference this port adapts. Their math is identical in structure
to the xArm6 formulation above; this records it step by step with the C++
sources.

**Step 1 — the planner does not control joints.** The seven-joint Panda is
reduced to the spherical pusher at its end effector. The planning state is
the same mixed robot/object vector:

```text
x = [ p^EE_W,  q_WO,  p^O_W,  ṗ^EE_W,  ω^O_W,  ṗ^O_W ]  ∈ R^19
```

**Step 2 — how the input is defined.** `u ∈ R^3` is the Cartesian force
applied at the pusher, in Newtons, bounded by the per-task force limits.
The planner decides what force the fingertip ball should exert; the OSC
finds the seven joint torques that realize it.

**Step 3 — the physics.** Drake's manipulator equation for the coupled
arm/object/table plant:

```text
M(q) v̇ + C(q, v) v = τ_g(q) + B u + J_nᵀ λ_n + J_tᵀ λ_t
```

**Step 4 — the matrices inferred each tick.** `@c3//`'s LCS factory
(`c3/multibody/lcs_factory.cc`, `FormulateAnitescuContactDynamics` at
`:496-545`) linearizes `f = M⁻¹(Bu − Cv + τ_g)` by autodiff into
`J_f = [J_q  J_v  J_u]` with exactness offset `d_v`, extracts the contact
geometry — gaps `φ (n_c)`, normal Jacobians `J_n (n_c × n_v)`, tangential
Jacobians `J_t (4n_c × n_v)` with a 4-edge friction pyramid per contact,
the tangent-summing selector `E_t`, and `μ` — and assembles the
discrete-time LCS under the Anitescu model:

```text
J_c = E_tᵀ J_n + diag(μ) J_t                          # friction-folded Jacobian

x[t+1] = A x[t] + B u[t] + D λ[t] + d
0 ≤ λ[t] ⊥ E x[t] + F λ[t] + H u[t] + c ≥ 0

D = [ dt² · qdotNv · M⁻¹ J_cᵀ ;  dt · M⁻¹ J_cᵀ ]
E = [ dt·J_c·J_q + E_tᵀ J_n · vNqdot/dt ;  J_c + dt·J_c·J_v ]
F = dt · J_c · M⁻¹ J_cᵀ                               # Delassus coupling
H = dt · J_c · J_u
c = E_tᵀ φ/dt + dt·J_c·d_v − E_tᵀ J_n · vNqdot · q/dt
```

The complementarity row predicts each friction-pyramid direction's
post-step contact velocity/gap; `0 ≤ λ ⊥ (·) ≥ 0` allows forces only to
push and only while the gap is closed. `n_c` (pusher-vs-object faces plus
object-vs-ground witnesses) changes with the sampled candidate and the
object pose, so every matrix is re-inferred per tick and per candidate.

**Step 5 — the objective** is the same quadratic tracking problem as the
xArm6 section: `Σ (x_t − x_d)ᵀ Q (x_t − x_d) + Σ u_tᵀ R u_t` over the LCS,
ADMM weights `G`/`U` on the `(λ, η)` copies
(`systems/controllers/sampling_based_c3_controller.cc`).

**Step 6 — realizing `u`.** `franka_osc_controller.cc` instantiates the
shared inverse-dynamics OSC QP
(`systems/controllers/osc/operational_space_control.cc`) on the seven-joint
plant: it tracks the planned pusher trajectory, applies the planner's force
as an external-force objective, and outputs `τ ∈ R^7`. The xArm6 port
replaces the plant and frames but reuses this OSC QP unchanged. The
planner's `λ` is a cost demand, not a measured force — the OSC and the
simulator decide what is physically exerted.

A Python/PyDrake reproduction of this same Franka formulation — with the
per-tick linearization implemented in `control/lcs_formulator.py` — lives in
the companion repository:
[push_anything_ADMM](https://github.com/hdoh-ucsd/push_anything_ADMM#franka-arm-mathematics).

#### Experiment history (gate ledgers)

Terminal `open_table` success has not yet been achieved; each ledger records
the mechanism landed, its receipts, and the measured terminal error:

| Gates | Landed | Terminal `(e_p, e_θ)` |
| --- | --- | --- |
| [241–340](examples/sampling_c3/oim_t/GATES_241_340_2026-09-01.md) | Measured contact-phase budgeting, persistent response completion, whole-capsule/live-IK local repositioning, recovery lateral reserve | 0.757 m, 2.950 rad |
| [341–350](examples/sampling_c3/oim_t/GATES_341_350_2026-09-01.md) | Removed positive-gap false completion; elevated traverse split from lowering; nonincreasing live-FK posture steps | 0.764 m, 2.844 rad |
| [351](examples/sampling_c3/oim_t/GATE_351_2026-09-01.md) | 5 mm predicted lateral gate authoritative at execution admission (clean exit, no contact: admission sets were disjoint) | — |
| [352–362](examples/sampling_c3/oim_t/GATES_352_362_2026-09-01.md) | Replenished admission set; contact-continuation latch; dwell → release → measured replanning → successor contact cycle (one productive cycle: 4.7 mm, 0.033 rad) | 0.773 m, 3.038 rad |
| [363–369](examples/sampling_c3/oim_t/GATES_363_369_2026-09-01.md) | DAIRLab-Panda-aligned execution logistics; capsule-clear reorientation (fixed-home recoveries 11 → 0; joint-3 saturation 19.3% → 6.7%) | 0.726 m, 2.910 rad |
| [370–374](examples/sampling_c3/oim_t/GATES_370_374_2026-09-01.md) | 2 ms Drake point-pair contact receipts; diagnostic contact classifier and corrected renderer | 0.784 m, 3.086 rad |
| [375–379](examples/sampling_c3/oim_t/GATES_375_379_2026-09-01.md) | Receipts joined to the controller's selected-face transaction; exposed 357 dwell credits over 7.12 s with no physical contact | — |
| [380](examples/sampling_c3/oim_t/GATE_380_2026-09-02.md) | Root cause fixed: direction-preserving limiter + restored `w_tilt = 80` tilt objective (the clamp had masked the dropped tilt cost); C-run clean — 0 dwell REJECTs, 0.13 mm drift | gate tier |

### DIRCON
A modern Drake implementation of the DIRCON constrained trajectory optimization algorithm. Currently under construction. See `/examples/PlanarWalker/run_gait_dircon.cc` for a simple example of the hybrid DIRCON algorithm. The more complete example set (from the paper) currently exists on an older version of Drake https://github.com/mposa/drake/tree/hybrid-merge

Based off the publication

Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization of Trajectories for Constrained Dynamical Systems." Proceedings of the International Conference on Robotics and Automation (ICRA), 2016. 

Available online at https://posa.seas.upenn.edu/wp-content/uploads/Posa16a.pdf

## Docker (experimental)
Docker support is currently experimental. See `install/bionic/Dockerfile` for an Ubuntu Dockerfile. Docker is being used in conjuction with Cirrus Continuous Integration, and should be better supported in the future.
