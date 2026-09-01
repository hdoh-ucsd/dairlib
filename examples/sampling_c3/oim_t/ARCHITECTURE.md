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
        xarm6_sim        xarm6_osc_controller  xarm6_sampling_c3_controller
              |                   ^                   |
              | XARM_STATE       | XARM_INPUT        | OIM_XARM_TRAJECTORY
              +------------------>|<------------------+
              |
              +---- OIM_T_STATE --------------------->+

 xArm execution plant        reduced-order planner plant
 --------------------        ---------------------------
 six controlled joints      floating point end effector
 rigid wrist/tool             one floating OIM T
 physical table/contact       synthetic support contacts
 2 ms integration             C3+ planning time step
```

Canonical xArm6 file identity is explicit:

- processes: `xarm6_sim.cc`, `xarm6_osc_controller.cc`, and
  `xarm6_sampling_c3_controller.cc`;
- shared model/config support: `xarm6_process_common.{h,cc}`;
- full planner: `xarm6_full_sampling_c3plus.{h,cc}` and
  `test/xarm6_full_sampling_c3plus_test.cc`;
- spatial LCS pusher: `xarm6_lcs_pusher.urdf`.

The corresponding Bazel targets use the same `xarm6_*` names. Legacy
`xarm_*` labels are compatibility aliases only and do not create ambiguously
named source files or binaries.

The measured lateral-recovery gate remains a small, explicit state machine:

```text
physical face at 2 mm x guard
              |
              v
 reachable high-x face? --yes--> collision-checked reposition
              |
              no
              v
 measured-x tangential fallback on the current physical face
              |
              v
 >= 1 mm measured recovery inside 2 mm corridor
              |
              v
 first proof: zero Cartesian advance through 1,000-step dwell
              |
              v
 productive receipt -> resume same face while progress watchdog passes
              |
              +--> later independent recoveries: measured receipt, no repeated
                   proof dwell
              |
              +--> deficient 2,000-step window: preserve ABORT receipt and
                   release to a task-progress face
```

## Ownership boundary

- OIM owns the xArm model, T geometry, start and goal, table, time steps, and
  success tolerances.
- DAIRLab owns Sampling-C3+ mode selection, LCS construction, consensus solve,
  trajectory messages, and OSC infrastructure.
- `xarm6_sim` owns physical integration and publishes xArm6 and T state.
- `xarm6_sampling_c3_controller` converts xArm6/T observations into the existing
  reduced-order C3+ state and publishes Cartesian trajectories.
- `xarm6_osc_controller` converts those trajectories to six actuator torques.
  It must not retain Panda joint names, seven-DOF assumptions, or Panda limits.

The Drake actuator port is torque-valued, while the source xArm actuator port
is desired joint velocity. `xarm6_osc_controller` preserves the source boundary
explicitly: it inverts desired servo torque into
`qdot_command = qdot + torque / kv`, applies the source velocity box, reapplies
`kv * (qdot_command - qdot)`, and clips only that servo force. Gravity
compensation is added outside the clamp and joint-4 stiffness remains passive.
During bounded reposition only, the controller also supplies the model's
joint-damping term for the velocity-limited posture trajectory before this
unchanged servo/clamp boundary. This compensates the damping already present in
the imported xArm plant; it does not retune OSC or servo gains.

## Validation gates

1. The canonical YAML loads and all vector dimensions agree.
2. The xArm MJCF loads with six explicitly recreated actuators, including
   wrist-roll joint 6.
3. Start and goal match OIM without the historical DAIRLab frame warp.
4. State messages contain six positions, six velocities, and six efforts.
5. OSC holds home and respects the configured per-joint effort box.
6. Sampling-C3+ reaches its first solve without a workspace assertion.
7. A deterministic short rollout records commit, dirty state, YAML checksum,
   command, seed, dependency versions, and output location.
8. Only after gates 1-7 pass may the Python port consume this YAML.

## Current status

The clean branch now contains the three named native process entry points and
the narrowly vendored open-table OIM-T model. Each entry point loads the one
canonical YAML, imports the xArm plant, recreates the six actuators omitted by
the pinned Drake MJCF parser, and rejects a plant that violates the six-joint
contract. `xarm6_sim` additionally initializes and advances the physical scene.

The compatibility layer restores the ignored joint-4 spring, configured joint
velocity and actuator effort limits, and xArm gravity compensation. A one-second
static hold has zero measured joint drift and leaves the T at its configured
rest height.

The first closed-loop transport gate is also connected. `xarm6_sim --lcm=true`
publishes the six selected xArm joints on `XARM_STATE_SIMULATION`, accepts
six named actuator torques on `XARM_INPUT_SIMULATION`, and publishes the native
Drake T state separately. `xarm6_osc_controller` consumes each state message,
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
effort box. In a three-process test, a +10 mm x request produced a
+7.604 mm tip displacement.

`xarm6_sampling_c3_controller --first_solve_only` completes one native C3+
solve over a one-contact linearization. The contact gap uses the exact OIM T
minimum-y boundary (-79.4 mm), the measured 5.55 mm pusher radius, physical
T mass 0.1 kg, configured start/goal, five-knot horizon, 0.05 s planning step,
and three ADMM iterations.

`xarm6_sampling_c3_controller --live_sampled_plan=true` is the next closed-loop
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

The acquisition gate now explicitly selects the boundary whose object reaction
direction best matches the live goal displacement, breaks equal-face ties by
tip proximity, and approaches that boundary in three-dimensional task space.
The configured measured pusher radius, free-space clearance, and 3 mm
activation tolerance define the transition; C3+ remains disabled until the
physical tip reaches it. This keeps separated-contact LCS solutions from
driving execution before their unilateral mode is physically meaningful.

The task-space block now instantiates DAIRLab's inverse-dynamics
`OperationalSpaceControl` QP on the six-joint xArm plant. Its translational
tracking data uses the unchanged Sampling-C3 gains (`Kp=200 I`, `Kd=20 I`,
`W=I`) and shared acceleration regularization (`1e-7 I`). The QP's total torque
passes through the verified xArm servo bridge, with gravity removed before and
restored after the source actuator clamp. The xArm configuration intentionally
omits Franka-only orientation, elbow, and seven-joint objectives.

A state-aware trajectory source holds the measured tip until the first valid
planner trajectory arrives. Planner targets contain two identical knots over
one planning interval because DAIRLab's first-order-hold receiver contract
requires at least two samples.

Large vertical motion is conditioned by a second, translation-only OSC. A
Drake differential-IK source evaluates the live tip Jacobian, respects the
xArm position and velocity limits, and supplies a weak six-joint posture
target over one planning interval. The acquired posture is the null-space
nominal. The controller selects this path monotonically on the first downward
waypoint. Acquisition likewise latches one exact-T boundary sample and advances
monotonically through `align-y -> align-x -> descend -> engage -> contact`;
contact cannot fall back to the clearance target after its 3 mm gate passes.

The contact LCS now carries T yaw and yaw rate in addition to its translational
state. Each sampled boundary point is transformed by the measured T yaw, and
its moment arm maps the scalar normal reaction into angular acceleration using
the T SDF's planar moment of inertia. Wrapped terminal yaw error is part of the
C3+ state cost. The live selector ranks finite candidates by weighted x, y, and
yaw goal error only after applying a hard predicted-x corridor; rejected and
finite sample counts are emitted with each logged contact decision.

Contact execution adds a small configured inward normal advance to the C3+
pusher target and fixes target z to the live object's planar height. Fixing z
is essential: using measured tip z as the next target created positive feedback
from vertical collision reactions and allowed the capsule to climb over the T,
invalidating the planar contact prediction. The simulator reports yaw-error
reduction and physical lateral drift as diagnostic gates while retaining the
original simultaneous translation/orientation definition of
`open_table_success`.

Successor-face execution follows DAIRLab's C3/reposition split while remaining
specialized to the exact OIM T. C3 is restricted to the active physical face;
different-face candidates are evaluated at hypothetical contact configurations
and must pass the same predicted-x corridor. A candidate wins only after the
minimum face dwell and either insufficient yaw progress or relative cost
hysteresis. The transition latches that face and executes
`outward retreat -> lift -> traverse -> descend -> engage`; traversal cannot begin until the
77.38005 mm waypoint is reached, and descent cannot begin until the successor
clearance point is reached. Object translation/yaw during free-space motion and
minimum measured traverse height are part of the collision-free receipt.

Repositioning is a separate primary six-joint OSC mode. It first retreats by
the configured descent clearance along the current physical face normal,
breaking unilateral contact before any vertical motion. Release retreat and
lift use bounded, measured-state-seeded tip IK so the weak
translation-only null-space objective cannot rotate the inclined capsule over
the T. Drake inverse kinematics then constrains the tip waypoint and requires
the stick axis to put the wrist on the selected face's outward side. Each
velocity-limited posture command is swept at nine
joint-space samples. At every sample, the exact MJCF capsule centerline
(`z=5.55..173.85 mm`, radius 5.55 mm) is tested conservatively against both
radius-expanded SDF T boxes and the table; the local `z=179.4 mm` tip is checked
separately against the unexpanded boxes and table. Only a passing posture is
published as `collision_aware_reposition_posture_target`. The reposition OSC
tracks that collision-checked posture and its corresponding Cartesian tip
waypoint. It uses the existing YAML `reposition_posture_kp/kd` and task-space
gains; ordinary contact OSC is unchanged. The paired objective prevents a
small posture steady-state error from becoming a large neutral-anchor tip
error.
Traverse and vertical descent retain one pusher radius of geometry-derived
standoff beyond the configured descent point. That extra execution clearance
is removed only by the subsequent lateral engagement, whose configured
clearance and contact gates are unchanged.

Engagement has two distinct gates. Reaching the 5 mm clearance target within
the existing 3 mm activation band permits C3 handoff, but does not claim
contact. The configured inward step then continues on the latched face, and a
separate physical-contact receipt is emitted only when the exact normal gap is
at most 3 mm. Tangential error must remain within the existing 5 mm approach
tolerance throughout handoff. This avoids both chasing a moving world point
after contact begins and falsely treating a clearance pose as physical contact.

Diagnostic flags can force the first switch and successor sample to exercise
the long physical transition; both default off and do not participate in
canonical selection. Canonical execution has now autonomously completed
`crossbar_top_left -> crossbar_right -> crossbar_top_left`. The simulator owns
the authoritative capsule--T force/contact receipt; planner clearance and
geometric-contact messages are not treated as physical proof.
After the first reposition, successor selection is inhibited until geometric
contact and the existing `successor_minimum_contact_steps` dwell both complete.
This prevents the measured-lateral guard from abandoning a corrective face in
the interval between reduced-model handoff and Drake's force receipt.

Corrective contact now has a stronger receipt than ordinary successor dwell.
The measured-x selector ranks normal authority first; y/yaw recovery is
deliberately deferred to the task-progress selector. If the required side face
was blocked by the prior five-DOF shaft branch, the already physical face is
retained and its tangential target is conditioned along the signed measured x
error using the existing Cartesian step bound. Recovery must reduce absolute x
error by at least half of the conservative 2 mm guard (1 mm), enter that guard,
and finish within the unchanged 5 mm lateral tolerance. The first recovery also
completes the unchanged 1,000-step proof dwell; once its reduction is measured,
both normal and tangential advance are zero for the rest of that dwell so the
controller cannot erase recovery by pushing through the goal. Later
independently measured recoveries receive the same reduction/corridor/tolerance
test and resume immediately on the same face while the configured task-progress
watchdog passes. A deficient watchdog window terminates a recovery that has not
reached the corridor and preserves an explicit `ABORT` receipt before
repositioning. Nominal force polarity, geometric arrival, and passive
measurement noise are not accepted as recovery.

The live controller drains LCM subscriptions to the newest state before every
solve, matching `LcmDrivenLoop` and preventing the 500 Hz robot stream from
forming an unbounded stale queue. Successor shaft clearance is the outward
wrist displacement `-normal_W.dot(stick_axis_W)`. Candidates with negative
clearance are rejected, and the choice is checked again while elevated before
lateral traverse or descent. This sign convention prevents an inclined tool
from reaching the T top before the intended side face.

Acceptance is compositional across the autonomous face episode: negative-y and
yaw progress are measured from its initial latched pose, while the current
successor must still complete the configured physical/geometric dwell. A
rotation-dominant contact releases at the conservative 2 mm pre-guard. A
translation-dominant contact can use the remaining corridor to finish the
dwell, but the unchanged 5 mm lateral tolerance remains mandatory. Full
`open_table` completion remains pending repeated accepted face cycles to the
unchanged terminal translation and orientation tolerances.

Contact tracking now applies a second swept whole-capsule predicate before
publishing every Cartesian command. It reconstructs the live MJCF capsule from
the measured tip and stick axis and checks the centerline against both
radius-expanded T boxes and the table at nine samples through the command. The
selected finite face admits a bounded cylindrical-contact band down to the
existing activation tolerance; the other box, opposite/deeper intersections,
and table remain forbidden. A failed receipt holds the measured tip for one
update, latches the failure, and enters the ordinary collision-free reposition
transaction on the next solve. The measured capsule is checked independently
so a command rejection is not misreported as an existing collision.

Measured corrective recovery also has a wrong-polarity exit. After the
unchanged successor dwell, growth of absolute lateral error by at least the
existing derived 1 mm recovery threshold emits
`measured_wrong_polarity_response=REJECT` and selects another measured-lateral
face. Nominal force direction alone is never accepted as response evidence.

The first 300-second diagnostics-off replay proved the fail-safe path: ten
unsafe shaft commands produced ten explicit clearance receipts and ten passing
collision-free repositions, with no top contact. It also exposed a stricter
prior five-DOF feasibility blocker: every intended contact command was rejected
before Drake measured force, so the wrong-polarity runtime branch had no
physical episode to evaluate. Two preserved orientation-conditioning
experiments either displaced the tip or stalled below the geometry-derived
shaft requirement and were reverted. Contact safety is implemented; physical
contact acceptance remained pending a capsule-feasible contact-posture policy.

The next implementation commands OIM's local +Z stick axis vertical-down via
the existing differential-IK trajectory during descent, engagement, and
contact. Contact handoff additionally requires a swept receipt from the live
posture to the selected loaded face. The deterministic witness admits a
vertical cylinder at that face and rejects an inward-inclined shaft at the same
tip. A diagnostics-off Drake replay then established eight physical side-force
episodes at z=34.41--34.82 mm, below the 59.6 mm T top, while every reported
contact command retained `shaft_t_clear=1`. This accepts the first physical
face. Successor engagement still requires reachability conditioning: the next
`crossbar_right` target stopped 8.975 mm short, so second-face and terminal
acceptance remained pending at that revision.

Successor engagement now uses a velocity-bounded, measured-state-seeded IK
posture step with a vertical capsule-axis constraint. Every interpolated joint
sample must pass the selected-face capsule receipt before the posture OSC is
activated. This produced a second physical `crossbar_right` force contact.
Every successor engaged outside the measured lateral guard also owns a
measured recovery transaction, so wrong-direction growth after the unchanged
dwell cannot be bypassed by a capsule- or progress-triggered switch.

Loaded recovery projects its command onto the selected face tangent after
normal contact is established. Successor arbitration excludes the empirically
unsafe crossbar-top perimeter after initial acquisition. A blocked lateral
traverse first attempts fixed-tip collision-aware posture recovery; if that is
also infeasible, execution holds the measured state and replans rather than
publishing an unsafe command or throwing. Physical terminal acceptance remains
pending because preserved attempts still observed upper-perimeter contact near
the T top before this final hold/replan revision.

Acceptance is reset after every passing cycle. The new cycle baseline stores
both the measured T pose and the completed-reposition count, so a later receipt
cannot reuse a prior handoff or prior progress. The bounded 2,000-step face
watchdog evaluates negative-y and yaw against their existing YAML minima and
reselects a task-progress face when either component is deficient. If the
inclined shaft makes all lateral successors unsafe at the low posture, the
watchdog may select a staging face only for vertical release; the elevated
traverse state must revalidate shaft clearance and task direction before any
lateral motion. Cycle, handoff, geometric-contact, switch, and collision-free
receipts carry the same acceptance-cycle number for log correlation with the
simulator's physical-contact episode counter.

Post-yaw candidate availability is fail-closed at two layers. If both measured-
velocity and quasistatic workspace-filtered C3+ buffers are empty, dynamically
accepted and input-bounded contacts are replenished from the preserved rejected
buffer; they still require measured xArm live IK and whole-capsule execution.
At the shared high neutral anchor, a failed measured-seed verticalization may
retry nonlinear IK from the configured home seed. Its velocity-clipped joint
step is publishable only when the swept capsule stays clear, vertical-axis
error decreases, and tip drift remains inside the existing activation
tolerance.

The OSC QP uses DAIRLab's persistent `FastOsqpSolver`, which reparses the
updated mathematical program and rebuilds an OSQP 1.x workspace each control
update to tolerate changing sparsity. `EigenSparseToCSC` allocates its three
CSC arrays; the resulting OSQP matrix now owns those arrays so the existing
per-rebuild `OSQPCscMatrix_free` releases them. Solver destruction also owns
and releases the remaining workspace, matrices, and settings. This is an
ownership correction only: cost/constraint construction, floating-point
ordering, OSQP setup/solve ordering, options, and warm-start policy are
unchanged. The OSC reads the named LCM trajectory block directly rather than
constructing four temporary trajectory maps; it still builds the identical
two-sample first-order hold for the tracking input.

Contact-cycle admission now uses one accounting domain. The selected live-IK
path length is converted from its 50 ms planning discretization into 20 ms
controller updates, then added to the unchanged physical contact dwell and the
largest measured release/recovery receipt from the current rollout. A cycle is
admitted only when the remaining execution budget is strictly larger than this
sum. Missing measurements and equality fail closed. This guard is downstream
of candidate selection and live IK but upstream of the first physical lift, so
budget rejection publishes no partial acquisition command and preserves the
measured safe hold. It adds no solver, tolerance, cost, horizon, or YAML change.

Live-IK admission and physical acquisition are now linked by an explicit
conformance transaction. Seven ordered physical phases correspond to the
selected preview. A measured task-space regression quarantines the selected
candidate; early failures skip candidate-face release because no candidate
contact exists. Replanning is fail-closed until the pusher returns to the
elevated home-tip anchor with a vertical capsule and the preceding cycle's
terminal evidence remains present. This separates safe retry permission from
task success: a recovered divergence is a passing conformance receipt, not a
productive manipulation cycle.

Physical phase handoff consumes a fresh robot measurement rather than the
subscriber sample that preceded a potentially long Sampling-C3+ solve. Joint
velocity channels are imported into the same measured acquisition context, and
arrival additionally requires that one planning interval of measured tip
motion remain inside the existing activation ball. The task-space regression
envelope is anchored at phase entry, so bounded convergent transients do not
look like a rejected preview while a phase that retreats past its admitted
start still fails closed.

Cycle productivity is owned by the final measured state of the entire physical
transaction. The contact-response pose remains valid model-residual evidence,
but task translation, orientation, and lateral acceptance are recomputed after
release and corrective recovery. Candidate quarantine and the productive-cycle
counter consume this post-recovery receipt, preventing an intermediate contact
response from receiving credit for progress that recovery later removes.

Corrective recovery retains the same global object goal as the primary C3+
problem. Its candidates must be predicted nonregressive in both terminal
translation and orientation before the existing polarity, live-IK, and capsule
checks may authorize motion. This is a prediction-side safety condition only;
the productive-cycle counter still consumes the measured post-recovery receipt.

Within that admissible recovery set, predicted translation and orientation
descent are normalized by their unchanged terminal tolerances and stable-ranked
by the resulting dimensionless magnitude. The selected predicted receipt is
retained through physical recovery and compared with the post-recovery pose.
This comparison is evidence, not acceptance: physical Pareto descent remains
the authority when the C3+ rollout overpredicts the measured response.

Recovery contacts participate in measured-response conditioning. Their model
prediction and final post-recovery measurement are stored in the existing
pose/contact-neighborhood history. Candidate ordering first applies the
existing compatible/unseen/incompatible evidence classes and only then applies
normalized predicted or corrected descent. Unseen contacts remain explorable;
an observed incompatible neighborhood cannot outrank them.

Candidate availability has a fail-closed component-decomposition fallback.
Only after the combined subgoal and dynamic-contact replenishment are empty,
translation-only and rotation-only C3+ batches are generated from the same
measured state. Decomposition affects proposal generation only: the unchanged
global Pareto, response, corridor, live-IK, and capsule gates still decide
whether any proposal may execute.

Corrective-response transfer is rotationally equivariant. Model residuals are
stored in the T object's frame and rotated into the current world frame before
correcting a new candidate. Recovery observations have a dedicated history;
the corrected world-frame terminal must still pass the unchanged global Pareto
and lateral gates before its evidence class can improve.

Preview recovery prefers the fixed elevated home anchor. If it is unreachable,
the already-clear measured lift endpoint may serve as a reachable anchor only
after vertical posture IK and the existing whole-capsule clearance both pass.

An equivariantly corrected recovery with incompatible measured evidence is
quarantined before live IK. Incompatible evidence is no longer an execution
fallback; only compatible or unseen contact neighborhoods may reach geometric
feasibility checks.

Recovery preview and physical execution share a controlled vertical-escape
contract for their first lift. It admits an initially intersecting shaft only
when height increases, outward motion is nonnegative, and the endpoint clears
the table constraints; subsequent waypoints require normal swept clearance.

Recovery proposal exhaustion has a deterministic seed-replenishment layer.
Four additional perimeter/mesh seed pairs are generated only after the
canonical set fails, and retry provenance is encoded in candidate names. No
retry bypasses response quarantine or physical feasibility gates.

Corrective C3+ objectives are receding-horizon reachable: lateral x remains at
its global target, while y and yaw subgoals are bounded by their unchanged
terminal tolerances. Global prediction and measured acceptance remain separate
and continue to use the final open-table pose.

Recovery response neighborhoods use the physical pusher diameter for contact
position locality. The terminal translation tolerance no longer causes one
observation to cover an entire T face; yaw-equivariant transfer otherwise
retains the same corrected global acceptance rules.

Preview recovery has a third anchor after fixed-home and in-place posture IK
fail: the selected candidate's collision-validated overhead x-y point at the
measured lift height. It does not bypass IK or swept whole-capsule checks. A
failed anchor leaves the candidate invalidated and preserves the terminal
receipt.

When every elevated anchor is posture-IK infeasible, preview recovery may
de-elevate position-only at the measured clear x-y point to
`CapsuleObjectClearanceHeight`. The descent and subsequent in-place
verticalization each retain interpolated whole-capsule checks; this is a
sequencing fallback, not a new height parameter or collision tolerance.

Terminal status uses cumulative physical provenance. A productive cycle
remains a valid closed-loop handoff after a later measured-budget defer, but it
does not imply open-table success. A pure status evaluator independently emits
handoff, global terminal acceptance, reason, and process return code; a defer
without any productive cycle remains fail-closed.

Primary response conditioning also exposes a dimensionless physical
productivity gain: accumulated measured normalized Pareto descent divided by
the corresponding C3+ prediction. Compatible candidates are ordered by the
gain-calibrated magnitude; unseen exploration and incompatible quarantine keep
their existing evidence-class precedence and all geometric gates still apply.

Primary contact wrench admission is Pareto-consistent with task descent. The
translation and orientation alignment products must both be nonnegative and
at least one strictly positive. A yaw-helpful contact cannot be admitted when
its normal force opposes required translation, or vice versa; no force or
moment magnitude threshold is added.

Lateral recovery is event-aligned. Candidate prediction stops at the first
state inside the unchanged x corridor, as physical dwell already does, instead
of judging a later horizon overshoot. Selection requires corridor restoration,
global translation nonregression, and orientation debt no larger than the
unchanged terminal orientation tolerance. Measured post-recovery Pareto and
2 mm reserve receipts remain the authority for productive-cycle credit.

Terminal budget reporting is measurement-derived and explicitly optimistic.
It combines the maximum observed productive progress in each task component
with the minimum observed complete-cycle update cost to emit a lower bound on
remaining cycles and updates. It never changes admission, benchmark budget, or
the global terminal acceptance gate.

Clearance-height de-elevation is measurement-conditioned. Its target owns only
z: x-y is relatched from measured tip state on every command substep, so an LCM
refresh cannot convert a vertical recovery into an unpreviewed diagonal move.
The existing step limit, position-only IK, and swept whole-capsule checks are
unchanged.

Lateral recovery candidate admission has an explicit transaction-debt model.
Restoring x may temporarily spend no more than the unchanged terminal
translation and orientation tolerances. This does not create task progress:
the subsequent measured post-recovery receipt remains strictly nonregressive
in both global task components before productive-cycle provenance is granted.
