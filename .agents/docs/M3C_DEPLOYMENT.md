# M3C SchurVINS deployment evidence

For the laptop physically connected to M3C, follow the
[laptop test and return runbook](M3C_LAPTOP_HANDOFF.md). It covers the separate
checked runtime/data transfer, board identity and ABI checks, mono diagnostics,
and raw-result handoff to this development host. Git pull alone does not include
the binary or public dataset. The second laptop return, session
`m3c-20261102T080438Z`, establishes actual M3C execution and resource measurements.
All four complete monocular runs **FAIL**: one pose, one tracking loss and no
verified metric initialization. Host scoring confirms ATE/RPE are unevaluable.
The next software gate is a corrected, validated PC monocular baseline before
issuing a replacement ARM64 runtime; repeating this unchanged package is not a fix.

This records the independent baseline and both actual M3C laptop returns. The
[shared integration record](SCHURVINS_INTEGRATION.md) describes the imported
SchurVINS research baseline; it is not evidence of execution on M3C.
OpenVINS and the RK3576 worktree remain separate and unchanged by this task.

## Authorized joint bootstrap/filter repair (active candidate, not qualified)

The user now authorizes committing and pushing the unfinished source as a **WIP
checkpoint**, before a possible change of direction. Publication includes the
current joint engine, runtime, patch, native fixtures and this evidence record;
it does not qualify the candidate or authorize hardware operation. The latest
engine batch still has no subsequent fresh complete build/suite/full replay.
No replacement ARM64/laptop package is issued. Raw sources, recordings, builds
and local diagnostic artifacts remain outside Git.

The 704c40a first-failure evidence and all prior failed candidates remain
unchanged. The current user authorizes implementing the missing statistical
handoff inside SchurVINS/SVO; this is a software task, not a board dependency.
The candidate is built in the new `R/joint-handoff/candidate` tree and must pass
production-path tests and full sensor-only mono/stereo replays before promotion.

The implemented contract is a persistent joint raw-factor window. Each camera
state contains body-to-world quaternion, world position and velocity, and body
accelerometer/gyro biases (15 local coordinates). Quaternion perturbations are
right/body, `q <- q Exp(dtheta)`; other increments are additive in their stated
frames. Landmarks are world XYZ. Gravity is an estimated persistent world 3-vector,
including its cross-information; the first pose rotation/translation define the
six-dimensional gauge exactly. There is no free similarity/scale gauge once SI
IMU factors are informative, and no post-hoc fitted scale or gravity rotation
that discards gravity uncertainty.

Raw camera bearings use actual extrinsics, focal lengths, configured pixel noise
and existing robust loss. Raw IMU spans are clipped to exact camera boundaries,
with the active configured continuous measurement noise and bias random walks, complete
15-dimensional preintegration covariance and bias derivatives. Only the configured
initial velocity/bias distributions are priors. SVO geometry and the original
alignment are seeds/diagnostics, never independent information priors; neither a
fixed diagonal point covariance nor the old conditional pose Hessian is exported
as a joint posterior. Existing physical, residual and observability gates apply
to the actual estimated solution before metric/tracking success is published.

Camera elimination and point recovery use the same linearized joint objective.
Active raw factors remain in that objective and are relinearized rather than
re-consumed as independent data; posterior means only seed the next solve.
When dropping an old pose, eliminate its velocity/bias and all landmark track
segments it observed, including every observation of those segments at retained
poses. Absorb those incident factors and the complete applicable previous prior
into one rank-revealing state/gravity marginal prior. Nonincident factors remain
raw. Future observations of a retired physical landmark start a free segment:
the previous point mean is a numerical seed only. This deliberately discards
cross-segment identity constraints, a conservative loss of information, and avoids
an unbounded dense point-point prior. Original measurement identities are global
(epoch, frame timestamp, camera, feature index), independent of segment IDs;
absorbed observations cannot be re-added under a new segment. Prior linearization
is frozen and its coordinate convention is retained explicitly.

This candidate does not establish that the missing contract is the unique root
cause. No new accuracy threshold is invented: complete execution, continuous
scorable tracking, precision and operational validity are reported separately.
Applicable predeclared absolute Schur ATE/RPE precision acceptance remains BLOCKED.
Hardware operation and replacement ARM64 packaging remain outside this
checkpoint; WIP source publication is separately authorized as stated above.

### Implemented PC candidate and current evidence

**Stopped at the user's next-step boundary.** The latest source has targeted
locked-GCC tests PASS for restored inherited three-view/1km admission, explicit
pending-factor retirement, and LM scaling within the same undamped point
subspace. Exact C4 evidence identifies two-view point436 growing to4.46km;
its near-null depth eigenmode is spuriously activated by old damping scaling,
producing a14km trial step. This is a new-path admission/numerical-model defect,
not evidence reopening the excluded old Schur solver hypothesis. Iteration
budgets, physical gates and numerical convergence tolerances were not relaxed.
The new captured-graph counterfactual converges in5iterations, but ALL241current
visual rows are pending; it is inertial+old-prior numerical evidence and **not a
visual correction or VIO PASS**. The old prior itself came from the older model.

This final engine batch is **unintegrated/unqualified**: the runtime still needs
pending/warmup handling, admitted mature valid counts and the inherited3px mean
point rejection. No fresh C5 build, full suite or full replay followed this batch.
`R/joint-handoff/stopped-state.json` and the scoped `stopped-source.patch` bind
that exact source; the unique activity plan records the next implementation step.
The C3 partial V103 replay was explicitly terminated on user request; all its
raw output is retained and marked incomplete, not scored as a complete failure.
Previously completed results retain their original status.

Snapshotv1 had proven16-digit loss in the locked old yaml-cpp Node conversion;
its reloaded model is approximate and must not be called an exact reproducer.
Snapshotv2 encodes and bit-checks every source double before writing; locked-GCC
roundtrip tests retain the original H/b/cost assertions and PASS. Exact C4
source/iterate snapshots and point436 dense blocks are preserved.

Noise provenance is also explicit: covariance currently uses inherited upstream
YAML densities, not the supplied sensor-metadata model. Both datasets' metadata
specify200Hz and(acc,gyro,accRW,gyroRW)=(.002,.00016968,.003,.000019393), while
the pinned unchanged mono YAML says800Hz and(.008,.0012,.1,.03). Integration
uses actual timestamps, not the800Hz field. `sensor-model-provenance.json`
records a metadata-derived calibration prepared but NOT_RUN; no covariance
values were fitted. The initial velocity/bias distributions remain configured
assumptions, not a verified dataset posterior.

`R/joint-handoff/result.json` indexes immutable candidate patches, fresh builds,
raw complete replays and post-run fixed-scale scoring. Candidate2 implements the
contract above in the actual mono FrameHandler/SchurVINS path; stereo still uses
its validated existing update. It is **not a qualified replacement runtime**.
Native C++10/10 and software587/no-skips PASS. The captured original four-frame
public-API packet admits4frames/3IMUedges/638unique pixels/193points and correctly
refuses metric readiness because its bootstrap history is absent; the common raw
objective falls1016645.17→10.468, not evidence of full VIO success.

Two independent lifecycle/coordinate fixes are established. First, retirement
cleared frontend landmark pointers before removing their reverse observations.
After the delayed last-removed-keyframe owner was released,148 distinct surviving
points referenced expired frame0 and frame148 matching collapsed to40. The actual
frontend regression FAILs on the preserved first candidate and PASSes after
`removeObservation(frame_id)` precedes pointer clearing. Second, the raw-factor
landmark outlives a deleted SVO Point; seeding only live weak pointers left latent
points in visual units. All latent means now receive the same camera-origin
similarity, with checked transactional setters. A real V101 witness has2expired
points, V1036; source→scaled camera-coordinate errors are1.31e-14/5.16e-14m,
while the old skipped-point coordinates remain unscaled. Independent YAML-prior
native coverage destroys the frontend owners before seeding and checks the
persisting factor landmark. These fixes do not establish unique VIO causality.

Candidate2 full sensor-only V101/V103 mono process execution PASS, but continuous
VIO **FAIL**:9/24poses and one tracking loss each, all2912/2149camera and
29120/21500IMU inputs delivered. Both pass the unchanged physical joint bootstrap
gates; that is not proof the metric initialization is correct. The next observed
refusal is numerical tracking solve at bundles155/198, being captured for exact
same-graph replay. No full mono success, repeated-success claim or deployment
promotion is made. Candidate1's native abort/failed replay remains unchanged.
Stereo numerical controls complete2911/2149poses with zero losses, ATE
0.047677/0.065406m and RPE1s0.042490/0.033189m. These are diagnostic fixed-scale
scores; absolute precision acceptance remains BLOCKED. Five repeats, paced valid
VIO resources, stability and a new ARM64 package remain NOT_RUN pending full mono.

## Qualification

| Boundary | Result | Evidence / limitation |
|---|---|---|
| Independent PC ROS build | PASS | All 26 packages built from the pinned dependency closure |
| PC ROS-free C++ build | PASS | Real estimator/SVO sources, no ROS dynamic libraries |
| ARM64 ROS-free C++ reference build | PASS | AArch64 ELF, GNU 7.5/Bionic reference sysroot; this checked package now also executes on the observed M3C |
| ARM64 execution under QEMU | PASS stereo numerical replay / FAIL mono | Checked runtime: full 2912 images / 29120 IMU; stereo 2911 poses, zero loss; mono one pose and loss |
| Full PC mono V1_01 / V1_03, ROS runner | FAIL | Trajectory not evaluable; process exit zero is insufficient |
| Full PC mono V1_01, ROS-free runtime | FAIL | 2912 cameras, 29120 IMU samples admitted, one pose and one tracking loss |
| Full PC stereo V1_01 at 1× | PASS for numerical replay | 2911 poses, one explicit input-prefix rejection, no observed tracking exit |
| Actual M3C identity / OS / SDK inventory | PASS inventory / PARTIAL SDK | Laptop confirmed attached device; observed MaixCAM2 DT, axera,ax620e, AArch64 Linux; SDK candidate paths are not SDK qualification |
| Actual M3C checked package / loader help | PASS | 6,226 file hashes match; checked binary `--help` exits zero, not an estimator replay |
| M3C unpaced offline replay | PASS execution / FAIL estimator | Full V1_01 input, one pose and loss; fixed-scale accuracy unevaluable |
| M3C paced 0.5× / 1× replay | PASS execution / FAIL estimator | Three complete paced runs including sampled 0.5×; same failure boundary, not live sensor operation |
| M3C native resource measurement | PASS measurement only | Private GNU time and Linux sampler work; measurements describe the failed mono pipeline |
| Real OS04A10 + IMU VIO | NOT_RUN | New capture needs authorization; timing/calibration/input chain unqualified |
| Physical PX4 UART | NOT_RUN | Safety/configuration confirmations required |
| 60-minute qualified VIO endurance | NOT_RUN | Requires a valid mono baseline; short failed replays do not qualify |

Numerical replay PASS is scoped to complete selected public-data execution,
fixed-scale scoring and observed output continuity. It does not establish
operational tracking validity, accepted visual corrections, flight readiness or
the existing 1 cm repeatability requirement. No gate was weakened.

## Implementation and reproducibility

The branch starts at `a8392c6f1e32e76532bd3ea8aa34ef98e0f06da6` and imports
only the reusable dependency closure from the protected dirty worktree.
`source-import.json` records each imported and protected dirty-file hash.
Implementation and validation initially remained local. The user authorized
committing and pushing this work to `origin/research/m3c-vio` on 2026-10-31;
publication does not change the hardware qualification or authorize a merge/PR.
Large artifacts live in the independent
`/home/shiuhou/Projects/rm27-m3c-schurvins-research` directory, abbreviated `R`
below. A fresh checkout needs the pinned third-party sources and public input;
these external paths are evidence locations, not repository dependencies.

SchurVINS pin: `d8ab6dff20860694d8d879d716d7fd6cb2c2530e`.
The PC build uses locked image
`sha256:b4a9eb291ae6c65fd80d2841bb225c5bf86658f6c0a58cdca710ce1265cfc34a`.
`R/headless/build-manifest.json` records source, compiler, executable and library
hashes. A subsequent output-write fix is separately recorded in
`R/headless/checked-build-manifest.json`: effective params, frame/pose streams
and summary now fail on open/write/flush/close errors. Estimator static library
and factory object hashes are unchanged. The original measured binaries and
runs are preserved. The checked ARM64 package at
`R/headless/arm64-bundle-checked/` contains 182 loader/library dependencies,
parameters, calibration and notices, about 148 MB total. Its wrapper was tested
with a relocated path containing spaces; post-replay hashes verify both bundles
unchanged. The ARM64 compiler/sysroot and opengv artifacts were read from the
existing reference research without changing them. The laptop returns below
supply observed RAM, Linux, libc and device owners. The checked runtime completed
actual target estimator execution; SDK and custom carrier qualification remain pending.

The [standalone C++ runtime](../../tools/schurvins/headless/README.md) replaces
the ROS parameter/factory/input/output boundary with YAML and ASL input. It
retains estimator equations and the mono initializer. SDK/driver dependencies
are outside the estimator. Automatic restart, loop closure, global mapping and
Ceres backends are explicitly disabled, and IMU use is enabled. OpenCV GUI
libraries remain linked through upstream helpers; execution opens no window.

Example full replay from the repository root, using a matching native loader:

```sh
export R=/home/shiuhou/Projects/rm27-m3c-schurvins-research
"$R/benchmark/pc-runtime/lib/ld-linux-x86-64.so.2" \
  --library-path "$R/benchmark/pc-runtime/lib" \
  "$R/headless/build-pc/schurvins_headless" \
  --dataset "$R/inputs/v101" --mode mono \
  --calib "$R/headless/src/SchurVINS/svo_ros/param/calib/euroc_mono.yaml" \
  --params "$R/headless/src/SchurVINS/svo_ros/param/vio_mono.yaml" \
  --output "$R/new-mono-run" --rate 0.5
python3 -m tools.schurvins.headless_evaluate --run "$R/new-mono-run" \
  --archive /absolute/path/to/V1_01_easy.zip --output "$R/new-mono-evaluation.json"
```

The evaluator is expected to return failure for the current mono baseline.
It never feeds ground truth to the estimator or fits scale/time offsets.
Each output directory must be new. Source integer camera times are retained;
conversion to upstream double-second IMU APIs remains a precision limitation.
The first V1_01 camera has no strictly older IMU sample and is explicitly counted
as rejected instead of inventing a pre-roll sample. The original recordings,
failed runs and raw artifacts are preserved.

## Monocular diagnosis

The unchanged upstream mono path enters Tracking after arbitrary-scale geometry
without a verified metric Schur initialization. Existing imported derivative
work is retained as a candidate patch, never enabled silently. Its historical
first accepted correction left only 25 valid features against the unchanged
minimum 40; a newer historical run rejected all 2762 metric alignment attempts.
A bounded execution of the production alignment header on recorded bootstrap
poses accepted 0/9 windows. A separate truth-only mathematical oracle accepted
8/8; replacing only rotations accepted 0/8, while replacing only translation
accepted 6/8. This points to visual bootstrap translation/IMU consistency as the
main observed limitation, and does not prove a specific frontend code defect.
Truth was never given to VIO. No scale fit, threshold, covariance or estimator
math change was used to force acceptance. The reproducible diagnosis and hashes
are in `R/mono-diagnosis/provenance.json`. Mono remains **FAIL**.

The second board return localizes the unchanged runtime's failure more precisely.
PC and all five board diagnostics have identical source-index transitions:
camera index 110 / 5.50 s enters Tracking, index 111 / 5.55 s enters
Relocalization, index 116 / 5.80 s returns to Paused (indices start at zero).
The complete runs then remain paused for the rest of the source sequence.
The first Schur Forward event has state timestamp -1 and zero position;
`FrameHandlerMono::processFirstFrame` committed the geometric map without a
metric handoff. In `SchurVINS::Forward`, `InitState` starts from the current
accelerometer with zero position/velocity and writes the propagated state into
the camera pose. The subsequent first reprojection retains only 1–2 matches,
below the unchanged minimum 40, and fails before any Schur Backward call.

This source call path and failure boundary are observed on both architectures.
They identify an initialization handoff defect to address, but do not prove a
minimal numerical repair or that every visual failure has that sole cause.
The previously tested metric candidate still fails its visual/IMU consistency
checks; simply assigning the arbitrary-scale pose to the filter would not
establish metric initialization. No estimator, scale, calibration, threshold or
automatic-restart change was introduced in reviewing the return.

### Isolated repair experiments, 2026-11-02

The opt-in `tools/schurvins/mono-initialization.patch` now also fixes three
independently reproduced lifecycle defects. Mono tracking quality compares
mature Schur observations with their own previous count, preserving the existing
minimum 40, drop 40 and keyframe exception. Reset/destruction detaches the
frame–landmark ownership cycle. Backward applies each unique landmark's
conditional correction before evaluating camera–point residuals, and the later
frontend path no longer applies that correction again. These changes are research
code; normal source preparation and the distributed runtime do not enable them.

Actual production API fixtures cover quality, reset/destruction, window
management and exactly-once joint correction, alongside the existing alignment
fixture; all five compiled CTest cases PASS. The joint test checks an independent
information-form covariance identity, not a copy of the production update.
Real-data shadow instrumentation
also showed one stale-point rejection changing from 0/90 to 49/90 inliers after
the joint correction. Neither this example nor fixture PASS qualifies mono VIO.

The final original-alignment control retains the original solver and physical
gates, with the three lifecycle fixes and synchronous depth filtering:

| Complete PC diagnostic | Cameras / IMU | Poses | Result |
|---|---:|---:|---|
| V1_01 FivePoint | 2912 / 29120 | 0 | FAIL: metric alignment rejected |
| V1_01 TwoPoint with IMU rotation prior | 2912 / 29120 | 0 | FAIL: metric alignment rejected |
| V1_03 FivePoint | 2149 / 21500 | 0 | FAIL: metric alignment rejected |
| V1_03 TwoPoint with IMU rotation prior | 2149 / 21500 | 0 | FAIL: metric alignment rejected |

All four processes exit zero and consume the complete selected input; none has
an evaluable trajectory. Zero tracking losses here means Tracking was never
established. Across all 20 complete mono diagnostic trials, none passed.
Other full candidate trials reached only 1–8 poses before losing
tracking and also FAIL. A cumulative-position alignment improved some synthetic
noise cases but changed the observation weighting and regressed recorded oracle
windows; it was not promoted. Exact covariance elimination reproduced the old
solver and its scale attenuation. No camera covariance, scale fit or threshold
was invented to force acceptance.

The changed joint-update path completed full stereo controls: V1_01 had 2911
poses, zero loss, ATE 0.039515 m / RPE 1 s 0.042618 m; V1_03 had 2149 poses,
zero loss, ATE 0.053281 m / RPE 1 s 0.033937 m. These are single diagnostic
numerical regressions, not accuracy improvements or mono acceptance. A separate
28-update comparison with an equivalent information-form solve found maximum
state disagreement 2.98e-8, providing no basis for a speculative solver change.

Sources, binaries, full failed runs, post-run fixed-scale evaluations and native
test evidence are preserved under `R/mono-repair/`; `repair-result.json` indexes
them. The remaining software problem is statistically consistent visual–inertial
bootstrap and sustained accepted updates. The current bootstrap does not export
a justified joint visual pose/map covariance. This is an unresolved algorithm
and observation-model issue, not a missing M3C dependency. There is **no new
qualified mono baseline or replacement ARM64 package** from these experiments.
The laptop should retain its existing raw results and await a validated package.

### Continued repair: validated defects, monocular gate still FAIL

A continuation from `408c2f3` retains three additional reproduced defects in the
same opt-in patch. Rejected current image associations now leave both the point's
local observation index and the cloned state's feature list; previously the next
Solve reused the rejected bearing. The public-API regression observes the next
actual update and preserves all 39 uninjected points and their 156 observations.
`Frame::resetKeyPoints()` replaces all five entries instead of resizing an
already-five-element vector, so changed map geometry and removed feature indices
cannot leave stale overlap coordinates. The explicit-ID `FrameBundle` constructor
now initializes its own ID as well as its child frames, preserving historical
observation ownership during relocalization. Existing observation-count,
Reset/destructor and joint-update-order fixes remain intact. No threshold,
alignment equation, calibration, covariance or success flag is changed in the
retained patch. Source preparation still does not apply this research patch.

A separate raw-bearing camera/landmark refinement candidate recovered a synthetic
known scale (2.29996 against 2.3) with unchanged alignment gates, but its first
complete V1_01/V1_03 monocular replays still failed (one/zero poses). A subsequent
candidate made one camera/point update agree with an independent dense MAP solve
using the existing priors and historical keyframes. It improved a V1_01 fragment,
but repeated filtering produced a catastrophic V1_03 stereo ATE of 640.37 m.
The generic numerical evaluator returned PASS for that continuous output; this
was **not accepted as a valid regression**. One-update equivalence does not
establish the repeated state/landmark correlation model. Both candidates and
their exclusive fixtures were removed from the repository implementation;
source/binary snapshots, raw failures, post-replay scores and diagnostics remain
external. No covariance was invented or gate relaxed to retain either candidate.

Restoring older source mtimes also exposed a local incremental-build pitfall:
the `final-*` and `retained-*` trial labels do not establish source/binary parity
and are excluded from final validation. Their raw records remain unchanged.
Final verification used a **new empty `build-clean` directory**, compiled every
translation unit, checked the final source delta against the handoff, and applied
the complete research patch to a fresh prepared source with byte parity.
The final compiled source differs from the handoff source in only the three
owning files described above. Clean CTest passes **7/7**; full sourced-ROS2 Python
regression passes **587/587**, no skips, including the actual C++ boundary tests.
Earlier unsourced Python skips and an unrelated ROS pytest-plugin import failure
are retained as environment diagnostics, not counted as passes.

| Clean final PC replay | Cameras / IMU | Poses / losses | Fixed-scale result |
|---|---:|---:|---|
| V1_01 mono | 2912 / 29120 | 1 / 1 | FAIL; ATE/RPE unevaluable |
| V1_03 mono | 2149 / 21500 | 0 / 0 | FAIL; never reaches Tracking; ATE/RPE unevaluable |
| V1_01 stereo | 2912 / 29120 | 2911 / 0 | Numerical control PASS; ATE 0.041284 m, RPE 1 s 0.043090 m |
| V1_03 stereo | 2149 / 21500 | 2149 / 0 | Numerical control PASS; ATE 0.062850 m, RPE 1 s 0.034020 m |

All four consume the complete sensor-only input and exit zero. V1_01 mono has
one metric commit and seven observed visual corrections, but still has no
continuous/evaluable trajectory; neither an event nor a process exit certifies
metric initialization. V1_03 has no metric commit. Ground truth is read only
by post-replay fixed-scale SE(3) scoring. Stereo results are single regression
controls, not five-repeat acceptance or evidence of accuracy improvement.
Five monocular repeats and a replacement ARM64 build/package are **NOT_RUN**
because the initial full monocular gate failed. The distributed M3C binary and
laptop replay/return instructions remain unchanged; no hardware was connected.

`R/mono-repair/metric-bootstrap/continuation-result.json` indexes all 16 complete
trial records, including failed/retired candidates and six trials with unverified
incremental source/binary association. Final acceptance uses only the four
`clean-*` records, `clean-build-*`, `clean-python-*` and
`retained-patch-verification.json`. The original `mono-repair/repair-result.json`
and previous board sessions remain immutable. The protected worktree was only
read for a 63-file hash audit; seven changes since the prior handoff were observed
and left untouched. No build or write targeted either protected path.

### First-failure localization from 704c40a: no safe local repair established

This round writes only the M3C worktree. No commit, push, merge, ARM64 build or
hardware operation is authorized. Prior `clean-*` evidence and retired-candidate
failures remain unchanged. New source, a fresh empty diagnostic build and all
new artifacts are under `.agents/runs/m3c-first-failure-704c40a/` (abbreviated `D`).
`D/result.json` binds the baseline research patch, diagnostic source manifest,
compiled binary, complete replays and the counterexamples below. Estimator
math, gates and the tracked research patch are unchanged; the candidate source
adds bounded read-only diagnostics only.

The trusted clean V1_01 metric commit is bundle 674, followed by its first
mature correction at 676 (124 surviving observations), then failure at 677
(one survivor). Seven correction events do not establish correct metric
initialization. Clean V1_03 has no metric commit: 1,016 alignment attempts
contain 979 gravity, 35 accelerometer-bias and two translation rejections.
Its first translation rejection already has about 11.06 m/s² estimated bias;
the reported reason does not mean translation is its sole problem.
Both sequences use the same alignment contract and physical gates.

The new diagnostic V1_01 replay still fails, but initializes at bundle 338,
not 674. It must **not** be presented as the exact earlier clean window.
The numerical-window difference is retained as a reproducibility limitation;
this round does not establish its origin or infer it from matching pose counts.
The following detailed residual/covariance evidence belongs to that separately
recorded diagnostic sequence:

| Current mature observations | Before correction median / P95 px | After camera step | After point step | Above existing 4 px gate after both |
|---|---:|---:|---:|---:|
| First actual update, bundle 340, 136 observations | 0.773 / 2.514 | 3.436 / 7.394 | 2.141 / 4.713 | 15 |
| Next update, bundle 341, 136 observations | 2.544 / 6.837 | 6.482 / 11.284 | 5.290 / 10.303 | 109 |

The first complete joint step already creates rejected residuals, before any
previous filter visual update can have been reused. The next update reuses 391
local and 665 historical indexed observations and leaves 27 valid current
features. Reuse is observed; it is not by itself proof of an invalid likelihood.
No local/historical duplicate was found *within* an update. Finite covariance
and approximately 1e-16 negative clone eigenvalues do not support a catastrophic
nonfinite/indefinite-covariance failure hypothesis. State/landmark correlations
remain unqualified.

A public-API reproducer eliminates image processing and private-state injection:
`D/reproducer/sensor-only-first-four.json` plus its calibration total 202,844
bytes. `InitializeMetric`, `Forward` and `Backward` reproduce valid counts
`[156, 0, 121, 27]` for bundles 338–341. All 17 recorded phases match point,
clone and selected-point identities; maximum scalar difference is 1.06e-8
(the 1e-6 comparison tolerance is numerical reproduction tolerance, not a VIO
precision gate). Sources, extraction, locked-container commands, hashes and
instructions are in `D/reproducer/README.md` and `comparison.json`.

A smaller self-contained counterexample is `D/minimal-point1013.json` (10,189
bytes), verified by `D/verify_minimal.py` with NumPy. It contains three local
and two historical rows, raw bearings, poses, operators and the actual update.
For this point, the current residual increases from 2.194 px before correction
to 5.562 px after both steps. Camera elimination uses the local free-point
objective, while the point correction adds historical rows and a finite prior
absent from that elimination. That objective mismatch is source-confirmed;
it does not prove that replacing one of the objectives yields a valid repeated
estimator model.

The bounded local-conditional counterfactual confirms the danger: it reduces
this point's local residual to 1.507 px but moves it 0.334 m and increases one
historical residual from 3.010 to 22.265 px. Across the first update, median
point displacement would be 0.951 m and historical squared residual cost would
rise from 1,258 to 98,430. This standalone correction is **REJECTED**, not inserted
into the estimator. It repeats neither the withdrawn BA nor the prior-MAP
camera-objective change. No covariance, noise or threshold is invented to make
it pass.

Other bounded falsifications are recorded separately:

- At metric similarity handoff, all 156 matched landmarks preserve unit-plane
  projection within 6.2e-13; their depths receive the intended uniform scale.
  This excludes a simple geometric handoff error for this capture, not incorrect
  *true* metric scale.
- Native re-solves of seven captured V1_01/V1_03 windows reproduce decisions;
  visual-scale renaming and rigid world yaw preserve physical estimates and
  decisions to numerical precision. A simple unit/gauge change cannot cure the
  observed gravity/bias violations.
- Summed clipped IMU integration durations equal camera spans exactly. Repeated
  sample timestamps in edge-containing delivery are interpolation support,
  not evidence of integrating the overlap twice.

The last two checks run from `D/counterexample.cpp` and are bound by
`counterexample-execution.json`. Neither proves that the accepted initialization
is statistically or physically correct. The common unresolved interface remains
visual bootstrap uncertainty and the joint state/landmark information supplied
to the filter; a unique common root cause has **not** been established.

All four new full sensor-only diagnostic replays finish with exit zero:

| Replay | Cameras / IMU | Poses / losses | Continuity and scoring | Precision acceptance |
|---|---:|---:|---|---|
| V1_01 mono | 2912 / 29120 | 1 / 1 | FAIL; trajectory unevaluable | NOT_RUN |
| V1_03 mono | 2149 / 21500 | 0 / 0 | FAIL; no trajectory | NOT_RUN |
| V1_01 stereo | 2912 / 29120 | 2911 / 0 | PASS; ATE 0.049445 m, RPE 1 s 0.042444 m | BLOCKED: applicable Schur precision criterion missing |
| V1_03 stereo | 2149 / 21500 | 2149 / 0 | PASS; ATE 0.052751 m, RPE 1 s 0.032623 m | BLOCKED: applicable Schur precision criterion missing |

The old generic evaluator outputs, including the 640.37 m `PASS`, remain
immutable. `headless_evaluate.py` certifies full-sequence numerical replay only;
it has no magnitude-based ATE/RPE gate. Historical 1 cm in
`OPENVINS_ADAPTER.md` is repeat-vs-native-run02 consistency, not absolute Schur
accuracy, and the Schur README explicitly forbids borrowing it as an algorithm
comparison allowance. The 200 Hz requirement concerns IMU input. No applicable
predeclared Schur absolute ATE/RPE limit or permitted regression margin was
located; none is selected retrospectively. Continuous/scorable therefore does
not mean accuracy accepted.

The fresh diagnostic build passes seven native tests, and the existing full
sourced-ROS2 regression passes 587 tests without skips. These check instrumentation
and software behavior, not VIO success. Repeats remain NOT_RUN because full mono
fails. No estimator repair is retained: the result is a localized first-failure
counterexample, excluded hypotheses and a rejected unsafe local correction.

### Bounded bootstrap/filter information-contract decision

The proposed bounded check is now complete on the same four-frame packet and
first correction. A read-only public-API rerun again returns `[156,0,121,27]`;
all 907 frozen source hashes match. No estimator or tracked research-patch
change, full-sequence replay, ARM64 build, hardware operation or publication is
part of this continuation. `D/joint-contract-replay.json` records the actual
rerun and independent numerical checks; `D/result.json` indexes the decision.

At one frozen linearization, a common joint Gaussian objective requires the
same point blocks in camera elimination and conditional point recovery:

```text
Hs = Hxx - Hxp inverse(Hpp) Hpx;  gs = gx - Hxp inverse(Hpp) gp
Hpp dp = gp - Hpx dx
```

`D/joint_information_check.py/json` reconstructs the executed first correction
from 136 points, 408 local observations and 668 historical observations. There
are no repeated observation keys or local/historical overlaps within this
update. The production point step and covariance are reconstructed within
1.33e-11 m and 1.09e-12 respectively. Camera elimination uses local `Vl,gl`;
point recovery instead uses `Vl+Vh+100I,gl+gh`. The `100I` comes from the existing
uncalibrated `0.01I` point prior. Using those latter blocks consistently in the
camera equation changes its Schur Hessian by 62.7% in relative Frobenius norm
and its reduced gradient by 434.4%. This is a material objective change, not an
equivalent numerical rearrangement. Adding fixed historical poses and the point
prior also changes the local visual rank from 11/18 to 18/18. These ranks describe
the frozen visual terms, not the observability of the complete inertial system.
Neither adding those terms to camera elimination nor dropping them from point
recovery is established as a safe estimator fix.

The executed bootstrap interface does not supply the missing joint model.
`MonoAlignmentResult` exports means and scalar gate diagnostics. At
`metric_commit`, the actual 15x15 state covariance contains fixed diagonal
blocks and zero cross blocks. Before correction 340, point covariances are
uninitialized. `InitializeMetric` receives no joint state/map information;
`transformMetricMap` transports mean geometry and clears the point Schur caches.
The estimated shared similarity depends on scale, gravity alignment and the
first-pose gauge choice; its induced uncertainty and correlations are not
propagated.

A 6x6 conditional pose Hessian **does** exist through
`PoseOptimizer::getHessian()`. It treats landmarks as fixed and uses a different
pose tangent and adaptive robust weights. Its inverse does not supply the joint
bootstrap posterior. The point `V/W` likelihood blocks and scalar alignment
residual are also insufficient substitutes. Full raw factors and a justified
noise/prior model could support a new statistical extraction; current exports
do not identify it uniquely.

`D/joint_prior_witness.py` and `joint-prior-witness.json` demonstrate that last
limitation on one captured point. The actual pre-update point covariance is
null. Even granting the code's explicitly unqualified `0.01I` placeholder,
three PSD joint priors with identical means and identical state/point marginal
covariances yield different corrections when their cross covariance differs.
This is an identifiability witness, not a proposed correlation/noise setting,
proof that independence is invalid, or proof of the cause of tracking loss.
Both numerical checks were independently rerun with identical JSON results.

| Bounded contract boundary | Result |
|---|---|
| Public-API input reproduction / frozen source identity | PASS |
| Reconstruct actual point equation / covariance | PASS |
| Camera and point steps implement the tested common joint objective | FAIL |
| Derive a unique justified bootstrap joint prior from current exports | BLOCKED: missing producer statistical contract |
| Safe estimator repair / full monocular VIO | NOT_ESTABLISHED / FAIL unchanged |

Stop local numerical patching under the present interface. Further Schur mono
work requires an explicit bootstrap/marginalization architecture decision: a
consistent joint prior or equivalent factorized information, receiver tangent
and gauge policy, uncertainty of retained historical poses, and measurement
identities showing which image/IMU factors are already represented. A proved
conservative treatment of correlations may also satisfy that contract. The
missing information is not uniquely recoverable by renaming scale, adjusting
one covariance constant or copying an existing conditional Hessian.
This finding does not establish a unique shared root cause for both sequences.
The current mono runtime remains unqualified; no replacement laptop package is
issued. Prior seven-native/587-software regression results remain preserved;
they were not rerun because estimator and build sources did not change.

## Measured PC performance

The final checked C++ runtime completed full V1_01 stereo 1× with 2911 poses,
no loss, ATE RMSE **0.041158 m** and 1-second RPE **0.042977 m**. GNU time reports
**80,980 KiB Peak RSS**, 24.85 s user + 0.50 s system CPU over 145.63 s
(0.174 average cores). Frontend P50/P95 is **2.024 / 2.652 ms**, Schur forward
**0.098 / 0.157 ms**, backward **0.530 / 0.647 ms**. Numeric output is 20 Hz;
replay backlog P95 zero, max three. Evidence is
`R/benchmark/checked-v101-stereo-full-1x/`. This verifies the I/O-only fix against
the preserved numerical baseline; the observed accuracy change is within run
variation and is not a demonstrated algorithm improvement.

The initial full V1_01 stereo 1×, fixed-scale SE(3) ATE RMSE is **0.044436 m**.
Its 1-second translational RPE RMSE is **0.042905 m**.
The directly measured native process used **79,676 KiB Peak RSS**, 25.62 s user
CPU and 0.53 s system CPU over 145.63 s elapsed (0.180 average CPU cores).
Native frontend P50/P95 was **2.106 / 2.733 ms**, Schur forward
**0.099 / 0.138 ms**, and backward **0.572 / 0.783 ms**.
Numeric tracking-stage output was approximately **20 Hz**.

Scheduled replay backlog P95 was zero and maximum three frames at startup.
This measures overdue source timestamps in a synchronous replay, not a physical
camera queue. Effective *qualified valid* output frequency, accepted visual
update ratio and actual input queue depth are NOT_RUN. V1_01 orientation truth
has known limitations; retain that caveat with rotational errors.

The predeclared five complete stereo 0.5× repeats all passed numerical
continuity and the stricter frame/pose/schema recheck. Every run contains 2912
camera rows, 2911 numeric poses and no observed tracking exit; the initial
uncovered image is retained as an explicit rejection. ATE RMSE mean ± sample
standard deviation is **0.043560 ± 0.003341 m**; 1-second translational RPE is
**0.042862 ± 0.000189 m** (n=5). Phase-time P95 ranges across these runs are
2.237–3.096 ms frontend, 0.108–0.166 ms forward, 0.617–0.806 ms backward;
GNU Peak RSS ranges 79,344–80,940 KiB. Numeric output is approximately 10 Hz in
wall time at half-rate replay, while original sensor stamps remain 20 Hz.

These are native x86 PC measurements, including decoding, trace/log I/O and
concurrent workstation activity. The 1× accuracy lies within this repeat spread;
these runs do not demonstrate an accuracy/performance tradeoff or a speedup over
ROS. Reducing features/resolution or changing filter parameters has not been
measured, so no such optimization is recommended from these numbers. Stock
transitive OpenCV GUI libraries and upstream frame logging are remaining size/
I/O costs, not evidence of a current target bottleneck. Repeats cover about
24 minutes of wall time in separate epochs; this is not continuous 60-minute
stability.

`R/benchmark/reviewed-repeat-accuracy-performance.json` includes every declared
member, resource/timing observations, mean/std and the original run hashes.
Initial evaluations remain immutable; stricter results are separate
`reviewed-evaluation.json` files. Raw per-run evidence is under `R/benchmark/headless-v101-stereo-full-1x/`:
`benchmark.json` for execution/resources/provenance, `summary.json` for phase
timings and backlog, `evaluation.json` for ATE/RPE, and `native/` for raw states,
events and exact source-to-completion association. The benchmark rejects
measurement of a Docker/SSH client as if it were the estimator.

The checked ARM64 bundle also executed the full unpaced V1_01 sequence under
explicit QEMU: stereo ATE **0.043261 m**, RPE 1 s **0.042883 m**, 2911 poses and
zero observed loss. The same full ARM64 mono run produced one pose and lost
tracking, preserving FAIL. These are emulated architecture/math checks;
CPU/RAM/timing under QEMU is not M3C performance. The checked PC runtime completed
full unpaced V1_03 stereo with 2149 poses, no loss, ATE **0.080527 m** and RPE
1 s **0.034950 m**. This single different-sequence run is a diagnostic; it does
not replace a five-repeat V1_03 acceptance cohort. Checked PC mono still failed.

## Software verification

The complete local suite most recently passed **587 tests**, using the existing compatible
OpenCV 5 environment and sourced ROS2, with the checked C++ binary supplied
through `SCHURVINS_HEADLESS_TEST_COMMAND`; no C++ boundary tests were skipped.
This includes all seven compiled boundary tests. The repair rerun is at
`R/mono-repair/host-regression-junit.xml`, with zero failures/errors/skips.
The five opt-in production-core CTest cases also pass; their actual build and
execution log is `R/mono-repair/runtime/native-tests-build.log`.
The previous rerun accompanies the second return at
`R/incoming/reviewed-sessions/m3c-20261102T080438Z/host-regression-junit.xml`.
The earlier 583-test result predates four private-resource-tool checks.
An earlier system-Python run had 520 passes and one pre-existing ChArUco
API failure; its evidence remains in `R/baseline-software.json`. No test was
removed or weakened. Original baseline results remain in `R/final-software-junit.xml` and
`R/headless/final-cpp-boundary-junit.xml`.

The evaluator now requires the native summary schema and typed counts,
parses/compares every frame and emitted pose, verifies source-prefix association,
and preserves integrity failures independently of a numerical score. Real
stereo runs were rescored into separate reviewed results; failed mono stays FAIL.
`R/software-review.json` records the original audit findings. Source protection
is recorded in `R/source-protection-final.json`: original HEAD/branch are
unchanged; other-task concurrent dirty-file changes are observed, not reverted.
The parent drone repository report is deliberately untouched.
The second return's read-only source-protection check is retained alongside its
derived results as `source-protection.json`: original HEAD/branch still match
the import baseline, while concurrent work on protected files is recorded and
left untouched.

## OS04A10, IMU and PX4

Historical SDK calls are `AX_VIN_GetYuvFrame` / `AX_VIN_GetRawFrame`.
The opaque `u64PTS` unit, clock, exposure event, row timing and camera–IMU clock
mapping remain UNKNOWN. No SDK source or raw camera recording was found in the
inspected current/archive locations. Existing 1344×760 full180 calibration has
geometric checks only; five distortion coefficients must not be silently
truncated to four. A live frontend/PTS result cannot be claimed from EuRoC.

`experiments.schurvins_headless` now projects actual recorded `poses.csv` /
`frames.csv` / `summary.json` against the sensor-only camera ledger into the
existing `OnlinePoseRecord` envelope. It checks exact integer associations,
frame/pose counts, order, stage/emission parity and unit quaternions. Native
steady callback completion is retained without fabricating a publication or
receipt time. Tracking/initialization remain UNKNOWN, validity remains false,
and mono scale remains UNKNOWN. Stereo preserves the native calibrated meter
convention without claiming operational validity. The real full stereo (2911
records) and failed mono (one record) were both projected and round-trip tested;
see `R/interface/headless-{stereo,mono}/`. This incomplete observation envelope
is not an eligible MAVLink state estimate.

Existing offline sensor, `LocalizationEstimate`, state-output and MAVLink tests
were executed. A real production CLI sent one synthetic ODOMETRY over localhost
and suppressed stale state; this is transport evidence only. Old PX4 IMU logs
remain incomplete/unqualified. No camera capture, UART, firmware or EKF change
was performed. New capture requires user authorization. Physical UART requires
props removed, Disarmed, exclusive ownership and current configuration checks;
retain 115200/8N1. See `R/interface/` and the existing data contract.

The user identifies the hardware as a custom M3C carrier with **ICM-42688-P**.
Its bus, pins, DRDY, clock, driver and camera synchronization are unverified;
MaixCAM2 DT identity does not identify the custom carrier's IMU or its address.
No board IMU device was opened. Standard MaixCAM2 IMU documentation must not be
used as the wiring specification for this carrier.

## First laptop return: deployment without estimator launch

Session `m3c-20261101T164018Z` is preserved under
`R/incoming/received-sessions/`; return archive SHA256 is
`248b991848fd0a74f41316a8db2b9e2315ddda4e74cd7848b3ccc011fd37af4c`.
Host verification confirmed the checksum and 103 regular-file/directory tar
members, without unsafe paths or macOS sidecars. Earlier failed SSH attempts and
the missing-tool failure remain alongside the successful connected audit.

Observed target: DT `MaixCAM2` / `axera,ax620e`, Linux 4.19.125 AArch64,
Ubuntu 22.04, glibc 2.35, two processors; MemTotal 457724 KiB and initial
MemAvailable 342716 KiB, no swap. GCC/G++ 11.4 are present; CMake is absent.
Maix/OpenCV library and SDK candidate paths were listed, but required SDK headers
and acquisition interfaces have not been qualified. A pre-existing `launcher`
owned `/dev/ttyS4` and AX devices and was left running. These observations do not
identify the custom carrier's physical SoC marking or IMU wiring.

Loader/help PASS validates only that startup path. The old runbook stopped at
`/usr/bin/time` with exit 127 before any estimator run; all new-board trajectory,
CPU, Peak RSS, phase timing and backlog outcomes in that session remain NOT_RUN. The Mac laptop
ran the original four test files unchanged: 42 PASS / 6 FAIL due to the Linux
resource sampler's platform requirements; these failures are not reclassified.
The next runbook lists portable analysis and Linux native tests separately.

An optional private ARM64 GNU time package is at
`R/handoff/rm27-m3c-time-v1.tar.gz` (17,115 bytes, SHA256
`3054bea40d0227f7d2afebd1e52ce527be762687617c5440fa9d98a19a987a23`).
It reuses the unmodified Bionic reference tool with the existing checked runtime
loader/libraries; no target system installation or libc replacement is needed.
QEMU checks confirmed GNU time identity, resource format and child exit 7
propagation; this is not M3C measurement evidence. The benchmark accepts an
explicit `--time-binary`, validates GNU identity, records tool hashes/version,
detects tool changes and rejects mixed resource tools in repeat cohorts.
The revised runbook permits estimator diagnostics even without a resource tool,
retaining CPU/Peak RSS as BLOCKED/NOT_RUN, and documents the native sampler with
the private tool. A new session must preserve the first session and raw results.

## Second laptop return: actual execution, monocular FAIL

Session `m3c-20261102T080438Z` ran branch commit
`6ecd2d2e539486eff408e0dd6e4d0d98f9c0a67e`. Its 4,744,327-byte archive has SHA256
`8136a69f94235b9afec2230bf5cc52999e3159eb572cd47d25abc794df77987e`.
Host review verified all 144 tar members are safe regular files/directories and
all 130 extracted regular files match the archive, including a post-analysis
immutability check. Original records remain under
`R/incoming/received-sessions/m3c-20261102T080438Z/`; derived host results are
separate at `R/incoming/reviewed-sessions/m3c-20261102T080438Z/`.

The observed target identity agrees with the first session. The same kit was
reused and all 6,226 files were reverified. Checked ARM64 binary SHA256 is
`697c26e36fe2f0325f315b9c8da6bb1848e8ebcb0883a158cbb8d62a6d1bbe37`.
The private GNU time version and `/bin/true` probes now pass on the actual M3C.
No estimator rebuild, calibration/parameter change or system tool installation
was performed. Board wall time is incorrect (July 2026); measurements use one
native monotonic clock, not a presumed laptop/board wall-clock mapping.

| Run | Selected cameras / IMU delivered | Poses / losses | Estimator | GNU elapsed s | GNU CPU s | GNU Peak RSS KiB |
|---|---:|---:|---|---:|---:|---:|
| mono-offline-full, unpaced | 2912 / 29120 | 1 / 1 | FAIL | 29.52 | 26.63 | 114320 |
| mono-half-full, 0.5× | 2912 / 29120 | 1 / 1 | FAIL | 292.63 | 27.00 | 113712 |
| mono-half-sampled, 0.5× | 2912 / 29120 | 1 / 1 | FAIL | 293.29 | 28.10 | 114164 |
| mono-one-full, 1× | 2912 / 29120 | 1 / 1 | FAIL | 147.24 | 27.09 | 114144 |
| mono-smoke, bounded prefix | 200 / 1992 | 1 / 1 | FAIL, partial | 5.03 | 4.80 | 114884 |

All five child processes exit zero. The four full inputs pass the existing
frame/pose/schema/source-association checks, but each fails `TRACKING_LOSS`,
`TRAJECTORY_NOT_EVALUABLE` and `MONO_METRIC_INITIALIZATION_UNVERIFIED`.
The host used the original V1_01 ZIP with SHA256
`a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa`
for post-execution fixed-scale scoring. **ATE/RPE are unevaluable, not zero and
not PASS**. Ground truth was never an estimator input or part of the board kit.
The smoke additionally fails complete-sequence/delivery gates by its deliberate
prefix limit; it is not a fifth full-sequence repeat. Delivery counts alone do
not qualify internal IMU integration. The initial uncovered image remains an
explicit rejection, as on PC.

| Complete run | Frontend P50 / P95 ms | Schur Forward P50 / P95 ms | Replay backlog P95 / maximum frames |
|---|---:|---:|---:|
| unpaced | 17.684 / 19.077 | 0.945 / 1.020 | 0 / 0 |
| 0.5× | 17.712 / 23.175 | 0.931 / 0.995 | 0 / 0 |
| 0.5× with sampler | 19.135 / 23.100 | 1.091 / 1.987 | 0 / 9 |
| 1× | 18.008 / 20.355 | 0.978 / 1.036 | 0 / 1 |

Each run has only **116 frontend spans, six Schur Forward spans and zero
Backward spans**. These cover bootstrap and failure/relocalization. The full
1× average CPU is 0.184 cores (18.4% of one core), and GNU Peak RSS about 111 MiB,
but most of that replay occurs after tracking stops. These are failed-pipeline
resource observations, not proof of valid-VIO capacity, output frequency or a
tradeoff between precision and speed. Backlog counts overdue replay timestamps,
not camera hardware queue depth. Native phase timers and whole-process GNU
resources have different boundaries; absence of a Backward timing is NOT_RUN,
not a zero-cost backend.

The Linux sampler passes its measurement/identity/artifact/cleanup checks:
1,557 samples, sampled simultaneous process-tree RSS maximum 115224 KiB,
observed temperature maximum 57.35 °C and CPU frequency 1200 MHz. Sampled RSS
includes the wrapper and can miss peaks; it is not interchangeable with GNU
high-water RSS. This short failed pipeline establishes no 60-minute thermal or
tracking stability. The Mac portable suite passes 33 tests; the Linux native
sampler suite is explicitly NOT_RUN on macOS. The first session's 42/6 result
remains unchanged.

`receipt-integrity.json` binds the original package, `*-evaluation.json` contains
the trusted host scoring, `*-observations.json` rechecks native timing and the
sampled run's sealed evidence hashes, and `lifecycle-comparison.json` binds the
PC/board failure comparison and source call path. `review.json` collects the
per-run decisions. The generic evaluator's `board_qualification=NOT_RUN` field
is deliberately unchanged: execution-host identity is separate evidence in the
returned audit/argv/exit records, not something trajectory scoring certifies.

No physical OS04A10, carrier ICM-42688-P, UART or flight-controller MAVLink was
used. Real Camera+IMU VIO and operational validity remain NOT_RUN. No new runtime
package is promoted from this review. First repair and validate the PC metric
initialization/visual-update chain; then cross-build, verify artifact identity
and conduct a new, separately retained laptop session.

## Next decision and RK3576 reuse

Further platform performance optimization is premature until monocular metric
initialization and continuous tracking pass. The independent headless build,
ASL boundary, source manifests, fixed-scale evaluation, native phase/resource
benchmark, offline interfaces and deployable reference artifacts can be reused
for RK3576. They do not require the M3C camera SDK. Actual M3C ABI execution now
has evidence; monocular initialization is the immediate software gate. Sensor
timing/calibration and performance during qualified tracking still require the
connected laptop and valid input chain. No RK3576 branch edit is required.
