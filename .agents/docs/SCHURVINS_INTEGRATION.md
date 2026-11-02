# SchurVINS candidate: PC reproduction and RK3576 preparation

SchurVINS is an authorized VIO candidate for RK3576; OpenVINS remains the
comparison baseline. This work does not qualify a camera/IMU assembly, RK3576
runtime, estimator/controller integration or flight. The existing OpenVINS
VIO-P repeatability and VIO-S0 evidence gates remain unchanged. Historical
MicoAir/PX4 losses are retained evidence, not prerequisites for a new independent
IMU chain.

Current pre-hardware verdict is **PARTIAL**. Public-data stereo, OpenVINS
comparison, ARM64 build/emulation and interface evidence have verified scopes
below. Reliable continuous Schur mono tracking, complete required cohorts and
operational health/recovery remain software acceptance gaps. RK3576 BSP/ABI,
physical sensor timing/calibration and onboard performance are separately
HARDWARE BLOCKED / NOT_RUN; this work is not yet at an only-hardware-remains stage.

## WIP publication checkpoint

The user explicitly requested committing and pushing completed and unfinished
repository work together. This is a WIP checkpoint, not a new research acceptance.
Publication verification reran the existing PC test suite: **1310 passed,
1 skipped**. Ignored plans, raw experiment artifacts, datasets and the external
research workspace remain local. Research remains stopped; no new estimator,
dataset replay, hardware or integration experiment was started for publication.

## Accepted-state snapshot recovery preflight: BLOCKED

**USER_STOPPED / NOT_FINALIZED.** The user stopped work during independent
read-only review. The author inventory below is saved, but independent review
and finalization have not been completed/validated by root. Any completion or
receipt references below describe the intended closure, not a completed audit.
No state export, re-execution or follow-up experiment was performed. The stop
point is recorded in `accepted-state-snapshot-recovery-preflight-v1/interrupted.json`.
No work remains queued; resumption requires a new user instruction.

**SNAPSHOT_RECOVERY_REQUIRES_REEXECUTION.** This source-inventory milestone
does not recover/export a complete state. Both flavors' five accepted candidates
exist only as past in-memory results, with partial output records. The required
complete candidate snapshots and historical fingerprint values are absent.
Re-execution/reconstruction, geometry calculations and all follow-up experiments
are **NOT_RUN**. The milestone is **BLOCKED / USER_STOPPED / NOT_FINALIZED**;
overall pre-hardware status remains PARTIAL.

Fresh evidence is in
`.agents/runs/schurvins/state-flight-mock/accepted-state-snapshot-recovery-preflight-v1/`.
`inventory.json`, SHA-256
`cb98186e1533d413be9fda1d281feb9a36410ad9d97f1e1516fc520c23da1254`,
records the complete source/input inventory, both flavor-specific compile flags,
binary/runtime identities, available frame/state/point rosters, ten state
feasibility rows and the unexecuted reproduction proposal. It freshly checks
all622 protected source/input/installation/artifact identities and the previous
closed milestone's receipts. Branch/HEAD and existing dirty work are preserved;
no production, schema, policy, scorer, matrix or threshold is changed.

| Flavor | Accepted state | Complete saved poses | Complete saved IMU states | Saved point positions | Direct complete export |
| --- | --- | ---: | ---: | ---: | --- |
|O1 |Original |0/49 |0/48 |313 |BLOCKED |
|O1 |Sealed seed, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|O1 |Additional1, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|O1 |Additional2, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|O1 |Additional3, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|Exact production flags |Original |0/49 |0/48 |313 |BLOCKED |
|Exact production flags |Sealed seed, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|Exact production flags |Additional1, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|Exact production flags |Additional2, alpha1/16 |0/49 |0/48 |313 |BLOCKED |
|Exact production flags |Additional3, alpha1/16 |0/49 |0/48 |313 |BLOCKED |

For every row,49 frame records are aggregates and47 IMU records are residual
factors, rather than actual pose/state vectors. The existing17-digit decimal
point output is round-trip-capable for binary64, but does not supply missing
state components. No partial point export is presented as a complete snapshot;
strict complete-snapshot precision/quaternion validation is NOT_RUN.

The accepted labels are exactly `nominal`, `known_seed_alpha1over16`,
`outer1_accepted`, `outer2_accepted`, `outer3_accepted`. All four
`outer*_total_min` alternatives are excluded. O1 binary SHA is
`90f2e4b7df8a718bb92f5ee8e87808fd65a55d64c40d691e157a8b1cbe366953`;
production-flags binary SHA is
`c5fab65ed919ebbbbcc684cbce8971e3e9e02c7770a8989dfb8f0fa63dd765c8`.
Their separately sealed stdout hashes are `0bf41c99…91498` and
`9089fe0b…2087`; full hashes are in the inventory. Their component costs are
read back from those exact artifacts, not recalculated or mixed across flags.
Each original runtime manifest binds204 dependencies and the actual compiled
`InitImuModel`, `Prediction`, `PredictionState` implementations.

### Why Original also requires execution

Under the existing external `bootstrap-visual-likelihood/` source root:

| Source | Direct data or computation |
| --- | --- |
|`selected_capture_fixture.h:52–112` |Bit-decoded transforms/points; world-from-IMU pose still requires transform inverses |
|`captured_imu_data.inc:3–9,759–761` |q,ba,bg and terminal velocity are bit-decoded seeds; IMU position is scale×camera-position−q×arm; first47 velocities are placeholders |
|`nonlinear-relinearization-v1/relinearization.cpp:88` |Applies fixed metric-scale bits4607815570575527631 to visual poses/points and computes47 velocities backward using compiled prediction |
|`nonlinear-window-v1/nonlinear_window.cpp:23–37,65–95` |Defines candidate ordering, in-memory fingerprint and whole-state/point update; no complete candidate writer |
|`bootstrap_local_system.h:113–154` |Maps48 IMU identities into49 ordered poses using full sensor_ns/bundle/frame/camera tuples |
|`imu_residual.h:30–41` |WORLD-left normalized Cayley update; additive p/v/ba/bg, tangent order theta,p,v,ba,bg |

This source arithmetic makes a capture-only export insufficient even for
Original. No capture field is silently substituted for the computed nominal.
The stored quaternion direction is world-from-IMU with Eigen coefficient order
x,y,z,w; position/velocity use the selected local world and biases use IMU axes.
This does not qualify a physical NED world or metric bootstrap. Native IMU
double-time bits remain distinct from supplied integer sensor identities;
unknown timestamps are not synthesized.

The original candidate fingerprint hashes scalar bits in vector/map order but
does not encode identities/counts. Its value was used only in memory and was
never emitted; equality booleans are not a saved fingerprint. Therefore any
future new snapshot needs its own identity-bound canonical SHA and freshly
computed fingerprint. **A newly reconstructed snapshot cannot be claimed to
match the full historical candidate bit-for-bit**, because the historical
fingerprint and complete vector baseline do not exist.

### Minimal deterministic reproduction proposal — not executed

If separately authorized, prepare a fresh external export-only diagnostic child
with the original input lock, model, calibration/noise, fixed initial-Q/gauge,
source identities, image `sha256:b4a9eb29…c34a`, GCC7.5.0/CMake3.10.2 and each
flavor's exact compiler flags/dependency bindings. The complete image/flags/input
hashes are in the inventory. Production's `-march=native` also requires an
explicit same-host ISA record; compiler flags alone do not prove determinism
on another CPU. Never directly invoke the old runner: it executes extra grids,
trial alternatives and validation updates outside this minimal proposal.

1. Recreate nominal using the existing transform/scale/backward-velocity order.
2. Export nominal; reuse the exact sealed parent `Objective::direction` for the
   seed, followed by three existing current-state relinearizations. Apply only
   the four previously selected alpha1/16 updates; no grid, line search, extra
   accepted state, total-min branch or unused final direction.
3. Save five states per flavor, each49×7 pose scalars +48×16 IMU scalars
   +313×3 point scalars =2050 doubles. Use authoritative IEEE-754 uint64 hex
   strings, explicit field/byte order, full ordered identities, source/build
   hashes and canonical SHA. Do not renormalize quaternions, flip their signs
   or replace negative zero during export.
4. Check finite scalars, quaternion validity, pose/state correspondence and
   bit-roundtrip. Compare the same-flavor retained costs/points/factor evidence,
   fixed-Q hashes and memberships. Any mismatch fails closed. An independent
   reader reviews outputs; no new geometric research or runtime handoff follows.

Proposed limits:2CPU/4GiB/pids128/no network; configure60s/build300s,
execute180s per flavor;16MiB snapshot/receipt output,4GiB total new build/data,
at least20GiB free. Label artifacts `REPRODUCED_ACCEPTED_CHAIN`, not recovered
historical snapshots. Even exact agreement with retained costs/residuals/points
cannot replace the absent full historical fingerprint proof. None of this
reproduction, export or numerical verification is authorized/executed here.

The inventory's independent read-only review and final process/container cleanup
receipts were planned as `independent-review.json` and `finalization.json`, but
their completion has not been validated by root. The saved `interrupted.json`
records the actual stop point and inventory-reader exit. No new container or
background worker was created. Sealed artifacts remain unchanged. This milestone
remains stopped, awaiting explicit review/authorization.

## Fixed-window visual geometry consistency: preflight BLOCKED

The new geometry/weak-landmark milestone stops at its mandatory read-only
preflight: **BLOCKED, not diagnostic PASS**. Both compile flavors lack the
required previously saved complete state snapshots. No geometry recomputation,
solver, state reconstruction, alpha grid, counterfactual or compiled diagnostic
execution was performed. The previous numerical qualification remains PASS
within its original scope; it did not establish that complete candidate states
were serialized.

The new local evidence directory is
`.agents/runs/schurvins/state-flight-mock/visual-geometry-consistency-v1/`.
Its `preflight.json`, SHA-256
`dc85ed2b52286ce05d8ac6e8b850222a81c9007ae46b46e62bf677aae32231dc`,
checks622 exact source/input/installation/matrix/artifact identities. The prior
`bootstrap-relinearization-causal-v1/finalization.json` remains exactly
`425f3f2e21c134504212ea9a00c4abd7b56fafc42cc11336240a3f67a040cd77`;
all11 referenced receipts match. Source/model availability and identity pass.
The window remains49 poses,48 IMU states,313 point IDs,6897 selected visual
factors,47 IMU transitions,484 raw memberships and626 excluded identities.
The75-gate/67-required-cohort matrix and1cm repeatability gate are unchanged.

For **each** of nominal, alpha1/16 seed and Additional1/2/3 in **both** flavors:

| Required evidence | Actual sealed content | Result |
| --- | --- | --- |
|313 point coordinates |313 `point_attribution.position_m` rows | PRESENT |
|6897 selected factors, costs and identities |6897 visual factor rows,49 frame aggregates,47 IMU factor rows | PRESENT |
|49 exact pose quaternion/translation vectors |Frame rows contain identities and statistics, no pose vectors | MISSING |
|48 exact IMU q/p/v/ba/bg vectors |IMU rows contain15-component residuals/whitened residuals, no state vectors | MISSING |
|Complete candidate scalar-bit serialization and per-state hash |In-memory fingerprint equality booleans only | MISSING |

The external writer
`bootstrap-visual-likelihood/nonlinear-relinearization-v1/relinearization.cpp`
has SHA-256
`c462c882de21b5f2a4dfbddd26c2dfdbd87c7dc71bdd1f4b5eb0184087cf2133`.
Its output definitions and complete external file inventory confirm that there
is no separate snapshot export. Both independent re-execution output streams
have the same limited schema. Two independent read-only reviewers reach the
same missing-snapshot conclusion. Cost/residual records, point positions and
displacement norms cannot substitute for the absent pose/IMU vectors.

The final independent read-only receipt `independent-preflight-review.json`
has SHA-256 `541766fcf969ebb90b59781e5b8c010a2c074345f813b657270f87b1b2ce4f4c`.
It actually rehashes all622 identities/853845415B, the prior finalization and
11 references, and parses both complete previous output streams. Its audit is
PASS; the geometry milestone remains BLOCKED. It runs no compiled model and
does not reconstruct missing vectors.

The five historical exact component costs are preserved in the prior section
below and copied as sealed readback, without a new evaluation, into the new
preflight receipt. All requested new full6897-factor projection ledgers,
313-point parallax/Jacobian/conditioning reports, point310/446/477 calculations,
point477 raw-pixel/1D reconstruction, production3px/4px identity comparisons,
three counterfactuals and geometry positive/negative tests are **NOT_RUN**.
No new native-inlier equivalence or cross-run causal comparison is claimed.
The118→119,121→122,117→118 IMU findings remain historical attribution only;
there is no new residual/Q decomposition or noise change.

The smallest next decision is a **separately authorized export-only reproduction
of the existing accepted chain**, preserving exact sources/flags/fixed-Q and
old component costs while serializing full state scalar bits, identities and
hashes. This requires re-executing prior updates; it is outside this milestone's
no-update/no-reconstruction boundary and has not been run or queued. There is
no production mean/point/prior/linearization/epoch/handoff change.

Schur continuous mono V101/V202/V103 remains FAIL. Bootstrap joint
covariance/correlation and6897-observation/47-transition measurement ownership
remain UNQUALIFIED; atomic handoff remains NOT_RUN. OpenVINS tracking remains
UNKNOWN, and PX4 EKF/RK3576 hardware remain NOT_RUN. Overall status is PARTIAL.
The independently requested shared-video checks below are a separate scope and
do not resolve this missing-snapshot blocker.

### Shared real-camera clips: independent offline decode/quality regression

The user supplied the existing `data/shared_vision/` entry. Its README and
`real_camera_exposure/manifest.json` were read before use; all four MP4 size and
SHA-256 identities passed **before any decoding** and again after execution.
Original bytes/catalogs were preserved. The manifest hash is
`00ba541032febbfd203396c043d5e20d8aced6a5e6e49811a95857d6c1792208`.
This independent additional scope is **PARTIAL**: full decode/integrity passes,
while the existing encoded-PTS acceptance gate fails on all four clips.

The unchanged local `rm27/perception/vision/localization/analyze_video.py`
(SHA-256 `df02adf1865d30979be12eaa43b5036efd9229933676462a2d605dd2bdd0770e`)
was reused; no estimator or SLAM backend was started. Its existing tests ran
with **4 passed / exit0**. This is the only new test execution; the previous
1310 passed/1 skipped full-suite result remains historical. Decode reaches EOF
and processes gray/hash for every frame; ORB/blur/luma/grid and LK diagnostics
are sparsely sampled every1800 frames. LK across this sparse sampling is not a
native VIO tracking test, physical-motion measurement or frontend acceptance.

| Catalog clip ID | Decoded/declared frames | Quality samples | Decode/integrity | Original encoded-PTS gate | First BGR hash equals catalog |
| --- | ---: | ---: | --- | --- | --- |
|clip01 |5505/5505 |4 |PASS |FAIL |No |
|clip02 |17987/17987 |10 |PASS |FAIL |Yes |
|clip03 |10079/10079 |6 |PASS |FAIL |Yes |
|clip04 |11891/11891 |7 |PASS |FAIL |Yes |

The fixed denominator is45462 decoded frames,27 quality samples total. The
environment is Python3.12.3/OpenCV4.6.0/NumPy1.26.4, compared with catalog
OpenCV5.0.0; a first-frame equality is not whole-decoder equivalence. Every
clip has14 nonpositive OpenCV-reported encoded-PTS intervals and ends at zero.
The original timing thresholds remain unchanged; all four analyzer commands
exit1 and their complete failure results are preserved. No timestamp is filled
from frame index or nominal180FPS. These are observations of the current
OpenCV presentation-time interface, not proof of physical capture gaps or of
the underlying container's independently demuxed PTS. Clip01's pixel difference
does not imply changed original bytes: the MP4 SHA is unchanged.

Fresh derived outputs are under ignored
`artifacts/shared-camera-exposure-vio-v1/`, including each clip's provenance,
timing.csv, quality metrics, sparse JPEGs and completion status. `receipt.json`
SHA-256 is `08f4ef360982070fa1cfefd0a3bcabb18d1dc4bfd90748f4c6406dee5ec10ab2`;
it binds130 output artifacts, all inputs, source version/HEAD/configuration,
original exits and the existing-test log. It reports PARTIAL; coordinator exit0
only records successful evidence collection. The only added wrapper is in the
ignored artifacts directory, SHA-256
`c0945735158d412f7cdfc5291d8f2b6c3ba993926981e7c529052d93327fee1f`.

Limits are one CPU affinity,2GiB address space,180s per clip,32MiB total derived
output and20GiB free-space reserve. All clips complete without timeout/cap
termination; worker elapsed20.762/91.755/55.287/66.037s, maximum RSS295034880B,
total derived12184781B. OpenCV processing threads are1; the actual decoder's
thread count is not directly exposed by this binding. CPU affinity bounds
execution despite any internal threads. All five owned process groups have
fresh absence checks; no new container or estimator is created.

An individual analyzer invocation, as actually run inside that bounded wrapper,
is reproduced below. A new output path is mandatory; existing paths refuse
overwrite. This documents the invocation rather than queuing another run.

```bash
/usr/bin/python3 -m rm27.perception.vision.localization.analyze_video \
  data/shared_vision/real_camera_exposure/raw/file_v3_00172_037e73f1-eea4-4d17-8111-eadd29e25eag_b19d514a-c6c6-4dc4-9a6b-facf6c517789.mp4 \
  --out artifacts/shared-camera-exposure-review-new --sample-step 1800
```

The exposure multiset1500/2500/2500/3000μs, LCG1×, is batch-only user
information. Per-clip exposure stays null; brightness is not used to assign it.
There is no synchronized IMU, matching calibration, physical exposure timestamp,
sensor identity or ground truth. **VIO and hardware acceptance are NOT_RUN**;
these clips cannot qualify positioning, scale, physical timing or RK3576 speed.
Production sources and the dirty branch/HEAD remain unchanged by this check.

Independent read-only review is recorded separately in
`artifacts/shared-camera-exposure-vio-v1-independent-review.json`; it verifies
the130 output artifacts, nine source inputs and complete timing CSVs without
another decode. Each clip's14 nonpositive intervals comprise one negative
reset plus13 zero differences in the final14 frames. The27 sparse quality
samples produce23 intersample LK pairs, not adjacent-frame visual updates.

The Schur part is BLOCKED and stopped; the independent video part is complete
with preserved FAIL results. Overall remains PARTIAL. Final evidence/identity
and independent review references are sealed in this milestone's
`visual-geometry-consistency-v1/finalization.json`; no next run is scheduled.

## Bounded Schur relinearization and OpenVINS causal readback (2026-11-02)

This milestone is an **external PC offline diagnostic**, with no production
estimator, policy, schema, scorer, matrix or sealed-run changes. Schur numerical
qualification and its independent actual recomputation pass. OpenVINS causal
readback and its independent complete-row review pass. This milestone is
**CLOSED / PARTIAL / STOPPED for user review**. The overall
pre-hardware verdict remains **PARTIAL**. Neither local descent nor freshness
recovery qualifies metric bootstrap, continuous tracking or flight output.

Evidence directory `N` below is
`.agents/runs/schurvins/state-flight-mock/bootstrap-relinearization-causal-v1/`.
External diagnostic sources are in
`/home/shiuhou/Projects/rm27-schurvins-research/bootstrap-visual-likelihood/nonlinear-relinearization-v1/`.
`N/preflight.json` and `N/compiled-workspace-preflight.json` bind the existing
dirty worktree, branch `docs/progress-handoff-rule`, HEAD `a8392c6f…`, protected
sources, installation, three57-output/19-runtime-source OpenVINS manifests,
original compiled Schur models and frozen inputs. All identities matched before
execution. The existing75 gates/67 required cohorts and1cm repeatability gate
are unchanged. Container `/ws/…` library symlinks were resolved against their
frozen host workspace, not replaced by another build.

### Schur: bounded current-state relinearization

The diagnostic reuses the sealed nonlinear objective, actual compiled
`InitImuModel`/`Prediction`/`PredictionState`, camera projection and coordinate
charts. The49 poses,48 IMU states,313 points,6897 selected visual factors,
47 IMU transitions/484 raw memberships, calibration/noise/scale, gauge slice and
18 original alpha values stay fixed. At every relinearization the residual and
Jacobian are recomputed at the current accepted state, but the47 **initial-Q**
whiteners stay bit exact. This is the original fixed-initial-Q surrogate, without
a state-dependent likelihood/log-determinant or qualified bootstrap posterior.
Visual residuals use the existing2-software-pixel scale, original edgelet
projection and unit robust weights; they are not production tangent/Huber gates.

The original nominal18-grid and alpha1/16 seed reproduce all sealed component
and total costs exactly, separately for O1 and exact production flags. Only
the conservative diagnostic chain with both components nonincreasing is
continued. Each flavor has exactly **three additional accepts**, all alpha1/16.
The total-minimum trial is reported separately at each outer state; it is not a
second accepted chain. World-left unit-Cayley rotation updates and one common
alpha apply to the entire state/point direction. Every trial uses an independent
copy; rejected/invalid trials leave accepted state fingerprints unchanged.

| O1 state | Alpha | Visual nonlinear cost | IMU nonlinear cost | Total nonlinear cost |
| --- | --- | ---: | ---: | ---: |
| Original | 0 | 920.951972 | 9297251.595455 | 9298172.547427 |
| Original sealed seed | 1/16 | 857.255202 | 8171588.540386 | 8172445.795588 |
| Additional1 | 1/16 | 818.356006 | 7182206.188531 | 7183024.544537 |
| Additional2 | 1/16 | 793.083828 | 6312606.733782 | 6313399.817610 |
| Additional3 | 1/16 | 774.439914 | 5548289.474612 | 5549063.914526 |

| Exact production-flags state | Visual | IMU | Total |
| --- | ---: | ---: | ---: |
| Original | 920.951972 | 9297613.732376 | 9298534.684348 |
| Original sealed seed | 857.255167 | 8171906.841224 | 8172764.096391 |
| Additional1 | 818.356350 | 7182485.959800 | 7183304.316151 |
| Additional2 | 793.084732 | 6312852.637432 | 6313645.722165 |
| Additional3 | 774.441409 | 5548505.610971 | 5549280.052380 |

The following O1 step maxima are changes relative to the immediately preceding
accepted state, not physical-error or safety limits. Full max/median/P95,
per-state and cumulative populations for both flavors are retained in the
strict JSONL records and `N/A/qualification.json`.

| Accepted step | Translation m | Rotation rad | Velocity m/s | Gyro bias rad/s | Accel bias m/s² | Landmark m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Original seed | .008973 | .009520 | .007797 | .000573 | .089628 | .805696 |
| Additional1 | .008443 | .008947 | .007282 | .000535 | .084241 | .840236 |
| Additional2 | .007936 | .008388 | .006797 | .000500 | .078973 | .874674 |
| Additional3 | .007452 | .007849 | .006342 | .000467 | .073890 | .908963 |

| Final cumulative O1 change | Maximum | Median | P95 nearest rank |
| --- | ---: | ---: | ---: |
| Translation m,49 poses | .032802 | .012313 | .031008 |
| Rotation rad,49 poses | .034702 | .033799 | .034193 |
| Velocity m/s,48 states | .028217 | .020278 | .027790 |
| Gyro bias rad/s,48 states | .002074668 | .002074561 | .002074665 |
| Accel bias m/s²,48 states | .326701 | .326390 | .326701 |
| Landmark m,313 points | **3.429521** | .171829 | .273656 |

All five current systems, including qualification of the final accepted state,
have the original722/722 gauge-slice rank from726 physical dimensions. The
column-normalized condition number grows approximately
`4.300e7 → 4.598e7 → 4.889e7 → 5.169e7 → 5.440e7`.
The exact313-point rank map is unchanged: points446 and477 each retain one
unobserved direction; maximum step motion in these null directions is below
`4.1e-17`. No point prior or covariance floor is introduced. Final-state
qualification recomputes rank and component finite differences only: there is
no fourth extra alpha grid or accept. Each of the five systems has four
component-direction checks; at h=1e-5 relative derivative errors are below
`1.8e-9`. This supports local derivative/chart consistency, not physical
posterior observability or global convergence.

Each flavor has nine complete attribution states: original, sealed seed,
three additional accepted states and four separate total-min alternatives.
Every state retains all6897 factor rows,313 point/49 frame aggregates,
47 full15-component IMU residuals and484 exact memberships. All selected depths
are positive; minimum O1 depth rises .587530→.644086m along the followed chain.
The626 excluded identities remain an explicit ledger with unresolved point/
promotion mapping; they are not a verified independent holdout or new factors.

Attribution identifies limits that an aggregate descent test misses:

- IMU transitions118→119,121→122 and117→118 contribute about32.025% of
  initial and final IMU cost. Their costs decrease together; the objective
  remains IMU-dominated. No single-transition root cause is established.
- Nominal full-step linear prediction is approximately476.58 total, whereas
  its actual nonlinear total is385476.03 and visual cost grows
  920.95→378666.14. The largest full-step visual costs occur across ordinary
  points167/209/352 and early frames110/0/111; this is not confined to the two
  rank2 points. The total-min alternative remains alpha1 at later outer states;
  the last such independent trial has visual181906.57/IMU3160.77. It is not
  followed. A locally verified derivative does not validate a finite full step.
- The followed chain reduces both aggregate components, yet the external
  **fx/unscaled tangent comparison** population≤4 decreases6879→6865 and the
  point-mean≤3 comparison population308→306. These are explicitly external
  proxies, with native host/local-status/window/gate equivalence unestablished;
  they are not counts of accepted native inliers.
- Largest cumulative displacement is point310, a rank3 point with just two
  observations:3.429521m and objective cost.197445→.402637. The two rank2
  points are therefore not the only geometry concern. Point477's two selected
  edgelet residuals provide another witness: its projected objective cost
  .124312→.071031 while mean raw2D reprojection error1.039786→5.251643px.
  The original1D projection and raw2D comparison measure different quantities;
  cost descent alone cannot establish improved full image geometry.

Both builds use GCC7.5/CMake3.10.2 and their recorded original flags. Each
invocation is bounded at2CPU/4GiB, no network, configure60s/build300s/actual180s.
Each flavor passes nine new current-model rejection/immutability controls,
plus the inherited six synthetic checks/eight rejection controls. Cases include
rank-population collapse, positive-diagonal but singularQ, corrupted frozen
whitening, point-null motion, nonfinite/missing/mismatched input and bit-exact
zero motion. The first attempt and its review findings are retained: final v2
restores exact parent arithmetic for the original grid/seed, adds final-state
qualification and exact point-rank checks, and relabels external gate proxies.
No production source or model is changed by these diagnostic corrections.

Independent read-only actual execution once per flavor reproduces every
numerical record,72 trials and nine complete attribution states, including
204 exact dependency identities/bindings. `N/independent/A-independent-review.json`
SHA `66e02b573169e0e832c61487e68d012d28f309bff39f5d7477a9e9b623da8e09`
and `A-prior-seed-readback.json` SHA
`d164a2014450930762445f8fe3033a23b286a19be124e7c53e2603d56e7ce1bf`
pass. Author qualification SHA
`d82582a23c7f57a256da640f7876077a98a7b31b9ab929094aada4e59605b733`.
A results including superseded evidence occupy77147092bytes, below96MiB.
Independent source-factor witnesses in `A-selected-factor-witnesses.json` SHA
`a1185f58f22cf129dfa70a4eb2dd17fb3fd1aae35b73f8fda9266b1701c5fa6a`
confirm the point310/477 distinctions above without another compiled run.

### OpenVINS: complete causal readback, no tracking classifier

Only the three sealed native-quality runs are read. There is no new estimator
execution, GT access/rescore, threshold selection or shared policy change.
The reader emits all160922 original health rows (normal56905/gap56700/bias47317)
and8390 camera snapshots with exact source hashes, original int64 stamps/time
bits, process session, camera transaction and native steady-clock availability.
Gate records preserve actual threshold DOF separately from projected DOF/NIS.
Finite covariance is a numerical feature, not statistical qualification.

Each row uses only available current/past evidence. An unavailable initial
snapshot remains null; held snapshot ages and source/clock identities remain
explicit. A held camera mean is not substituted for a current IMU prediction.
Quality and native event sequence numbers are separate domains; equal-clock
ambiguity fails closed rather than inventing a cross-log order. There are zero
such ties in these actual three logs. Actual accepted visual corrections end
prediction-only intervals even if their support time is old; watchdog ticks do
not start a prediction or refresh sensor/state/last-visual time. The v1 diagnostic
duration defect and v2 lack of tie proof are retained as superseded attempts;
final v3 corrects them without changing old health output.

Fault windows are used only after feature readback for offline source-interval
comparison. The counts below are individual native gates, not camera updates,
independent statistical tests or a classifier. Each run's steady clock and
sequence domain remain separate; matching sequence numbers across runs do not
establish simultaneity. Source elapsed time is not processing delay or a
sensor→PX4 clock mapping.

| Source interval | Normal accept/reject |500ms gap accept/reject | Bias accept/reject |
| --- | ---: | ---: | ---: |
|19–20s |1050/7 |1050/7 |1050/7 |
|20–21s |1105/10 |412/18 |89/302 |
|After30s |124523/2198 |124235/2286 |2/79 |

The first bias post-onset gate rejection is available at source20.050000128s;
at20.2s the first complete nonempty camera transaction with every gate rejected
has8/8 rejects,
with no accepted visual correction in that camera transaction. Normal has an
individual rejection at20.2s; the gap's first resumed frame at20.5s has11/41
rejections. Thus a single rejected gate has concrete normal/recovery
counterexamples. Bias snapshots continue to pass numerical covariance checks,
with little gyro-bias-norm change, while estimated velocity grows; these are
descriptive witnesses, not a calibrated warning threshold.

Gap native event24115 is STALE with last-visual source1403715293212143104ns;
event24420 accepts a new correction and restores CURRENT with last-visual
1403715293762142976ns. Bias event24609→24630 similarly restores freshness
only. Its later event42898 is measurement-freshness stale and42988 also exceeds
event freshness; no later accepted correction/recovery is established. Final
visual-expiry events are192085(normal)/191729(gap)/42988(bias), distinct from
earlier stopped-input ticks. Full timelines retain these reasons and held
features rather than collapsing them into a healthy/unhealthy label.

The final B reader passes16 positive/negative tests and an actual no-overwrite
rejection. Per invocation:1CPU/3GiB/180s; actual complete readback9–13s,
peak941510656bytes; contrast6s. At the author seal, all B evidence including
preserved attempts occupies55857424bytes, below64MiB. Final cleanup removes
only37810bytes of unsealed owned Python bytecode; sealed artifacts are unchanged.
`N/B/final-receipt.json` SHA
`2ca9b02a82ead5b6042b16b93adf9eea6c7a7bb35bea1f225f728d10eac0669e`
and `causal-contrast.json` SHA
`a6c117bb90f2fc7890e033154506e07a42c230ffe19f16662fb4ce8eae6a403d`
record complete coverage, commands, source/artifact hashes and rejected attempts.
Independent read-only reruns and full-row comparisons pass:
`N/independent/B-independent-review.json` SHA
`bf2c91691ccf5248c61acdec2b8f234ca9fe12ad7e3c228627cc1b1f4e9455a4`.
All160922 rows/8390 snapshots and native NIS/projected-DOF ledger entries
(152915 normal/152033 gap/17678 bias) match without omissions. The review's
first output-byte counter omitted nested ledger files; its original receipt is
retained, with exact physical-budget correction in
`B-output-budget-and-NIS-correction.json` SHA
`ffcba991d72a12f3d3d0088ea5fb8c9f0c5949d26df6db4c5ff5388a16e5ec35`.
Independent B material11537638bytes and aggregate independent material24050749
bytes remain within64/96MiB. This correction does not change numerical results.

### Reproduction and remaining qualification boundary

New code is restricted to the external Schur diagnostic directory and
`N/B/{causal_readback,contrast_readback,test_causal_readback,seal_readback}.py`.
Receipts contain exact source hashes/diffs, compiler flags, tool versions,
compressed/decompressed artifact hashes, commands and bounds. Use fresh output
identities: existing outputs are intentionally rejected, never overwritten.
The A driver has a fixed named output; rerunning its sealed name is not an
instruction to replace the current evidence.

```text
python3 /home/shiuhou/Projects/rm27-schurvins-research/bootstrap-visual-likelihood/nonlinear-relinearization-v1/run_revision.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c 0 python3 <N>/B/causal_readback.py --profile <normal|camera-gap500ms|imu-bias-step> --output <fresh-path>
python3 <N>/B/test_causal_readback.py
```

| Item | Verdict after the bounded diagnostics |
| --- | --- |
| Fixed-Q Schur nonlinear relinearization/integrity | PASS, PC OFFLINE VERIFIED scope only |
| Joint bootstrap covariance/correlation/physical scale | UNQUALIFIED; PARTIAL |
| Measurement ownership/atomic runtime handoff | UNQUALIFIED / NOT_RUN |
| Continuous Schur mono V101/V202/V103 | FAIL unchanged |
| Complete OpenVINS causal diagnostics | PASS, including independent complete-row review |
| OpenVINS tracking classifier/operational output | NOT_ESTABLISHED; UNKNOWN/ineligible for every row |
| Covariance consistency/native reset | UNQUALIFIED / BLOCKED-NOT_RUN |
| PX4 source/boot time mapping, EKF accepted/fused | NOT_RUN |
| RK3576 BSP/ABI, sensors/calibration/onboard performance | HARDWARE BLOCKED / NOT_RUN |

No native dataset replay, old repeats, ARM64/QEMU, SITL/EKF, production handoff,
hardware, firmware, flight, M3C operations or Git publication is authorized by
these results. Next work needs a separate bounded decision: investigate the
full-step geometry and sparse-point conditioning, qualify bootstrap joint
uncertainty/ownership before any atomic runtime handoff; separately validate
OpenVINS anomaly hypotheses on independent intervals/sequences without GT in
the runtime policy. These remain software gaps. No next experiment is queued.
All16 author containers, two independent containers and the exact author/review
reader PIDs have exited. `N/finalization.json` records the final fresh absence,
identity/budget/diff checks and current document hashes. Production/shared source
identities are unchanged, so the historical1310 passed/1 skipped regression is
not rerun or relabeled as a new result. The actual fresh diagnostic tests,
compiled controls and independent recomputations above are this milestone's
validation. The active plan §0 is closed; work stops for user review.

## Bounded OpenVINS quality and Schur nonlinear qualification (2026-11-02)

Current milestone status: **CLOSED / PARTIAL / STOPPED for user review.**
Three serial OpenVINS quality replays and the Schur fixed-Q nonlinear
directional checks are complete, including independent cumulative audits.
Operational tracking, covariance consistency, bootstrap and continuous Schur
mono tracking remain unqualified. The historical records below
retain their original source identities and failure verdicts. No old repeat
cohort, production Schur update, ARM64, hardware, M3C or EKF fusion was run in
this bounded scope.

### OpenVINS: direct native signals and conservative measurement diagnostics

The immutable OpenVINS69488123… parent and prior health child remain intact.
New private child `R/openvins/installation-native-quality.json` SHA
`be5e3f1327094c4fbec96a2fc64a281fc7fb9f98f886620fd2ecd0572d86a806`
adds one header/four CPP insertion sites, with unchanged math, gate predicates,
inputs and compile flags. The default-disabled quality sidecar is separately
bounded at200000 rows/100MiB/2048 gate entries per batch; dropped/incomplete/IO
failures cannot qualify. The existing200000-event stream and old configurations
are unchanged. Three packages/four ELF and actual parent-off/child-off/child-on
initialization/EKF outputs are byte exact; independent source/build review passes.

| Signal | Direct evidence / derived scope | Qualification limit |
| --- | --- | --- |
| Initialization | Original native commit and ready transitions | Does not establish subsequent trajectory health |
| Accepted camera constraint | Actual MSCKF/landmark EKF completion and exact accepted support; new original gate/NIS/DOF/feature population and branch witnesses | Native acceptance is not correct correspondence or global tracking proof |
| IMU prediction | Function-local fast-cache operands, requested IMU stamp and integration endpoint; native success/unavailable events | Never refreshes last visual time |
| Covariance | Camera-lock copy of full15×15 IMU marginal and associated16-value mean, exact returned-state binding | Native error chart and numerical properties only; no calibrated flight covariance |
| Receive/progress time | Same-process steady event serialization, independent watchdog ticks and original integer ROS stamps | Not physical acquisition/receipt time, publication time or a source→FC clock mapping |
| Reset/epoch | One isolated process session | Native runtime reset unsupported/TODO; epoch and reset count remain null |
| Overall health / output | Fail-closed existing operational policy | UNKNOWN/invalid; no GT or pose-count/process-exit shortcut |

The existing native decoder, state policy and comparison runner are extended,
not duplicated. `native_quality: {schema_version:1}` explicitly opts in and
requires the already qualified freshness lane. Original configs/trajectory and
fixed-scale evaluation remain unchanged. Source hash/session/time-bit/clock
checks bind complete gate populations to the same camera transaction and
native update, including actual feature counts and completion order.
Initialization retains its different original threshold DOF versus projected
residual DOF; precommit evidence is matched to the subsequent actual commit.
Unknown schema/capability, source substitution, nonfinite/overflowing statistics,
dropped entries, reused snapshots and output overwrite are rejected.

`evaluate_openvins_measurement_health` reports INITIALIZING, UNKNOWN,
VISUAL_CONSTRAINTS_OBSERVED, PREDICTION_ONLY, ALL_NATIVE_GATES_REJECTED,
NO_NEW_VISUAL_CONSTRAINTS, NUMERICAL_REJECTION or STALE from those observations.
This is **measurement diagnostics, not a qualified TRACKING label**. It adds no
retrospective feature threshold, tuned noise, GT input or bypass for VALID.
Only causally available snapshots may inform each original health row; no
future correction is backfilled. Watchdog evaluation, input receipts and
propagation cannot renew visual/state progress. Every current result retains
tracking UNKNOWN, covariance consistency UNQUALIFIED, epoch/reset null and
output eligibility false. A new accepted visual update may restore freshness
and observed support; it does not certify operational recovery.
The pure policy was actually evaluated over recorded native events in causal
order after each replay. This verifies offline diagnostic behavior, not a live
egress policy or flight-qualified health detector.

All three new V1_01 mono runs used the same pinned child, full source interval,
0.5× replay, original scale1 SE(3) scorer and unchanged ground truth. Truth was
available only to offline scoring. Each run has 57 manifest-verified outputs,
449 installation identities, 19 runtime identities and five child exit0 records.

| Profile | Images / IMU | Poses | ATE RMSE m | 1s RPE m / deg | Numerical covariance snapshots |
| --- | ---: | ---: | ---: | ---: | ---: |
| Normal |2912 /29120|2800|0.070115011|0.045237097 /0.478052290|2800 PASS|
| Camera gap500ms |2902 /29120|2790|0.067969763|0.045143984 /0.473858514|2790 PASS|
| IMU bias step |2912 /29120|2800|**13085.221677389**|**392.936389304 /2.206736636**|2800 PASS|

These are different fault conditions, not an algorithm ranking. Normal and
gap full `raw/state_estimate.txt` files are byte identical to their respective
preserved parent runs. No completed five-repeat cohort was rerun, and the
original1cm repeatability gate is unchanged.

The gap reuses seed270031, source20–20.5s and removes cam0 IDs400–409 without
changing IMU. The separate mono bias input uses that fixed seed, source20–30s,
gyro addition `[0.1,-0.1,0.05]`rad/s and acceleration addition `[0.5,0,-0.5]`m/s²
on exactly2000 IMU samples. It is a declared artificial fault, not original
EuRoC or historical Schur bias evidence.

Exact timeline witnesses, retaining integer sensor timestamps:

- Gap event24115 becomes STALE during prediction, keeping last visual
  `1403715293212143104`; actual accepted landmark update24420 advances it to
  `1403715293762142976` and restores CURRENT. This is freshness recovery only.
- Bias event24609 becomes STALE; accepted visual24630 restores freshness only.
  During the fault,2494 native gates include560 accept/1934 reject. The2312
  post-fault camera snapshots contain only two accepted visual updates. After
  event42898 visual-measurement freshness expires again; there is no subsequent
  visual recovery. The final state is STALE.
- Actual initialized stopped-input ticks normal192073, gap191717 and bias182484
  detect stopped streams without advancing state or last-visual timestamps.
- All160922 original health rows were evaluated; the compact transition
  ledgers retain every change. Output eligibility is0 throughout. Diagnostic
  row counts are not counts of accepted visual updates or estimates of frequency.

**Fault evidence integrity PASS; meaningful bias trajectory FAIL; operational
tracking qualification PARTIAL.** The bias trajectory diverges severely despite
2800 poses, finite/positive numerical covariance diagnostics, some accepted
constraints and clean process exit. Freshness detects missing corrections,
but these signals cannot prove the remaining fresh intervals healthy. No tuned
overall-health threshold, covariance calibration or successful native reset is
claimed. Full timelines are in each run's `online/native-health.jsonl` and
`online/measurement-health.jsonl`; original native events/quality sidecars and
their manifests remain sealed.

The existing comparison runner was used serially for each config below:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m rm27.perception.vision.localization.experiments.openvins_comparison \
  --config /home/shiuhou/Projects/rm27-schurvins-research/configs/openvins-health/v101-mono-normal-native-quality-half.json
```

The other two config filenames in that same directory are
`v101-mono-camera-gap500ms-native-quality-half.json` and
`v101-mono-imu-bias-step-native-quality-half.json`. Their sealed output directories
under `R/runs/` are respectively
`openvins-v101-mono-normal-native-quality-half`,
`openvins-v101-mono-camera-gap500ms-native-quality-half` and
`openvins-v101-mono-imu-bias-step-native-quality-half`; existing outputs refuse
overwrite. Run hashes are respectively
`59a2dd11d92451e1f21d4f8943e1bd5ec5dc509c9db24420b9e90669bf97e85b`,
`1ed1170b755858add6f095a2e2a608ef8d63cdae95764990407bc37bbc86e055`,
`eed4ffd556cb830b47ae20e2d527f34153066a40b508517a886600b5530f6add`.

The15-state chart is `[theta_IMU,p_GLOBAL,v_GLOBAL,bg_IMU,ba_IMU]`, using native
JPL left attitude error. The diagnostic inspects the original matrix without
symmetrization/floors: finite entries, symmetry within explicit roundoff and
spectrum of the symmetric part are separate numerical facts. The prior normal
and dropout MCAPs expose111992 finite6×6 pose/twist matrices with positive
symmetric-part spectra, but raw antisymmetry reaches3.525e-7. In particular52
STALE dropout predictions still carry finite matrices. These are direct
counterexamples to covariance-finite/PSD implying valid tracking.
The ROS pose export reorders local attitude covariance without proving fixed
world-angle coordinates; fast angular covariance uses sigma_w²/dt with a native
TODO for bias/cross terms. Missing full pose/twist cross information, selected
correlated innovations, and the initialization DOF mismatch preclude declaring
statistical consistency. The full native marginal observation does not repair
or silently replace those published covariances.

Source timestamps, process-local serialization, native publication time and
PX4 boot time remain distinct. No concurrent clock anchors or physical mapping
are inferred from offline replay rate. The prior isolated five-sample
`vehicle_visual_odometry` reception PASS is retained at its original identity;
its actual uORB timestamp/sample0 and unqualified virtual frames are not time
synchronization. **EKF accepted/fused BLOCKED/NOT_RUN in this milestone** because
tracking/covariance/reset/frame/clock prerequisites remain insufficient. No new
SITL instance, forced VALID, arming or controller was started.

Shared behavior tests cover timestamps, future/foreign evidence, stale revival,
prediction-only, new visual support, native reject/nonfinite decisions, reset
claims, covariance failure, stream completeness/caps, capability opt-in and
legacy trajectory/score preservation. Full regression is **1310 passed/1 skipped**
(exit0,19.51s); affected549PASS. Two development test-fixture mistakes are retained
in the compact contracts receipt: a mono/stereo bag mismatch correctly rejected
by production, and a negative-test population edit which initially made no
change. No estimator/scoring gate was relaxed to fix them. The independent
shared-code reviewer adds22 passing constructed production-path checks.

Evidence in `S/openvins-native-observation/`:

- `quality-contracts.json` SHA
  `ea39fdeb9749a88f2a6a3fabe031704144ad9ec1cc42cf42d0f62a5835cb4c07`.
- `native-quality-source-independent-review.json` SHA
  `3e37df0b4ef9463ae05c17eed193bb001113b07d3efd128e02146f84cfcf83e9`.
- `native-quality-build-independent-review.json` SHA
  `d33b73a13c054431500d3cbc6cfd14bae39535daa4152ba67875b81d34a6656a`.
- `native-quality-shared-independent-review.json` SHA
  `dc2e823cdd855875ecd80f242e3bcfa38500404794f30f44438d527027e69e6b`.
- `operational-quality-covariance-sealed-audit.json` SHA
  `a30974e1a30488d4bb748c3c4915cf0b2dfb5fa5ceae34ee52e7673a486f4435`.
- `native-quality-three-replays-independent.json` SHA
  `c27b7614ce99ba2a975c29d58fc9b8d038b5b9ff504899175e2798f3b661b0f9`.
  Its complete sensor/native/quality/timeline/raw-adapter scoring audit passes;
  the auditor's initial startup-tick witness-selection mistake and correction
  remain recorded, without modifying native data or rerunning an estimator.

### Schur: actual nonlinear descent and its limits

The existing frozen joint window is reused unchanged:49 poses,48 IMU states,
313 points,6897 selected distorted-pixel observations,47 IMU transitions with
484 raw memberships. Noise, calibration, frame/point identities, original Q,
factor selection and the declared local translation/yaw coordinate slice are
locked. The new external `R/bootstrap-visual-likelihood/nonlinear-window-v1/`
probe calls the actual compiled InitImuModel/Prediction/PredictionState and
checks204 dynamic dependencies; it does not replace the estimator or read truth.
The objective is the **fixed-initial-Q nonlinear residual sum**, not a normalized
state-dependent-Q/logdet likelihood or a proven independent-noise model.

Both flags use the same18 predeclared alpha values. A world-left normalized
Cayley quaternion update and one common alpha apply to every pose, velocity,
bias and landmark direction. Full state/point trials use independent copies;
finite-domain failures cannot modify the nominal or accepted offline copy.
Rank-revealing point elimination preserves two rank2 point null directions
(maximum null motion2.52e-17) without point priors or covariance floors.
The declared722-dimensional slice is numerically full rank; this does not
establish physical posterior observability, exact nonlinear yaw invariance at
nonzero residual, or a usable bootstrap covariance.

Actual O1 objective values:

| Common alpha | Visual cost | IMU cost | Total nonlinear cost |
| ---: | ---: | ---: | ---: |
| 0 |920.951972|9297251.595455|9298172.547427|
| 1/16 |857.255202|8171588.540386|8172445.795588|
| 1 |378666.138321|6809.888873|385476.027195|

The production-flags run independently gives total9298534.684→385490.349 at
alpha1 and8172764.096 at1/16. Initial Q is fixed separately within each compile
flavor; cross-flavor bit-identical Q is not claimed. Four directional finite differences per flavor
agree with the analytic total derivative within3e-9 relative error; the
synthetic component checks also verify visual/IMU signs and coordinate charts.
Independent readonly reruns reproduce all records exactly and verify41 frozen
sources/inputs before/after. No sign/chart mismatch is demonstrated by these
bounded checks; they are not exhaustive global model qualification.

**Nonlinear directional descent PASS; physical/bootstrap acceptance PARTIAL.**
The alpha1 grid winner trades a roughly411× visual cost increase for the
IMU-dominated total reduction. Its maximum translation/rotation/velocity changes
are0.143561m/0.152031rad/0.124749m/s; gyro/accelerometer bias changes are
0.009161rad/s/1.434052m/s². The maximum/median landmark displacements are
12.891128m/0.761567m. Positive projection depth and finite unit quaternions do
not make this a reliable initialization. The original linear full-step
prediction476.578 is far below the actual385476.027. Alpha1/16 decreases both
components, but is not a newly selected runtime threshold or accepted state.

The first evaluator build formatting failure and its subsequent294-column
pose-Jacobian versus726-dimensional state-vector assertion are retained. The
latter was a new offline diagnostic shape error, fixed with an explicit pose
projection and dimension guard; it was not an estimator defect or an excuse
to change the locked model. Both final flavors pass six checks/eight negative
controls each. Schur V101/V202/V103 continuous-mono failures remain FAIL.

Before any runtime experiment: separately review bounded offline
relinearization/component costs and weak landmarks; qualify joint bootstrap
uncertainty and its correlations/null support; define ownership of all6897
visual observations and47 IMU transitions; then design an all-or-nothing
mean/point/frame/linearization/uncertainty handoff at exact sensor times.
Only after those prerequisites pass is a fresh isolated single-sequence smoke
justified. None is automatically scheduled by this milestone.

Evidence in existing `D` diagnostic directory:

- `bootstrap-nonlinear-window-qualification.json`, SHA
  `e7c8375b4b08beee96837d89eb2d9f5a496bbb8eaa8c1c404d2b43af7bb857d0`.
- `bootstrap-nonlinear-independent-review.json`, SHA
  `ac0c6efbc8bda88e305d7c1f48588e8750dab9ff328decfce54a0e6f59a6d1ac`.

Both bounded milestones are stopped. Shared source/installation, protected
schema/scorers, matrix identity and known owned process/container absence are
rechecked in `S/openvins-native-observation/quality-nonlinear-milestone-finalization.json`.
There is no queued replay, SITL, ARM64 or Schur runtime experiment. Remaining
software work is qualified tracking/covariance and lifecycle/clock/frame
evidence for OpenVINS, and bootstrap uncertainty/ownership/atomic handoff for
Schur. These gaps cannot be relabeled as hardware-only dependencies.

## OpenVINS operational policy and real-output SITL reception (2026-11-02)

**PARTIAL — fail-closed policy PASS; isolated real-output reception PASS;
operational tracking/covariance/lifecycle and actual EKF fusion are not
qualified. The bounded milestone is CLOSED and STOPPED for user review.**
This work reuses the sealed normal/dropout outputs
below. No estimator replay or old five-repeat cohort was rerun. The estimator
math, v2 schema, scoring and original1cm gate are unchanged. No ARM64, Schur,
hardware or M3C work was scheduled.

`state_output.evaluate_openvins_operational_validity(row, summary)` evaluates
already decoded, manifest-verified native evidence without truth. It keeps
these decisions separate:

| Dimension | Current result / rejection rule |
| --- | --- |
| Freshness | CURRENT/STALE/INITIALIZING only with the pinned session, clock semantics, initialization, accepted-visual and actual state/input progress evidence; invalid times/identity yield UNKNOWN |
| Tracking health | UNKNOWN; a fresh pose/prediction or downstream TRACKING label cannot qualify it |
| Covariance | UNQUALIFIED; finite native matrices are not observability, consistency or health evidence |
| Lifecycle | Process-session evidence only; native reset/epoch remains unknown and runtime reset BLOCKED |
| Normal output eligibility | False on every current native row; tracking/covariance/reset/publication evidence is insufficient. Existing normal ODOMETRY gates additionally require explicit frames, velocity/rate and clock evidence |

The policy checks observation-clock rewind, deadline expiry, missing visual
support and state-source semantics without changing source timestamps. IMU
prediction and watchdog evaluation do not refresh last visual time. A new
accepted visual correction may restore configured freshness, but cannot restore
an unqualified operational validity flag. Unsupported qualification claims
fail closed. The function does not mutate the native record or create a valid
LocalizationEstimate.

Actual readback evaluated all **113767** sealed rows: normal56893
(852INITIALIZING/55782CURRENT/259STALE), dropout56874
(1018/55531/325). Freshness agrees with the prior native decoder on every row;
**zero** rows are eligible for normal output. The actual stopped-input tails
remain STALE. Dropout event24264 stays STALE with the old visual timestamp;
accepted correction24568 advances visual time and restores CURRENT only.
The extraction also verifies all27998 native prediction/MCAP/observer messages
per run. It reads complete pose, local velocity and angular velocity from one
MCAP message, rather than filling angular velocity or joining different states.

The default-disabled `prepare_observer_only_odometry` reuses the existing pure
coordinate/lever-arm conversion and MAVLink codec. Its output is explicitly
UNQUALIFIED, control/arming prohibited, and contains the policy's rejection
reasons. It is not a valid-v2-state workaround. Reception also requires both
`--laboratory-records` and `--observer-only-laboratory`; either missing opt-in
is rejected before launch. The existing synthetic entry remains available.

The lab basis is a **declared virtual LOCAL_FRD** (Rx(pi) of the dataset world,
arbitrary horizontal heading) and a virtual BODY_FRD equal to the dataset IMU,
with zero lever arm. It is not NED heading or aircraft mounting calibration.
Quaternion xyzw is reordered to Hamilton wxyz without extra conjugation;
position/orientation/twist use the shared numerical transform. The payload's
historical `position_ned_m`/`orientation_ned_body_wxyz` field names are retained
for compatibility; its actual wire frame is explicitly20 LOCAL_FRD/12 BODY_FRD.
Quality remains0, covariance is null in strict JSON and NaN-first unknown
sentinel on the wire. Reset0 is a fixed laboratory transport-session value,
not an observed native reset counter.

The first selected sample preserves ROS header `1403715278872143030ns`, sent as
`1403715278872143us` with30ns remainder; its native prediction endpoint is
`1403715278.871349s`, different from request `1403715278.872143s`. Those fields
remain distinct. No source-to-FC clock mapping or endpoint relabeling is
inferred. The minimal PX4 lockstep clock is undriven: all observed uORB
`timestamp`/`timestamp_sample` values are0. Source/wire integer scaling passes;
**PX4 timesync and fusion timing are NOT_RUN/unqualified**.

Five distinct sealed samples were actually transmitted (five identical UDP
deliveries per selected sample for bounded listener capture):

| Case | Native prediction event | Freshness | Normal eligibility | Actual PX4 reception |
| --- | ---: | --- | --- | --- |
| Normal first prediction | 4717 | CURRENT | False | PASS |
| Dropout before stale | 24260 | CURRENT | False | PASS |
| Dropout prediction while stale | 24264 | STALE | False | PASS, laboratory only |
| First prediction after visual correction24568 | 24579 | CURRENT | False | PASS |
| Normal final prediction | 192049 | CURRENT | False | PASS |

These samples are labelled by their separate native source sessions, not
stitched into a reset/continuous-state experiment. High-rate prediction is not
new visual correction. Each actual `vehicle_visual_odometry` listener output
matches the wire's position/quaternion/velocity/angular velocity, pose_frame2,
velocity_frame3, quality0, reset0 and variance sentinel. MAVLink float32 values
are checked exactly against source quantization; the text listener is checked
within its printed precision. PX4 copies the unknown covariance sentinel's
remaining zero entries into some variance fields; they are not qualified
zero uncertainty.

Only dataman and MAVLink run in an owned `--network none`, read-only,
capabilities-dropped container,1CPU/1GiB, with loopback-only UDP. Commander and
ekf2 are proved absent before and after reception. Thus **message received
PASS; EKF accepted NOT_RUN; EKF fused NOT_RUN**, with no arming or control use.
MAVLink stop and native SIGINT both exit0; owned process/container absence is
verified. The PX4 binary is the exact retained artifact
`433454f312553331cc3b3b0d515f3f8831bb52a0f2b12d04d430b8981d016bdd`,
build commit `dd0ad74fdadc68a62b479129ca3f382c006426f4`, image
`sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517`.
Only the locked binary/version were copied into research-root artifact storage;
no external repository Git command or mutation occurred. Its24 dynamic
dependencies are locked and rechecked.

The initial SITL attempt remains FAIL with zero cases transmitted: this pinned
`ekf2 status` silently returns0 with no instance, so the new absence check
correctly refused empty output. The repair uses the pinned `ekf2 stop` branch:
only absence of every selector/instance emits explicit `not running`; any
`stopping`, lock failure, empty output or wrong return code is rejected. This
probe never starts an estimator. Source semantics were independently checked;
the failed run and original1228/1 test receipt remain unchanged. Final current
regression: **1236 passed,1 skipped**;55 MAVLink tests and frozen source hashes
cover normal conversion, dual opt-ins, frames, velocity/rate, timestamps,
sentinels, unsafe labels, overwrite refusal and the absence-probe negatives.

Reproduce using the existing entry and a fresh output path (existing outputs
are refused):

```bash
source /opt/ros/jazzy/setup.bash
R=/home/shiuhou/Projects/rm27-schurvins-research
PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}" python3 tools/schurvins/verify_sitl.py \
  --artifact-lock "$R/sitl/artifact/lock.json" \
  --image sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517 \
  --laboratory-records "$R/configs/openvins-health/operational-sitl-records.json" \
  --observer-only-laboratory --output "$R/sitl/operational-reception-module-probe-v1"
```

Evidence under the existing research root:

- Actual reception: `sitl/operational-reception-module-probe-v1/receipt.json`,
  SHA `39c7e4a2e734b0225231d54f6f22e35ab66d07d9ca653d94e4339b7eb3b90aaa`.
- Actual listener/wire/process details: same directory `sitl-result.json`,
  SHA `38d77011ad4ca00999a20062bbb38bf6d88f46c8df01ab3d9c654d3295b5a31f`.
- Input records SHA `08a2da7261426c685e0ec4f738a8af46e7e78c5dbdb5a10133fdfddb2e401764`;
  portable artifact lock SHA `1a68b38154c84f6266520acd7e21b0e08bbae3da2dab06b19f0832939ce13335`.
- Sealed-input/policy audit SHA
  `3a4eb2a7dfbb344d040dab7f1fcbe0f47b97d1b3f41b6558cabdbbf0001e4e1d`;
  preserved failure/repair SHA
  `95f1fa233bfb966471a24967c3ef35a233182c9983b727ef18f86a0969a61189`.
- Final software regression SHA
  `ea4ae443d4fa3741a8c79abaa5f9d78310229dad0cb16766b4f320f60bd640ee`.
- Independent review: existing diagnostic directory
  `.agents/runs/schurvins/state-flight-mock/openvins-native-observation/operational-validity-independent-review.json`,
  SHA `2ea32c457be4c8f09df663abe84bc1e77ec6d36a5295d8c54e2384efef6d48d0`.
  All five source/CDR/native/policy/wire/uORB cases, twelve current source
  identities, the complete regression and cleanup pass; no review blocker remains.
- Final stop receipt in that directory: `operational-validity-milestone-finalization.json`,
  SHA `ff9124548a0944bbd441738d8bf17207cf78a1650876cf397b30f2a297bc4f6c`.
  Fifteen protected runtime files and the sole matrix are unchanged; both known
  owned containers are freshly confirmed absent. All scoped agents are finished,
  and no follow-on work is queued.

These are **PC OFFLINE VERIFIED** policy/source facts and
**EMULATED / MOCK VERIFIED** actual PX4 SITL reception, not flight/controller
eligibility or RK3576 performance. Tracking-health classification, covariance
qualification, real body/world calibration, FC time synchronization and native
reset semantics still block operational use. No next experiment is authorized
by this result; this milestone stops for review.

## OpenVINS native freshness/lifecycle milestone (2026-11-02)

**PARTIAL — PC OFFLINE VERIFIED for native observability and configured
freshness; operational health remains UNKNOWN/invalid. The bounded milestone
is closed and stopped for user review.** The work
executed one full normal mono replay and one 500ms camera-gap mono replay,
including their stopped-input watchdog tails. No old five-repeat cohort was
rerun, no Schur/P0–P7 schedule resumed, and no hardware or new SITL run occurred.
Runtime reset is **BLOCKED / NOT_RUN**: the pinned estimator has no implemented
runtime reset API. Terminal shutdown/new process construction is not native
reinitialization evidence.

The software gaps were absent fast-prediction witnesses and no composed native
freshness timeline. Separately, `StateOutputTracker` allowed an old/equal-time
IMU receipt to revive a stalled snapshot. Six negative cases failed before the
repair; actual state progress now has its own observation time, independent of
raw receipt. Watchdog ticks update neither time. Full regression: **1127 passed,
1 skipped**, affected tests514PASS, source identities stable and
`git diff --check` PASS. The legacy OpenVINS runner, v2 schema, fixed-scale
SE(3)/ATE/RPE and original1cm repeatability gate are unchanged.

The first instrumentation attempt's normal run remains
**FAILED / OPENVINS_HEALTH_PREDICTION_INVALID**, with its native process exit0
and numerical evidence intact. Event72249 exposed the decoder's invalid
assumption: a prediction cache offset can differ from a later, asynchronously
serialized main-state offset. The final read-only patch records function-local
cache time/offset and checks the exact integration equations. It does not relax
a tolerance or change estimator math. Parent/child production calls preserve
byte-identical mean/covariance with observation enabled/disabled (24cases).
Three-package incremental build and four ELF/dependency closure pass.

Changes are confined to the existing comparison entry's opt-in capability and
sidecar integration (`experiments/openvins_comparison.py`), the native decoder
(`tools/openvins/native_observations.py`), the state-progress freshness repair
(`localization/state_output.py`), the read-only native patch and OpenVINS README.
Behavior regressions extend the existing native-events, OpenVINS-runtime and
state-output tests; no duplicate runner/scorer or status document is introduced.

The final installation is `openvins/installation-native-health-cache-witness.json`
under the existing research root, SHA256
`346d184d2ee655b7f43faff67d2998753fcb2697fe5c0a0b932fa6750904b2fc`.
Upstream remains `69488123ed9362dd44b6f28e7f4680abbff1442b`; native capability
schema2 is explicit, while opt-in comparison config schema1 is preserved.
The isolated ROS2 Jazzy image remains
`sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14`.
Installation build-time NOT_RUN fields stay immutable; actual qualification is
recorded by the separate per-run receipts. The existing200000-event/100MiB
limits are unchanged; both mono logs have complete
zero-drop footers. This budget is not qualified for new stereo workloads.

| Signal | Direct native evidence | Derived state / remaining limit |
| --- | --- | --- |
| Initialization | State/covariance commit event; returned camera state inside actual `initialized()` gate | Commit and ready are distinct; first-pose count is not health |
| Accepted camera correction | Completed MSCKF/SLAM/delayed-initialization EKF with accepted measurement support and closed camera transaction | Only strictly newer accepted support refreshes last visual time |
| IMU prediction | Actual fast-propagation success/unavailable return, cache operands and finite flags | Separate from camera-driven clone propagation; neither refreshes visual time |
| Timestamps | Integer ROS sensor stamps; native steady event-serialization time | No hardware receive/publication time or sensor/native clock mapping inferred; sampled main state is asynchronous |
| Freshness / stopped input | Independent native20ms tick plus actual input/state/visual witnesses | CURRENT/STALE use explicit PC test limits, not flight safety or a native tracking classifier |
| Reset / epoch | Runtime reset is an upstream TODO, no callable reset API | BLOCKED/NOT_RUN; native epoch/reset count null; process-session adapter epoch0 is not a native reset counter |
| Covariance / quality | Native marginal/fast covariance and finiteness witness | UNQUALIFIED covariance; operational health UNKNOWN, validity false |

Both cases use full V1_01_easy mono+IMU at0.5x,4CPU/12GiB, the same original
truth/hash, and scale1 scoring. Normal has2912images/29120IMU. The artificial
case reuses seed270031 and the existing20–20.5s camera-gap profile, dropping
cam0 source IDs400–409:2902images/29120IMU. This is a new mono derivative of the
profile, not the historical stereo run. Original timestamps/pixels/calibration
and source denominators are retained; truth is outside the estimator mounts.

| Case | Result | Poses | Fixed-scale ATE | 1s translation / rotation RPE |
| --- | --- | ---: | ---: | --- |
| A normal full mono | PASS numerical/input/freshness evidence | 2800 | 0.0701150107652m | 0.0452370974100m /0.478052290355deg |
| B500ms camera dropout | PASS numerical/input/freshness evidence | 2790 | 0.0679697628194m | 0.0451439837438m /0.473858513512deg |
| C input stopped / watchdog | PASS in both real native tails | — | — | Sensor/state/visual times remain unchanged |
| D native reset/reinitialization | BLOCKED, execution NOT_RUN | — | — | No synthetic reset substituted |

The small ATE difference is not an algorithm ranking or an improvement claim.
Normal native state bytes match the retained earlier A exactly. Each final run
has54sealed outputs,448installation hashes, complete camera/IMU native coverage,
raw/adapter/saved score equality, five child exits0 and exact container absence.
There are192333/192144native events, respectively; high-rate predictions are
not new visual updates. Operational validity remains false on every projected
row; freshness recovery is not a native tracking/reinitialization qualification.

The health timeline is `online/native-health.jsonl` in each run; its compact
summary is `native-health.json`. Normal ready is event4713; after input stops,
event192065 becomes STALE. Dropout has CURRENT → STALE at event24264 while
IMU prediction continues and last visual stamp remains
`1403715293212143104`. New accepted landmark evidence at event24568 advances
last visual stamp to `1403715293762142976` and restores configured CURRENT.
Its stopped-input tick191889 returns STALE. No new native epoch is inferred.
Observed first-input→ready intervals are11.213631006s /11.214694295s in the
native observation clock at0.5x replay; these are not estimator execution
latencies. Limits are300ms input/state observation age,300ms visual sensor age and600ms
visual observation age; no sensor/native clock subtraction is used.

Reproduction uses the existing runner and immutable one-off configs:

```bash
source /opt/ros/jazzy/setup.bash
R=/home/shiuhou/Projects/rm27-schurvins-research
python3 -m rm27.perception.vision.localization.experiments.openvins_comparison \
  --config "$R/configs/openvins-health/v101-mono-normal-half-cache-witness.json"
python3 -m rm27.perception.vision.localization.experiments.openvins_comparison \
  --config "$R/configs/openvins-health/v101-mono-camera-gap500ms-half-cache-witness.json"
```

Those output directories already exist and must be refused on repeat; preserve
them and use an explicitly reviewed fresh run ID/output for any future rerun.
Outputs are `runs/openvins-v101-mono-native-health-half-cache-witness` and
`runs/openvins-v101-mono-camera-gap500ms-native-health-half-cache-witness` underR.
Run manifest hashes are `4246ed5e33460d056b7aaff6abe00c86ee41ec94202e1bec1c412287091b72d5`
and `8726fc56ca039a756133bfb68e1ce2c928b4be0ad313ca090647011928f069ca`.

Independent cumulative source/build review SHA256:
`63b9f878ff9611a2c165144c95f077fc3950431111ff0283b06967a95f8535ed`.
Final A/B integrity reviews:
`f250a35f201766b868c5e3612bd9826438e7f30ebf0a04bfde43426d9f1cf6ee` /
`629baaad37f8ec17a78c6c0fe256d4b5f8933d64e5b2325d8e99f418948d4333`.
Shared regression receipt:
`e54016abace9991df7c9f32dbd77922688042fbf995d0b53669e37db824bd40b`.
Independent timeline reviews verify all 56893/56874 projected rows and 27998
exact cache predictions per run. A/B receipt SHA256:
`3ec3a81f2149da99292ca0df763cdbda441f5abb60f696ecea640a60d0b923bc` /
`7a12e662e8eeb89dddfc022796ae2d86184f5bd573be73d4290c1e7750bba408`.
The B audit first rejected with a diagnostic TypeError: its fault-window
selector dereferenced a null sensor horizon before the first callback. The
failure is preserved; a minimal guard excludes those rows only from membership
in the sensor-time fault window. All450 pre-callback timeline rows remain
validated as INITIALIZING, and all73 known-horizon fault selections are exact.
No runtime source, test, score, timeline or native replay changed for this fix.
Rejection/repair receipt SHA256:
`43fcfa17d9265c421701170ab87de31bbea7446e7084925e067e625eb1c6f6ae`.

Final stop receipt SHA256:
`aa17cbd3b4c9fd6c3a405cc4ffb654456f6deca60d0140046c8a2539ea00a07f`.
Seven frozen source identities, fifteen protected prior runtime files and the
sole prehardware matrix remain unchanged at closure. All three known owned
native containers are absent; native runs and audit agents have stopped. Git
branch/HEAD and unrelated dirty work are preserved, with no commit/push/merge
or M3C/hardware operation. These are PC offline evidence, not ARM64/RK3576
qualification or actual VIO→PX4 fusion. No further experiment is scheduled.

The next minimal VIO→PX4 SITL step, **not executed in this milestone**, is to
qualify a truth-independent operational-validity policy, then connect the
existing native corrected state/prediction evidence and watchdog to the existing
ODOMETRY adapter. Require explicit world→SITL LOCAL_NED, IMU→BODY_FRD, velocity,
clock mapping and process/epoch provenance. Start with an isolated observer-only
SITL reception check; UNKNOWN/stale states must remain suppressed. Only after
those gates pass should a separately reviewed actual-VIO fusion test proceed.
Current synthetic/mock SITL evidence is not reused as proof of that fusion.

## Previous review checkpoint — stopped at the user's request (2026-11-02)

The previous milestone closed and all its owned work stopped. The serial
OpenVINS batch ended after13 completed members; its automatically started stereo
repeat1 was interrupted for this review and retains EXECUTION_INTERRUPTED.
All14 owned containers are absent and their retained artifact hashes verify.
The batch remains PARTIAL with execution_complete=false; no interrupted member
is replaced or called PASS. All subagents have finished. No new nonlinear solver,
build or replay will start without the user's subsequent instruction.

- **OpenVINS PC OFFLINE VERIFIED:** V1_01 mono at0.5x and1x each completed smoke
  plus five fixed repeats. Each has2800 poses, ATE0.0701150107652m and1sRPE
  0.0452370974100m/0.478052290355deg. Both original1cm/ten-pair gates pass with
  zero exceedances; health remains UNKNOWN/invalid. Independent receipt hashes:
  `9e795f1ea118ba061741664784b25f68a75eb5b2962bc8c8878f1e32ab0c5383`
  and `d3757f086520d08f10bb53f8a0b216ae2e32afb1e1c5fe213bd6f12b700a1bf7`.
- **Offline bootstrap tool verified, no tracking repair:** new external research
  code `bootstrap-visual-likelihood/bootstrap_local_system.h` and its fixture
  combine the explicit selected visual factors with captured IMU factors. An
  actual same-size frame-order binding defect was reproduced and repaired.
  Both O1 and exact production flags pass25 checks/12 rejection controls,
  standalone-header compilation and dependency/method-binding checks.
  Actual data supplies49 poses,48 IMU states,313 points,6897 observations and47
  transitions/484 integration memberships. The declared722-dimensional gauge
  slice is full rank; two point directions remain unobserved. No covariance
  floor or artificial point prior was used. Independent cumulative review:
  `1d8ec799d0a7b6afe5f9ce742489ec06b0b7913f9008e27779427aa9a6c12a6a`.
- **Still FAIL/PARTIAL:** reliable Schur monocular tracking is not repaired.
  The large nominal IMU residual and mixed-unit local step require nonlinear
  verification. Joint bootstrap uncertainty, correlation/measurement ownership
  and atomic runtime handoff remain software work. Existing InitializeMetric
  and Point::EkfInit cannot accept the full joint information by merely copying
  diagonal blocks. No nonlinear step, joint prior or production commit passed.
- The most recent complete shared regression remains1078 passed/1 skipped;
  its37 source identities are unchanged. The new external C++ diagnostics have
  their separate checks above. Existing ARM64 reference/QEMU evidence is
  preserved; it does not qualify RK3576 BSP/ABI or onboard performance.

Stop finalization receipt SHA256: `0ae9f2859ddcd010faaa98841c35b0400d61b3cb40cb6c7fdb26e85475d72940`.
The working branch/HEAD and all dirty files remain preserved; no commit, push,
merge, hardware operation or M3C worktree change was performed.

The observer-only bootstrap child has completed isolated25-package compilation,
bounded capture/lifecycle helpers and independent cumulative review (SHA256
`ada513a066ab3f2639b6eebda13faa2562e91bcf4549b0e466579657996b5c25`).
It records the immutable alignment window, original geometric pair, actual camera
model/pixels and original IMU timestamp double bits, with explicit population
bounds and completeness. No filter mean, prior, threshold or RNG changes are
included. Its full V1_01 replay has now exited with TRACKING_LOST; all four
children exited zero and the owned container is absent. Independent capture
integrity review passes (SHA256
`1b4ddf90c1423cdef03b66ad593a2ca595a31f06f075cf1962ff399dfe644a41`).
The eight poses cover only0.35s; no1sRPE exists. Joint bootstrap uncertainty remains unqualified.
This preparation is not a monocular tracking repair.

The fifteen remaining 0.5x artificial-fault replays have completed using the
existing entry point. Independent integrity audit verifies585 sealed outputs,
60 child exits equal to zero,15 exact container absences and unchanged shared
source identities (audit SHA256
`8f5bead5b4ea9ebbe953b18bab446ca50c1a944ce859ea746d04f5e678222235`).
The selected result is PARTIAL:8 recovery gates PASS and68 NOT_RUN across the
retained full matrix scope. Both IMU-gap cases stop at400 prefault poses with
no recovery. The bias-step case emits2912 poses but diverges severely:
ATE12767.2861682m and1sRPE364.307734574m; its algorithm outcome is FAIL.
The5s camera gap produces2812 poses, ATE0.554688531786m and1sRPE0.305740599327m.
All eight configured recovery successes later contain BAD episodes, so two
corrections do not establish sustained health. Final native GOOD (12 runs) or
NOT_GOOD (3 runs) remains separate from stopped-stream watchdog INVALID (all15).
Shared-PC offline measurements do not qualify onboard timing.

The formerly storage-blocked full-host IMU-gap autoreinitialization replay has
now completed. Independent review `c043c75f2c9cacb40f297ae1178304741c57499fdabed1cdb4753f1b45563aca`
verifies all four child exits,39 sealed outputs,16 runtime identities and the
state/propagation freshness boundary. There are2910 poses in two epochs; their
separate fixed-scale ATE values are0.0269504872531m and0.0426877869658m.
The configured two-correction recovery gate passes, but later quality dips remain.
The propagation artifact retains FAIL with22 sequence-gap rejections: all occur
before the new epoch's accepted visual snapshot, with the old snapshot invalid.
Nine subsequent reset-anchor-unavailable rows are also invalid. This readback
finds no post-snapshot sequence-gap recurrence; it does not relabel those failed
rows or claim continuous health. A stopped-stream watchdog remains INVALID.
This is PC OFFLINE VERIFIED, with no hardware claim.

Explicitly selected visual factors and actual captured IMU factors now have
sealed author qualifications (`8b6c72a581b6932d8ff435d1c37e15cac4063c4ff09dba1e73382ff99c2ead6b`
and `1a2344868dc888d834b2be0dfcae7a12132da59ccc78bed42760f6d46791f2c5`),
with independent reviews now closed as noted above. The former retains6897 observations from313
actual point identities and lists all626 exclusions; the latter checks47
transitions with484 integration memberships. These are offline foundations for
an explicitly conditional, frozen local Gaussian surrogate, not a qualified
joint bootstrap prior. Shared raw endpoints do not prove independent sample
noise. The combined diagnostic now passes the bounded offline review above; it remains
unconnected to estimator initialization or valid-state output.

The closed observer-only sidecar passes the existing strict decoder in the
independent readback:49 frames,7,523 observations and579 IMU members. Of those visual
observations,6,217 carry actual PointIDs and1,306 have no Point yet. Actual
captured promotion and exact owner identity can link680;626 remain unresolved.
A declared conditional observation subset must report every excluded identity;
it cannot stand for complete-window likelihood or tracking success. The run
manifest SHA256 is `3a38882b9e70acace10ff230e25f1f9d5c2fe8a28afcbf7ea78a79e4e5d48574`.

The offline IMU process primitive has passed independent review of the actual
compiled prediction methods, zero-initial-covariance process Q, raw-double
membership and three build groups (25 checks/54 rejection controls each).
Additional residual-chart and actual-process derivative tests pass in O1 and
production flags (29 checks/7 rejection controls each). Their nonzero-residual
covariance is only a fixed-target prediction-noise first-order pushforward;
normalized density, whitening and joint bootstrap prior remain unqualified.
A separate visual cross-review reproduced nonfinite final Jacobians from huge
finite inputs and a nonunit pose from an overflowing increment. The narrow
postcondition repair passes two new build groups (180 derivative tests and
21 rejection controls each), with independent cumulative qualification PASS
(SHA256 `e276725c312a48fa9a66ecd8aee7403e4d717b0f7e71e81b91235b0490dea4a2`).
The IMU primitive separately accepted a zero quaternion after huge finite gyro
input. Its narrow unit-output rejection now passes two groups of25 checks and
56 rejection controls, independently reviewed (SHA256
`f3c5407fe4e8e18e8186f1e992ec8c3d751c0fe73cc7d07fe17252acf9905992`).
Relinked residual tests pass29 checks/7 controls in both groups, independently
reviewed under the new identity (SHA256
`d609a19d5f835e1e9a5b054db2cb8ddbc9e4109edecc94436b28f0ebfdbf7173`). Earlier qualifications remain historical. None of these offline foundations is a monocular tracking
repair. The selected Gaussian error-coordinate process model would use
Q_predict directly; the fixed-target pushforward is not its nonzero-residual noise.

With additional storage available, the previously blocked full host-state
50ms IMU-gap automatic-reinitialization experiment has passed a fresh input and
installation preflight. Only run ID and output directory differ from its
original configuration; the explicit state policy, complete0.5x input and native
parameters are unchanged. The full host-state native experiment completed
with all four children exiting zero and verified container absence. Existing
integrity and configured recovery gates PASS (SHA256
`90cf54526992b0165d448b44047ed929055d4d53a5951fcc8a1daaadd0a5829b`).
Its2,910 poses belong to two separately scored epochs; continuous trajectory
qualification remains false. Independent full-state/propagation review now passes
under the precise limitations stated above (SHA256 `c043c75f2c9cacb40f297ae1178304741c57499fdabed1cdb4753f1b45563aca`).
The seven-cohort OpenVINS queue has stopped for user review; two mono cohorts
closed, one stereo smoke completed and its next repeat was interrupted. The
original smoke/five-repeat protocol and1cm threshold remain unchanged. The earlier blocked run and
successful core-only reinitialization remain separate evidence.

The preserved pre-readiness V103 mono 1x fixed cohort retains a failed third repeat:
`OPENVINS_NATIVE_INPUT_INCOMPLETE`, despite normal native/container exit. The
independent observer received all 2,149 images and 21,500 IMU samples, whereas
native evidence starts after original camera ordinals 0–16 and IMU ordinals
0–170. All recorded endpoints are reliable; graph discovery and the observer's
publisher match do not establish the native subscriber's match. The cause is
under investigation. This member is neither replaced nor described as an
initialization crop. The stereo smoke plus five repeats completed under the
same frozen runtime identity. Independent stereo cohort readback passed
(SHA256 `72c20f5230079b730bcdcd5ac7c8d4c04726a728198fa49b55a91fa25055c31d`).
All 60 child exits across the twelve runs were zero and owned containers absent;
normal exit does not override the mono input failure.

The new Matcher diagnostic reproduces all 275 original tuples bit for bit,
independent receipt SHA256
`e42bff9d5fcda86497137d1656f0f14cba0660d4cc6dff3eacfbd2ca5c485581`.
A separate bounded diagnostic made 189 actual calls: seven nominal controls,
168 fixed nonzero pixel offsets and 14 iteration changes. Author results show
local alignment sensitivity. Independent review SHA256
`6a6c5fc13aefcb9a64e95f56610d4d37c1af81f767553b859159e236677b3816`
verified all 189 calls, allowed input differences, 198 loaded ELF identities and
owned cleanup. Correct correspondence,
Schur acceptance and continuous mono tracking remain unproved. Even recovering
all seven lost alignment points would yield only 37 inliers, below the unchanged
40 threshold. Seven other lost points had successful matches but failed 4px
reprojection; 11 occupied-grid skips cannot be counted as successful matches.

The offline camera-transaction metadata projection is implemented and tested.
It retains the last actually accepted native state, velocity and merged
measurement support within observed CameraData boundaries. Propagation, ZUPT
and reused older support do not refresh visual time. The actual fixture has 69
records and four transactions with 0/1/41/3 accepted updates. Independent old
native readback preserves legacy summaries and source/output hashes; this is
not a fresh estimator replay. Tracking remains UNKNOWN/invalid; epoch/reset,
publication time, covariance and clock mapping remain unknown.

The optional native reader-match build passed three-package compilation,
19 actual no-sensor MOCK cases and independent cumulative source/ELF/lifecycle
review (SHA256 `bd8e7ef53030779e5aa91077719c513c71f3294b08bec88c902835f4d71fddd3`).
The controller now validates its bounded atomic snapshot against the actual
container child PID, mode, required camera/IMU identities and positive match
counts, and requires no pending publication before releasing paused playback.
Legacy builds retain their existing path. These sequential reader-match
observations do not prove complete sample delivery or explain the historical
missing prefix. Full source/native input reconciliation remains mandatory.
Shared controller review (SHA256 `425027086833d664c3439742f2f34c2de464172e1256475e65929c0a6598038a`)
and the nine-configuration lock migration passed. The single activity matrix
now has SHA256 `4cc25e6d8e6528b704c9acd0b81d505f885488a1813ea212e201f3e171be3192`;
75 gates, 67 required cohorts and all protocols remain unchanged. The fresh V103
mono 1x smoke plus five-repeat cohort completed: all six members and the original
1cm ten-pair criterion passed. Its selected snapshot is still PARTIAL (1 PASS,
0 FAIL, 75 NOT_RUN), SHA256
`da3d34a272ec09201702fd70f7efee786835ae765c02d214d875388c8bd16d2e`.
Root checked 52 sealed outputs per member, all 30 actual child exits zero, six
owned containers absent, and 37 shared source/fixture identities unchanged.
Complete independent cohort readback passed (SHA256
`f86979e8a7056c0101c8e50988424f901192c23f23bb04eeed6eae3c016ea2c3`),
with a separate 37-identity closure (SHA256
`f2c669f92361fb779c62ee9cd5cc0a2f557ac1c51bb39f05aaf29a42ca1862b0`).
Each run contains2005 poses, scale1 ATE0.0639299891115m, 1s RPE
0.0393038727239m/0.240055361930deg, and adjacent RPE0.00417034113m.
Coverage is93.2992%:144 frames before the first pose, no internal or final
missing poses. The7.2s first-pose proxy is not a measured initialization
transition. Native camera-update P50 ranges6.85–6.96ms, P9511.158–11.630ms;
the maximum across all members is182.81ms. Native process-tree average CPU
ranges0.2500–0.2753cores and peakRSS133292–137356KiB. These are instrumentation-
inclusive shared-PC measurements, not onboard latency or fair algorithm ranking.
All2149 transactions per member include1996 accepted transactions and1891
with multiple accepted updates. The sole transaction using only older support
does not refresh visual time. Tracking remains UNKNOWN/invalid; reset, health,
nonempty terminal drain and late-export behavior remain NOT_RUN.
The V103 stereo1x smoke-plus-five cohort has also completed under the same
frozen source and matrix: all six members and the original1cm criterion passed,
with56 sealed outputs per member,30 zero child exits and six owned containers
absent. Its result SHA256 is
`19fb3b5dc4fae2e4505fcc42b82bde78f7ba645e9ab0ac22e9d6733206b51884`.
Complete independent readback passed (SHA256
`487cfc34f7dcbf05469bfee4783eb090823f394efb6c8c88e0d282a8a8066173`),
with37-identity during/after closure SHA256
`b2ea330c778f8cfdf4513aa2563b5c6f4cdc8e652a2e23a1eadc3eb23c151b1f`.
Each run has complete2149 callbacks per camera,21500 IMU callbacks,2005 exact
native-state/ROS payload matches and no unbound transactions. Repeat1 has
scale1 ATE0.0556938258115m and1s RPE0.0352426566849m/0.239762022506deg;
the other five runs have ATE0.0639170372040m and
RPE0.0345613633471m/0.251241586608deg. The largest ATE difference is
0.00822321139247m, passing the original1cm criterion, while the actual maximum
trajectory difference is0.0455435194888m. Passing that criterion does not imply
identical trajectories. Native camera-update P50 is9.15–9.51ms,
P9515.786–16.170ms, maximum189.03ms; native process-tree average CPU is
0.3398–0.3601cores and peakRSS158992–173856KiB. Instrumentation and shared-host
limits still apply. All states remain UNKNOWN/invalid; no reset/health or board
qualification is added.
Three camera delivery-delay1x profiles completed through the existing matrix
and runner. All39 outputs and16 runtime identities per run, exact inputs,
source/devel/patch lineage, fixed-scale raw/adapter scores, twelve zero child
exits and owned cleanup passed independent readback. With20/100/300ms delay,
pose counts are2912/2911/2907 and scale1 ATE is0.0650970/0.0615911/0.0851645m.
Independent integrity receipt SHA256 values are
`f906dcaea6d91b2263d906139e0df4345f8a61b16d9d46d8e04833a2ab3391ca`,
`44c7b4e7ee664a142f6099a73c8bcd2b6a4d0110b00ff9880f2d5b79fdd3f667`, and
`648d8881bdb3479a986a7b69e8d2d57d181545e5542c38176b18d102b60b88da`.
The missing1/5 poses match explicit native camera-order rejections: when the
20–30s delivery-delay window ends, original frame600 overtakes delayed frame599
or595–599. All2912 source/derived/published/observed stereo pairs and29120 IMU
samples are retained; this is not sensor or transport loss. Independent full
publish-order and native admission reconciliation passed (SHA256
`89333d60c5980856a86844eaca4a9403b1ccc608832506528dfda6680025b50c`).
No timestamps were sorted, retimed or supplemented. The20/100ms strong recovery
gates remain NOT_RUN because their loss occurs outside the declared fault
interval. The300ms configured PC offline health gate passes only the recorded
same-receipt-clock interval association and two accepted visual corrections;
sensor/observer clock mapping and elapsed time from detection remain unknown.
None of these results establishes onboard health or continuous trusted state.
The result remains PARTIAL (SHA256
`872866aab4ab95842d9f3667199360e5ef707b1282295628e302e9a8301be5c4`).
The10/50/200ms IMU-gap1x group has also completed full replay. All three retain
only400 pre-fault poses: native gap invalidation resets the estimator, and this
explicit auto_reinit=false configuration does not recover visual output. All117
sealed outputs and twelve zero child exits with exact owned cleanup were checked;
37 shared code/fixture hashes remain unchanged. The result is PARTIAL (SHA256
`dca9d527117490b1bd002e5f2b6813d059280dfcd2e9478cf4b2021697fe2113`), with strong
recovery NOT_RUN; evaluable pre-fault scores do not establish continuous VIO.
Independent10/50ms integrity readback passed (SHA256
`5771dda1a9cc5070c9403f22db9fc6974e6030fa57553e92e2b42bd0a8cf1c1a` and
`4058aebb73f6aecf6628a252fe878d1665714b15a0ca8674d9ae3482d7339f52`).
The declared10/50ms removal of2/10 samples produces raw retained intervals of
15,000,064/55,000,064ns. These publisher/observer intervals are distinct from the
unrecorded exact native IMU-buffer rejection interval; actual per-sample
integration is unobserved. The original15ms protection boundary is unchanged.
Independent200ms and complete boundary reconciliation also passed (SHA256
`8930897c7f5a96354ff4fcb986d7d61c250aed8ff239338404c1113cffdf970e` and
`0e31af3e134e5e6bf58e5f5d1de527e2d0e97a77164aa74c14e9447a583671c8`).
The200ms group removes40 raw samples; its retained interval is204,999,936ns,
not exactly205ms. Each group has399 actual accepted visual corrections and400
poses, covering13.736% of the original full sequence, all before the fault.
The2s visual occlusion1x smoke has completed with2912 poses, scale1 ATE
0.0920730661345m and1s RPE0.0720828449412m/0.513716401782deg. All39 sealed
outputs, four zero child exits and exact owned cleanup were checked. The selected
snapshot remains PARTIAL (result SHA256
`cc47ebf6f9c230a5e1782695b5ee2463cb5e011ef8031f64639d5630fc40df80`).
Independent integrity and actual visual-health boundary readback passed
(SHA256 `de132d35470d2f2f1c97cd6a6e7f993c6473cff18520bceb85126dd19e309913`
and `d35df380b7ff685a4d1a2f160554f41f075c954b926fec9cadb8935394751deb`).
All80 injected PNGs are black. Frames400–439 still emit numerical poses but
have no accepted visual correction and are unusable in the offline projection;
last valid visual time remains399. Frames444–447 have constraints with bad
quality and do not refresh freshness. Only448/449 provide two GOOD corrections,
0.4/0.45s after the fault ends, in the same epoch with no post-startup reset.
This passes the configured PC offline recovery gate, not onboard health or
reinitialization acceptance.

The IMU bias-step1x replay then completed2912 raw poses and four zero child
exits, but host postprocessing failed with PROPAGATION_INPUT_INVALID. Its33
sealed outputs and owned cleanup were checked; no completed adapter score or
state-output artifact exists. The propagation contract compares declared
artificially altered samples against original unaltered source payloads. The
original source ordinal and the verified delivered payload must be represented
separately. The failed result is preserved
(SHA256 `77cbd8b5b61cb714433b2810210def7cf965b28c811bbe85219b6add54528d9c`).
No existing run is retagged or supplied with manufactured scores. Independent
read-only diagnosis, SHA256
`c7507d353046968acb0f493bf1bd8ade8b3c671e608ff4d3b396794f1f4e3614`,
confirms all29,120 identities and the2,000 declared altered samples match actual
delivery. The existing scorer, used only in memory, gives scale1 ATE12,767.29m
and one-second RPE364.308m: severe estimator divergence remains. Fixing the host
contract cannot make that an estimator success.

The host correction now preserves the full original ledger for sample ordinals
and gaps, and separately verifies delivered payloads against the sealed derived
manifest, declaration and parent-source ledger. Declared artificial provenance
stays explicit; incomplete proofs, altered values, unknown identities, duplicate
or unordered samples and nonfinite values are rejected. The legacy source-only
contract is unchanged. State-projection failure remains a primary FAIL while
the existing raw/adapter numerical scorer may still produce diagnostic output;
partial propagation is never marked complete.

Current software regression is **1,054 passed / 1 skipped** (19.34s); affected
checks are325 passed. The26 new cases changed from21 failures/five passes before
the correction to26 passes. Three owned test process groups exited and37
frozen code/fixture hashes stayed unchanged. Contract receipt SHA256:
`ca6545836451fc1bb0b4684dc2a576394284cb91b4a089c1a64aa2522e328e84`.
Independent cumulative review passes, SHA256
`2a0a09c50e36be74329cfeeec7e5ab11a9a132c1dea5cbafa5c9ad366d97b463`.
Only the sensor-contracts receipt fields are explicitly migrated in the sole
matrix, SHA256 `54726fb6b1528c90bd84e6c0cf509061432ea21d67a879145fefe8b184998ef3`;
all75 gates/67 required cohorts and other74 gates remain exact. The fresh1x bias
replay has now completed: all39 sealed outputs and four zero child exits/owned
absence pass the existing integrity audit (SHA256
`af6c89253b2c2ffd43eb3e95f099460a1fbdba9a60cab2cd7c5a657ee5f662eb`).
The run has2912 numerical poses but severe scale1 ATE12,814.157646m and one-second
RPE365.499199m /2.436708deg. The selected result remains PARTIAL, one PASS and
75 NOT_RUN; strong recovery remains NOT_RUN. State/propagation interface checks
complete without errors, which does not validate estimator accuracy or health.
Independent identity/health audit passes (SHA256
`685ded7d6b6df957c526412b0bbfa3c1f34b2c46b3470f0fb5fb9f7faee8bae5`):
all29,120 sample identities and2,000 declared changes match. Camera473 at
23.650000128s has31 inliers below40 and marks loss; subsequent outputs remain
invalid and no recovery occurs. Last GOOD visual time remains camera472 even
when later bad-quality constraints are accepted. Twelve zero-integration
IMU-callback outputs reuse an existing camera204 snapshot without changing
visual freshness; they are not new visual observations. The first audit-only
FAIL from an overstrong source-label assumption is preserved. Original failed
evidence remains unchanged. This is PC offline/interface evidence, not onboard
qualification. The preceding1028/one-skipped readiness receipt remains preserved
as `83dc8173c5bc621c179b1f35fa92604e7577d8d931351249e62dd5b6db010a66`.
Its4cc matrix completed a selected evidence/input-rejection readback:
14 PASS, 0 FAIL, 62 NOT_RUN, overall PARTIAL (result SHA256
`f26cd11f7f5dae70d851a9823797bef3de7dbd33a1c1f8da5672eeb957ec9465`).
This rechecks existing ARM64/MOCK/SITL receipts and that version's contracts; it does
not rerun those experiments or establish complete matrix acceptance.

The fresh current-host ARM64 full replay passes with the existing locked
AArch64 executable, Bionic/Melodic reference rootfs, full V101 stereo input,
0.5x replay and4GiB cap:2912 poses, four zero child exits and owned absence,
peak584,777,728bytes and zero OOM. Fixed-scale ATE is0.065721042m;
one-second RPE is0.043459242m /0.457553897deg over2871 scored poses. Receipt SHA256
`2d5027e4a4f611a00cb72987f762d0307aebbcab56f9ff3edbd1b471b842ac6c`.
Only two host source identities changed; four fresh configs alter only run
ID/output. Timeout/SIGINT/native-child cleanup also actually passes: expected
negative estimator runs remain FAILED, SIGINT per-child exits remain UNKNOWN,
and the injected native child exits137 with controller1. Independent group review passes (83 checks; SHA256
`2281a930a249248ebc952d4a82f0420228940283f286a72bbc152f1b7bfd9c6f`),
including189 receipt artifacts,151 exact run outputs, current code identities,
unchanged source/rootfs/AArch64 dependencies and exact owned process/container
absence. SIGINT terminal cgroup/OOM and per-child exits remain UNKNOWN; its
last partial resource row is not treated as complete. The four receipt paths/
hashes are now explicitly migrated, with75 gates/67 required cohorts and all
protocols unchanged. Current matrix SHA256
`41a6bcb26aea9510790b8264a96255ab2e5d8267caf91d1fc07adcf55425d491`;
migration SHA256 `48487a84cf361f897fae991f37a3840a9ffe3ad34e24ef5230eb65c19a727546`.
Fresh selected evidence plus six input rejections completes14 PASS,0 FAIL,
62 NOT_RUN, overall PARTIAL (result SHA256 `17bea583b11480ecdfb9fa7da46ae2c19efa5a0bbe3f75ea430f958902706a86`).
This rechecks evidence currency; it is not a full matrix execution.
This reference target does not establish RK3576 BSP, ABI or performance.

The three new1x camera-gap replays complete without process failure:500ms/
2s/5s remove10/40/100 stereo pairs while retaining the IMU stream. Numerical
pose counts are2902/2872/2812, fixed-scale ATE0.07948346/0.17796154/0.54850853m,
and one-second translational RPE0.04323229/0.04521490/0.30586353m. All117
sealed outputs, source/input identities and raw/adapter scores pass audit;
all12 child exits are zero with owned absence. Each passes the configured
two-GOOD-correction PC offline recovery gate in the same native epoch. The5s
group immediately falls back to bad quality after its first qualifying pair;
that gate does not establish sustained recovery. Watchdog loss is bounded
only in its recorded receipt clock, without a sensor-clock mapping. Independent full-run health-episode audit passes135 integrity checks
(SHA256 `8e56984cf8e526de9e7d20cb648dca0d2581b703ec4d07046bc9af1f7c680043`).
Post-fault bad-quality visual counts are4/5/27; all final frames are GOOD and
usably projected, but the diagnostic proposition of all-GOOD states after
the first qualifying pair fails for each. Complete episodes remain recorded;
this does not change the original configured gate or establish onboard safety.

The existing summarizer now adds observed health episodes without changing the
original two-GOOD recovery rule, run status, metrics or fixed-repeat groups.
Source and qualified receipt clocks remain separate; interleaved watchdog ticks
share projected snapshot epochs and have no fabricated sensor time or native
segment. Native epochs require exact native evidence and are never forward-filled.
Actual regression passes1078 tests with1 skipped; targeted24 and affected322
also pass. Read-only regression over12 sealed runs preserves all previous summary
fields and both five-member group objects. The three gap experiments have4/5/27
NOT_GOOD samples from fault end inclusive; after the second qualifying recovery
update exclusive, they have3/3/7 samples in3/3/5 episodes. Final GOOD Info can
coexist with a final stopped-stream invalid watchdog. These are observations,
not continuous availability, estimator acceptance or flight qualification.
Contract receipt SHA256 `e6bb45f79ccaa27ca30d470da11252123ab751e176abb2ef5e8c0b3fae830958`;
sealed readback SHA256 `5aab70f6e67f8af3cc088a72c5e3d3882e0c178ee4eaa446e5a80b35a4913213`.
Independent cumulative review passes(SHA256
`cffbb19b3fe6b463a1f9e155a72e446f96472ad428a23cded8a48ff1f3660930`).
Only the sensor-contracts receipt path/hash is explicitly migrated; all75 gates,
67 required cohorts, native config locks and protocols remain unchanged.
Matrix SHA256 `f8d6a9d604cfa11630529c6d921ee333191a89f71f79763c5036a8d256f51b25`;
migration SHA256 `d650366bde54e956ad15de63b5c753facc8046b68755651380844e2d1c41a612`.
Fresh evidence and six rejection checks complete14 PASS,0 FAIL and62 NOT_RUN,
with overall PARTIAL and execution_complete=true(result SHA256
`7e4539307872cbbdcffd601afc7ed2c1c0e594178ecbdb2a74198a7b0f4ad6ae`).
This selected invocation does not rerun native datasets or prove all required
cohorts; snapshots are not combined into an overall PASS.

A source/paper review narrows the next mono candidate. The upstream camera
Schur elimination uses raw visual V^-1; independent landmark covariance and
reused window observations are intended approximations. The current mono
finite-inwindow candidate instead uses(V+C^-1)^-1 in camera elimination. An
isolated, NOT_PROMOTED child is authorized to restore the upstream camera
equation and compute the finite conditional inverse separately at the point
update, keeping existing ownership, correction order, initialization and gates.
Fixed-snapshot199/200 comparisons establish changed information only, not a
tracking improvement or unique loss cause. Independent source/production-fixture review now passes (SHA256
`fb29c3273c22428d19225a2b08acdb7dfaad18746e41c5cc3e1c9fb14a8441b0`):
12 actual parent/child math processes, independent SVD/nullspace and Joseph
references, stereo equivalence, six lifecycle controls,25 packages,53 ELF
paths and five fixed full-sequence configs. The full V101 mono replay has
completed FAIL / TRACKING_LOST:8 of2912 source frames,0.4s output span and
fixed-scale ATE0.01716803157m; one-second RPE is unavailable. All four child
exits are zero,39 sealed outputs and raw/adapter scores pass the integrity audit
(SHA256 `1a68c6cccf0d184795e18d418dc4321ca8b9ab12a561424d27cc95150a75ae6d`).
No five-repeat cohort is started for this failed smoke. A different bootstrap
prevents a controlled tracking comparison with the parent's28 poses.

The same-run bounded dynamic audit selects the actual first loss156→157,
not the parent's199→200 pair. It captures145 point posteriors across two
complete matrix snapshots; independent raw-camera QR and finite-point Joseph
checks satisfy the original1e-8 bound. The original compressed-posterior decoder
still fails covariance checks at2.173e-7 and1.017e-7; these failures are preserved,
not replaced by the different reference's PASS. At157 the inlier count is already
6 before the camera update and rises18→33 through correction, below the unchanged40.
Receipt SHA256 `1e18373c2f762672fa68ad728b96e69acb8d710922806928c81e2adb18cce05b`
establishes bounded algebraic evidence, not a unique tracking-loss cause.
The full V202 mono0.5x replay also completes FAIL / TRAJECTORY_NOT_EVALUABLE:
only one pose at camera238 after native metric commit223, followed by loss239.
All2348 image callbacks,41 sealed outputs, four zero child exits and owned absence
pass the integrity audit (SHA256 `dfae5e9742ebba810a980186cd11c51bce7e9a8584e69e301e2b998063ef8b1e`).
ATE/RPE are unavailable; the one pose is not a valid scored trajectory. V103
mono also completes FAIL / TRACKING_LOST:two poses at original frames439–440,
0.05s in epoch4, scale1 ATE0.003710475704m and no one-second RPE. That tiny
scored span is not continuous tracking success. Its four child exits are zero,
41 sealed outputs pass audit (SHA256 `27d521901993f8090e35758b73a5f26f038d19a1bccc0c97202defad702a40ec`).
Observer images are complete2149/2149 and native IMU ledger21500/21500; the
Info stream lacks original frame245 despite contiguous topic sequence. The442
paired frontend spans do not establish complete native image ingestion. V101
stereo completes PC replay/numerical parity PASS:2912 poses, scale1 ATE
0.0557723939589m and one-second RPE0.0433937397720m/0.460472048097deg. Its42
sealed outputs, four zero child exits and owned absence pass generic integrity
review (SHA256 `8294388a64ad8a5defa231467ce8721bb785c07f34a6ae81e462fedcc9b04b08`).
V103 stereo also completes numerical parity PASS:2149 poses, ATE0.09798359424m
and one-second RPE0.03698280871m/0.4090535225deg. Its42 sealed outputs and four
zero child exits with owned absence pass generic review (SHA256
`12dea99f020b6bd321099972cce5a4192956ef5abf2703a3ed207631abe640ff`).
Both successful stereo smokes now have predeclared five-member repeat cohorts:
ten read-only configs change only run ID/output, under manifest SHA256
`a44afd77aa4ce822b63f7f5aa306ea7f4f4f6496cb9b819e9e33f9f8767defe0`.
All ten fixed repeats have completed. V101 has five numerical PASS runs with
2912 poses each; its1cm ATE-difference reference is exceeded in6 of10pairs,
with ATE0.0444116502–0.0890782420m. Generic integrity review passes(SHA256
`1f57073450b6e01c8c65a039aed34149f851e8a130a83ba225f15faeaa7c4bc1`).
Independent scoped review(SHA256
`db7b4771f1daedd4c274a2ad0407a9b84ec62c4932777572af3c99e29fab3ae9`)
verifies210 sealed outputs,80 runtime references,37 shared identities,20 zero
child exits and five exact owned absences. The original independent receipt
`cf063d27a333ea66b10149b08fb055820020ab508129d9cd2ad128b6f13f01b6`
remains FAIL:its checker compared complete alignment dictionaries with a compact
two-field dictionary; the supplement verifies actual SE3/scale1 fields.
V103 also has five numerical PASS runs,2149 poses each, ATE0.04098443719–
0.07009940040m and one-second RPE0.03430227632–0.03645778568m. Its1cm reference
is exceeded in4 of10pairs. Independent170-check review verifies210 outputs,
80 runtime references,37 shared identities,20 zero child exits and five owned
absences(SHA256 `1ff21cd3b429d266fd656daeb705541815d3c9c1d381d4d54320e9b172c4468c`).
The final existing-tool summary is sealed under SHA256
`a322b4cc5ac20ed2c543dd85c73fb1733be7863fb6dcd2c2d1e106c699b3c540`.
These Schur pair differences remain explicit diagnostic/reference results;
they do not add a new matrix gate. The original mandatory OpenVINS1cm gate
remains unchanged. All mono failures remain; this candidate is NOT_PROMOTED.
Independent closed-smoke integrity review passes all205 sealed outputs,80 runtime
references,37 shared identities,20 actual zero child exits and five exact owned
container absences (SHA256 `6260d5ea8781d3cff6f836c7ec91370aff0e8adfb4240c01c9f4ebf45f69e0c7`).
Its original review SHA256 `c481f6721cff83ff63cafd74302b3205533cf932c3052d8444c5396c5870df89`
remains FAIL:the sole failed assertion required all future repeat outputs to be
absent after repeat1 had already started. The scoped supplement rechecks only
closed smokes and makes no active-repeat freshness or repeatability claim.
Mono bootstrap uncertainty, continuous tracking and overall acceptance remain
unqualified. The candidate is NOT_PROMOTED.

A bounded same-run156→157 geometry readback excludes a new seed scale or point
handoff explanation for its inspected population. All54 points at157 exist at
metric commit154;43 shared point means carry exactly, with no promotions in
either frame. The independent IMU integration matches the native prediction.
At156 the70→3→65 inlier sequence already exposes geometry tension:for37 actual
current factors the camera-only linear residual mean8.507px agrees with the
nonlinear8.575px, rather than predicting an improvement reversed by the pose
chart. The raw free-landmark back-substitution would reduce these residuals to
0.1199px but requires mean1.054m point motion; the actual finite-point stage
moves0.04381m on average and leaves3.067px. This measures disagreement between
the two stages' implied changes; it does not establish the sole loss cause or
qualify a replacement estimator. The existing full snapshot/geometry evidence
is reused without new production code, truth fitting or acceptance changes.
The complete156 snapshot supplies a further, reproducible counterfactual using
all73 points and219 factors(current73, retained146), the same P/C/J/r and frozen
Huber weights. The actual raw-camera/finite-point pair increases weighted visual
cost106.752484→318.017701; a consistent finite-prior joint mean gives1.880792.
Including both declared prior penalties gives345.707913 versus2.058906. Its
438-dimensional independent measurement solve agrees with finite elimination
within the original1e-8 bound, with singular P preserved through exact augmentation
support. Root replay reproduces149 numeric scalars within that bound and retains
all15 input/source/reader identities and37 shared identities. Receipt SHA256
`0227cce00ca4da0ab5aaf07e5c1919a6537762b37dadfa8e38f35fd66eec3385`
contains the exact replay program. This is the existing finite-camera design's
fixed-snapshot counterfactual under zero-cross-prior assumptions, not a runtime
repair or a qualified bootstrap joint covariance. No core change is made from
this diagnostic alone.
A separate high-precision readback retains the compressed covariance FAIL but
finds no material one-step tracking effect:actual covariance diagonals remain
positive, near-zero eigenvalues are floating-point scale, and checked inlier
counts do not change. Singular clone priors must not be inverted as ordinary
full-rank matrices.

A read-only matrix inventory finds seven missing current OpenVINS readiness
cohorts:V101 mono/stereo at0.5x and1x,V103 mono/stereo at0.5x, and the0.5x500ms
camera gap. The two V1031x six-member cohorts already match their declared
installation and are retained. All19 legal Schur fault conditions have1x runs;
15 current-installation0.5x conditions remain unexecuted. Existing historical
results are not relabelled with new host hashes. The same matrix and runners
will execute the missing conditions serially; the overall verdict stays PARTIAL.

Four fixed-map camera-fit diagnostics used all observed correspondences in
frames 206/207, before and after the native update. The 207 mature 4px counts
could increase from 23/30 to 54/55, while its original selected 40-point subset
remained 29/29 and only one of seven rejected points recovered. This supports
a shared camera-error component and a remaining incompatible tail; it does not
justify writing the fitted pose into the estimator. No production estimator,
truth, scale, map or acceptance threshold was changed. Independent review of
this bounded diagnostic passed (SHA256 `057598951210ce6fb9f9b02c7036d2274c9dfc5f7c7e35255f750b7398b6da83`).
The estimator replay remains FAIL / TRACKING_LOST, candidate NOT_PROMOTED.

A further bounded readback verifies the exact first-loss207 joint finite
objective: five clones, 52 points, 197 factors (40 current and 157 retained), and
958 matrix chunks. All 52 actual point posteriors, including point1027 from the
same run's complete geometry, satisfy the unchanged 1e-8 algebraic bound. The
full linear objective including declared camera and point priors decreases
136.340014 → 133.324872 → 117.609565; native inliers remain23 →24 →30, below40.
Independent receipt SHA256
`243396f4fdb3544e3e53e751fe549fd15fd204abbe3db9fa8d4a329b276db945`
confirms this local algebraic agreement; it does not qualify the statistical
priors, correspondences, metric scale or continued tracking. The old truncated
258→256 observation window remains FAIL. Matching 144 historical observation
identities does not prove actual206 factor participation or double counting
without the missing complete206 matrix. No estimator or threshold was changed.

One additional supported conditional diagnostic constrains only the current
clone to the already saved207phase0 image-fit pose, using the exact native
quaternion chart. An independent207-variable covariance-form solve passes3080
checks at maximum relative discrepancy5.94e-13, below the unchanged1e-8 bound
(SHA256 `891285802d57979db689a2ea26c4591b903ad551052abb211b9c0cb7f7518f57`).
Full cost increases117.609565→2844.416123: camera prior contributes2540.495032
(93.167409% of the increase), retained157 factors102.059002, point prior76.307472,
and current40 factors7.945053. Linear4px counts change29→36 current and155→146
retained; the112-observation nonlinear image fit has a different scope. This
quantifies a conflict with saved information without proving its physical
validity or a sole loss cause. Under the assumed independent-prior model, all52
conditional-versus-joint point covariance formulas pass; nonzero omitted camera-
point and interpoint terms are observed, not qualified physical covariance.
An isolated two-file observation child is being implemented to capture the
immediate previous same-epoch good update at first loss. It preserves the old
four-snapshot/two-epoch protocol and estimator mathematics; its separately named
extension is capped at one global extra witness and32MiB serialized output.
No new native result or estimator repair is claimed for this in-progress child.

## Source identity and system boundary

The inspected SchurVINS source is
[`d8ab6dff20860694d8d879d716d7fd6cb2c2530e`](https://github.com/bytedance/SchurVINS/tree/d8ab6dff20860694d8d879d716d7fd6cb2c2530e).
Its README states that it derives from SVO2.0 (`rpg_svo_pro_open`), and names
Ubuntu 18.04/Melodic as the tested route. Reuse the complete SchurVINS tree;
there is no additional SVO2.0 fusion layer to implement. Upstream describes
run variation due to threading and provides sequence-specific start skips.
Those skips must be explicitly labeled supplemental experiments; the primary
RM27 comparison uses the complete sequence at 0.5x playback.

`tools/schurvins/platform.patch` disables the loop-closing compile definition
and removes the disabled loop/global-map dependencies from the active VIO
packages. Build the `svo_ros` dependency closure, not every package in the
upstream workspace. The upstream `svo_online_loopclosing`, `svo_global_map`
and pose-graph packages remain third-party source artifacts; they are not
RM27 development or enabled fallback capabilities. Runtime must also force
`runlc=false`, `use_global_map=false` and `use_ceres_backend=false` after
loading upstream parameters. The original mono launch and `vio_mono.yaml`
enable loop closing, so launching them unchanged is outside the VIO boundary.

The patch keeps the original x86 compiler flags and numerical optimization
settings. It changes architecture selection for AArch64 and disables excluded
modules; it does not change Schur updates, initialization, noise or scale.
Every experiment must record the applied patch SHA256, source/dependency tree
identities, image ID, compiler/package versions and executable/library hashes.
Unresolved dependency branches or submodule references are not a completed lock.

## Verified code-level input and output semantics

These findings are inspection of the pinned source, not evidence of a completed
native run:

| Boundary | Pinned implementation | RM27 treatment |
|---|---|---|
| Estimator selection | `svo/include/svo/global.h` defines `USE_SCHUR_VINS`; shared `sparseImageAlignment()` and `optimizePose()` call Schur Forward/Backward | Confirm compiled definition and actual processing before calling an execution SchurVINS reproduction |
| Mono | `FrameHandlerMono::processFirstFrame()` performs geometric initialization using an initial depth prior; shared Schur update path is used later | Real mono metric behavior and initialization remain runtime acceptance items; `map_scale=1.5` is a configured prior, not a measured metric scale certificate |
| Stereo | ExactTime ROS image synchronization, common timestamp passed to both images | Require exact timestamp pairing; do not silently retime or approximate-pair camera inputs |
| IMU | Separate callback thread, queue depth 10000; image path waits briefly for an IMU horizon and gets edge-containing measurements | Preserve ordered raw SI samples and source clocks; publisher/observer counts do not prove internal ingestion |
| Pose topic | Private `pose_imu`, normally `/svo/pose_imu`; `Visualizer::kWorldFrame` is `world` | Keep frame identity; convert only after verifying this convention |
| Pose geometry | `get_T_W_B()` passed directly as `T_world_imu`, Eigen Hamilton quaternion fields xyzw | Maps IMU/body coordinates into world; reorder xyzw to Hamilton wxyz without conjugation |
| Pose stamp | Input camera timestamp converted with `ros::Time().fromNSec()` | Integer camera-associated state stamp; do not call it an independent high-rate IMU propagation time |
| Pose publication gate | `publishResults()` emits pose in `Stage::kTracking` | Source-supported native gate, not automatic qualification of RM27 operational tracking or validity |
| Covariance | Wrapper constructs an all-zero 6x6 matrix | Preserve raw zeros; uncertainty remains unqualified rather than noiseless |
| Info topic | Private `info`, normally `/svo/info`; stage, tracking_quality, match count, processing_time | Retain raw fields. Timer is frontend processing, not sensor-to-publish latency |
| Native trajectory | Online `imu_traj.txt`, decimal seconds obtained through double and 10-place text formatting | Treat as rounded timestamp evidence; use integer ROS headers for exact association and preserve text separately |
| Trace/output paths | Trajectory directory is `parent(parent(trace_dir))/logs`; filename is fixed | Place trace path under fresh run output, create its corresponding logs directory, never write into read-only source |
| Reset | Native stages exist, but Info has no reset counter. Mono frontend reset does not recreate the Schur filter object | Adapter run epoch is not a native reset count; do not certify full reset/recovery from a stage transition |
| IMU propagation | Schur prediction occurs within frame-driven Forward; reviewed ROS wrapper publishes visual-state poses | No separately qualified high-rate propagation output is exposed by this wrapper |

Raw stage enum values are 0 paused, 1 initializing, 2 tracking, 3
relocalization. Tracking quality is 0 insufficient, 1 bad, 2 good. Preserve these
source values and their commit provenance. First adapter output conservatively
keeps standard tracking/initialization unknown and operational validity false;
numerical trajectory evaluation is a separate question. Local SVO tracking
recovery behavior is not a new global-map relocalization capability.

A source inconsistency exists in `SchurVINS::InitExtrinsic`: the stereo branch
assigns camera 1 to `cam0_param`, leaving `cam1_param` unset. Those camera pointers
are not read elsewhere in the inspected `.cpp`; this is not an established cause
of a runtime failure and no core repair is included. The reviewed Schur
observation code selects extrinsics by feature camera ID. Mono/stereo acceptance
must be based on actual executions, not guesses from this inconsistency.

The supplied EuRoC mono calibration includes `imu_rate=800` and zero configured
camera/IMU delay. These are upstream parameters, not measurements of the dataset
or the new hardware. Record both actual dataset cadence and configured parameters;
do not silently edit them to improve a score. Camera extrinsics use `T_B_C`,
which agrees with RM27 `T_imu_camera` direction after verifying the body/IMU
identity; never relabel an inverse transform.

The V101/V202/V103 source IMU calibrations were independently checked against
all three original archives and prepared copies, SHA256
`fddff7e72b241cf16fcc11d65e0f09e58c933e2eb6606d22ec446d6443c679ae`.
They share the following values; the native upstream mono file differs:

| Source quantity | EuRoC imu0/sensor.yaml | Native upstream mono YAML |
| --- | ---: | ---: |
| Rate (Hz) |200|800|
| Gyroscope noise density |0.00016968|0.0012|
| Gyroscope random walk |0.000019393|0.03|
| Accelerometer noise density |0.002|0.008|
| Accelerometer random walk |0.003|0.1|

The four source and native field units match. The loader passes continuous
noise parameters directly to Schur's model, which squares them and uses the
actual sample dt. The resulting mono/source input variance ratios are about
50.0151,16,2,393,054.087 and1,111.111 respectively, not estimator-error ratios.
The800Hz declaration does not set the current Schur integration step: its
stationary-check use is disabled and the Ceres route is excluded. Mono alignment
does not use these noise weights. An isolated five-scalar source-calibration
control has built all 25 packages. Its 527 devel files and node are byte-identical
to the terminal-observer parent; camera, extrinsics, time, gravity, initializer,
solver and acceptance thresholds are unchanged. Actual linked loader/model/
Prediction tests pass in normal, Release and ASan/UBSan fixtures. They check two
actual integration steps, one squaring of continuous noise, and no change from
nominal rate alone. The same five recorded alignment windows produce identical
results. Shared Release libraries are not newly sanitizer-instrumented.
Root independently checked archives, exact five-scalar diff, patch ancestry,
source/devel identities, fixture artifacts, three full-sequence 0.5x preflights
and 52 ELF dependency closures, receipt SHA256
`f8082d7bfc9ade8a84dee08730fb832e0b4515f7de9c912adf8478f8585c2144`.
Installation `build-mono-euroc-source-imu-calibration/installation.json` has SHA256
`9ffa36100c37d7e12b97694f1544ba8850637c51468c26ec03a70f941ecba72d`.
The three native controls completed serially in root session88326. V101 still
**FAILS / TRACKING_LOST**:26 of2,912 poses (0.8929%), scale1 ATE0.0792919 m
over only1.25 seconds and one-second RPE0.2130621 m /0.649594 degrees.
Four children exit0 with owned absence. Full input/source/output and raw/adapter
score integrity passes, receipt SHA256
`c274bad20addff75e98ded72de4f21fa44865f91743a7fd5dcbdd54feb8c31d7`;
run SHA256 `b3df801ac86e324dd672258f011a4e74a47a42dd6313a0888c86bc84f1e1db6d`.
The short scored segment does not satisfy full-sequence mono acceptance, and
the failed smoke is not followed by five success repeats. V202 full and V103
also **FAIL / TRACKING_LOST**. Their complete input/source/output/score integrity
passes, receipt SHA256
`c855a9ce1087f49a519e8a7cd36fceb5aee230a8e1ab2a550f0c13070f00c4a3`.

| Source-calibrated mono control | Poses / source frames | Scored span | Scale1 ATE | 1s RPE translation / rotation |
| --- | ---: | ---: | ---: | --- |
| V101 |26 /2912|1.25 s|0.0792919 m|0.2130621 m /0.649594 deg|
| V202 full |20 /2348|1.00 s|0.0971324 m|0.3328596 m /0.994919 deg|
| V103 |16 /2149|0.80 s|0.0544952 m|NOT_RUN: shorter than1 s|

All12 child exits are0 with exact owned absence. V103's scored epoch is4;
earlier epochs without poses are not concatenated or omitted from source
coverage. Native Info omits4/66/47 original camera inputs respectively.
At all three first failures the original minimum40 gate fails; observed camera
and point updates increase the current inlier count but leave it below40.
This does not isolate accumulated state/map error or bootstrap uncertainty.
The compact sealed-control observation has SHA256
`1d9d37d4e614d7d5b649a7a909462946c756b779a32ba065356f6150f787dc37`.
Different geometric bootstraps prevent causal accuracy claims across these
runs. The original upstream
configuration and failed runs remain intact. This difference is a testable
configuration hypothesis, not an established unique cause of mono failure.

Source references: [`svo_interface.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo_ros/src/svo_interface.cpp),
[`visualizer.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo_ros/src/visualizer.cpp),
[`frame_handler_base.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo/src/frame_handler_base.cpp),
[`schur_vins.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo/src/schur_vins.cpp).

## ARM64 and RK3576 readiness

The isolated ownership candidate now has a fresh **ARM64 BUILD VERIFIED**
reference build:25 packages,4 CPUs/4 GiB,59m29.5s, exit0 and owned absence.
Installation `arm64/build-mono-promoted-observation-ownership/installation.json`
has SHA256 `dcf6b1e1650c9512902e75284dc1b066d97d88ec60f3a4fec5f41a3c90479c96`.
It binds PC ownership installation1dff9ea9 and only the seven qualified ARM
platform differences; it does not include the later terminal observer,
source-IMU calibration control or precise bootstrap solver.
Actual target Normal/Release ownership tests each pass13 cases. The audit
checks339 AArch64 objects,52 ELF paths,238 loader dependencies and NEON
correctness, receipt SHA256
`82d0ec3462444575cfd7af88db84fbc46c4a275f8abcab07c1e2b74ae17fd337`.
The empty combined sanitizer control fails with leak detection enabled both
under PRoot and under bounded direct QEMU without PRoot. The direct target
loader/libraries check passes, but LSan still exits1; receipt SHA256
`8eb8e815ffd6a2ad5cca05ea9920ab912cd3ff734a15ca581974b10775e24424`.
The blocker is specific to the locked QEMU/Bionic combination; target ownership
ASan/leak qualification remains BLOCKED/NOT_RUN. A separate direct-QEMU
UBSan-only check passes the same13 ownership cases, an empty control and
baseline/UBSan Eigen ABI comparison. A deliberate signed-overflow control
exits1 with the expected detector error. Receipt SHA256
`78001b64c448ccc87c7861b47d1d6cfc49ef12d854d90e2bad086ee53bae2f2a`
checks202 AArch64 resolved libraries and19 owned container exits. Only the
fixture and five exact repair method bodies are instrumented; the remaining
shared libraries remain Release. This is not production-wide memory or leak
qualification. No disabled leak check is counted as a pass.
The existing package tool verifies base-plus-overlay restoration for39 changed
paths, zero removed paths and exact source/devel/symlink identities. The overlay
is8,323,884 bytes, SHA256
`62e84aa1522d42a617925342fe5d2db47b11ee35d2e10414883695478e1ff69e`;
final closure SHA256
`4fe426ab13fe8f63199b14a7301a3de8042f8fe6a7884922497c9742e9979e3f`.
The new identity has not run a dataset. It remains NOT_PROMOTED and establishes
no CV00/MV12 BSP, target ABI, board performance or mono tracking qualification.

The original `SvoSetup.cmake` appends `-march=native` and either x86 SSE flags
or the ARMv7-only `-mfpu=neon -march=armv7-a`. Its environment-variable switch
does not express an AArch64 target. The integration patch selects flags from
`CMAKE_SYSTEM_PROCESSOR`: x86 retains original flags, AArch64 uses
`-march=armv8-a+simd`, ARMv7 keeps its existing branch, unsupported architectures
fail explicitly. AArch64 enables `HAVE_FAST_NEON`; the unselected scalar FAST
branch contains an undeclared `L` index and is not assumed usable.

The reference ARM64 environment is now actually provisioned in an isolated
Ubuntu 18.04 / ROS Melodic rootfs. Its separate signed-apt lock covers 1,117 ARM64
packages. A scan of 2,902 regular ELF files found only ELF64 little-endian
AArch64 files and no x86 contamination. This is a reference distribution, not
the RK3576 BSP or an ABI qualification for an unknown carrier image.

Local vendor references were inspected on November 1, 2026 through the parent
checkout's `docs/REAL_HARDWARE.md` selection. Only **LPDDR5 CV00 / MV12** is
selected; the vendor revision note says LPDDR4 CV02 / MV01 connector ordering
differs. These references are local untracked parent-repository inputs, not
files supplied by a standalone VIO clone or proof of the purchased revision.

| Selected local reference | Relevant facts and deployment limit |
| --- | --- |
| `RK3576/01_硬件手册/硬件手册_新版（LPDDR5_CV00)/01_产品规格书/HZ-CORE-RK3576-LPD5-规格书V1.0.pdf` | Pages 9–11: 4 GB LPDDR5 / **64 GB** eMMC, A72/A53, BTB and CSI resources. Preserve the user's **32 GB deployment budget**; actual SKU is unverified. The core-board document explicitly does not support DPHY 2+2 lane splitting. Page 15 leaves working current TBD. |
| Same selected revision, `02_产品硬件说明书/HZ-EVM-RK3576-硬件设计指南V1.0.pdf` | Pages 41–42 describe SoC camera interfaces, including 2+2 splitting; this does not override the stricter core-board restriction. Pages 89–91 require controller-consistent IOMUX groups and appropriate IO domains. No final IMU wiring is selected. |
| `RK3576/03_文档教程/1、开发板使用手册/HZ-EVM-RK3576-Linux开发环境搭建V1.0.pdf` | Pages 14 and 45–47 describe an **amd64 Ubuntu 20.04 development host**, Linaro GCC 6.3.1 kernel/U-Boot examples and Buildroot GCC 11.3.0 with an RK3588 example path. They do not establish the RK3576 userspace, kernel release, compiler ABI or sysroot. |
| Same tutorial directory, `HZ-EVM-RK3576-测试手册V1.1.0(1).pdf` | Pages 21–23 describe evaluation-board IMX415 / GC05A2 and ISP virtual video-node examples. Enumerate devices after actual BSP/carrier binding; never hard-code `/dev/video33/42/51` or infer exposure timestamps, synchronization or VIO camera support from this example. |

The local `02_镜像及源码/` contains only a 73-byte README directing customers to
obtain the SDK through their vendor contact; there is **no local BSP, image or
sysroot** to inspect. This is a concrete missing software input, independent of
physical board availability. Obtain the selected CV00/MV12 SDK/image and its
hashes before checking target loader, libc/libstdc++ symbol versions, ROS
dependencies and sensor device-tree/driver configuration. No vendor contact or
hardware operation was performed. The existing reference uses arm64 libc
`2.27-3ubuntu1.6`, libstdc++ `8.4.0-1ubuntu1~18.04`, OpenCV
`3.2.0+dfsg-4ubuntu0.1` and ROS Melodic; compatibility with a different BSP is
**NOT_RUN**, not implied by AArch64 ELF success. Source hashes and page-specific
findings are in the bounded local `rk3576-reference-audit.json` task receipt.

`tools/schurvins/arm64.py` prepares the rootfs, explicit PRoot/QEMU helpers,
independent source workspace and target manifests. Bionic QEMU 2.11 works with
the selected PRoot; the newer host QEMU/PRoot incompatibility remains retained
negative evidence. The ARM-only FAST patch fixes both architecture selection
and `__ARM_NEON`/`__ARM_NEON__` guards without changing the frozen PC source.
Actual AArch64 and separately cross-compiled FAST/NEON binaries agreed on all
18 test cases / 12,466 corners. This verifies those target code paths, not the
entire estimator. The full target build has completed: **26 packages succeeded**,
with zero failures. The 46 devel ELF artifacts are AArch64, and the target loader
resolves 238 AArch64 libraries. Actual AArch64 and cross-compiled ZMSSD also
agree across 774 cases. Evidence is under the external root's `arm64/` directory,
including `build-clean/installation-runtime.json`, `devel-elf-scan.json`,
`rootfs-elf-scan.json` and `cross/neon-sysroot/`. This is **ARM64 BUILD VERIFIED**.

The fixed Bionic GCC 7.5 cross toolchain and a target sysroot are now available;
missing tools are no longer an external blocker. The 683 MiB deployment archive
has been extracted and verified against all frozen file/link hashes, then used
for isolated target Python/OpenCV/NumPy imports and roscore startup/SIGINT exit.
Only that owned temporary extraction was removed afterward; the archive remains.
These deployment-startup checks are **EMULATED / MOCK VERIFIED**. The PC amd64
image digest is never used as the ARM64 rootfs identity.

The complete V1_01 stereo 0.5x replay at 12 GiB passed input reconciliation,
all four process exits, container cleanup and raw/adapter metric equality:
2,912 poses, 2,871 GT-associated poses, fixed-scale SE(3) ATE RMSE 0.052639 m.
This is functional QEMU evidence, with no board-performance interpretation.
The original 4 GiB full run failed with recorded cgroup OOM: player and observer
were killed. Isolated target probes reproduced excessive memory growth from
32 MiB TCP receive allocations; 64 KiB fragmented TCPROS reads preserved all
message bytes and timestamps. A complete corrected 4 GiB execution retained
all inputs and exited cleanly with 587,145,216 bytes cgroup peak and no OOM,
but its formal result remains **FAIL / HASH_MISMATCH** because a host dataset
source changed during the run. That failure remains preserved.

Fresh frozen-source full replays then both passed:

| QEMU reference cap | Poses / GT-associated | ATE RMSE, fixed scale | Whole-cgroup peak | Native max RSS |
|---|---:|---:|---:|---:|
| 4 GiB | 2,912 / 2,871 | 0.041674 m | 581,947,392 bytes | 135,692 KiB |
| 12 GiB | 2,912 / 2,871 | 0.055885 m | 580,710,400 bytes | 135,520 KiB |

Both retained exact 2,912 cam0 / 2,912 cam1 / 29,120 IMU input equality, all four
process exits 0, container absence, raw/adapter metric parity, unchanged source
hashes and zero OOM events. `arm64/bounded-recv-resource-comparison.json` records
their same-code identities. Single-run accuracy variation is not a memory-cap
ranking. Subsequent necessary watchdog/observation-source changes retain these
as exact-version results. A current-source 4 GiB full replay also passed with
2,912 poses, 2,871 GT associations, ATE 0.0437255 m, whole-cgroup peak
912,211,968 bytes, zero OOM, all four exits 0 and verified container absence.
Its strict source/artifact receipt is `arm64/acceptance/current-replay/receipt.json`. These
are **EMULATED / MOCK VERIFIED**, never RK3576 timing or memory predictions.

The ROS separation boundary is concrete: node setup/parameters and factories;
image conversion/mono subscription or ExactTime stereo bundling; IMU callback
queue feeding `ImuHandler`; and pose/Info publication plus filesystem tracing.
Keep the estimator core untouched during first reproduction. A future ROS-free
adapter can replace these boundaries after a native baseline exists, with the
same input-order, clock, transform and lifecycle tests.

RK3576 bring-up still must establish camera sensor/mode, global versus rolling
shutter, exposure event, row timing, frame rate, trigger ability, IMU sample
anchor/range/filter, independent gyro/accel pairing, clock drift/uncertainty,
rigid extrinsics, temporal offset and noise. None can be copied from EuRoC or
old M3C metadata. Front and down cameras may not overlap. Validate each as an
independent monocular candidate first; ExactTime stereo support does not prove
non-overlapping multicamera fusion or shared-clock acquisition.

RK3576 acceptance additionally requires native target build and replay, sustained
live input, thermal/resource/latency measurements on the 4 GB board, observable
tracking/loss/reset semantics and separately authorized physical calibration.
PC resource limits of 12 GiB do not qualify that 4 GB deployment. Cross-compiling
or QEMU execution cannot provide target real-time evidence.

The ownership ARM64 installation now has a fresh complete V101 stereo replay
under the declared4GiB limit, using the same existing runtime entry. It produces
2,912 poses and fixed-scale ATE0.0601403759545 m; all four child exits are zero
with exact owned absence. Cgroup memory.max is4,294,967,296 bytes,
memory.peak589,340,672 bytes and OOM counters zero. The independent full
source/ELF/loader/deployment/input/raw-adapter audit passes, receipt SHA256
`467856d0cecca1c54c2e5e24b68aa127e85af36b283df01f9af2e5151937d10b`.
It checks45 outputs,17 runtime identities,953 source and527 devel files,
52 AArch64 ELF objects and238 loader dependencies. The actual base archive
and39-path overlay,13 patches and raw symlinks resolve exactly. One-second
RPE is0.0432402878233 m /0.461785630790 degrees. Complete state/watchdog
reconstruction and83,143,767-byte propagation output are exact;3 health
transitions and526 invalid states remain visible. This is **EMULATED / MOCK
VERIFIED**; IMU callbacks are complete, while actual downstream integration
samples remain UNOBSERVED. These results belong to installation dcf6b1e…
(PC1dff ownership plus seven ARM platform deltas),
which does not include later PC terminal/calibration/precise-kernel changes.
It remains a Bionic/Melodic ARM64 reference, with no RK3576 BSP/ABI, physical
sensor, performance or flight qualification.

## UZH-FPV feasibility boundary

The [original dataset page](https://rpg.ifi.uzh.ch/uzh-fpv.html) redirects by HTML
to the [current site](https://fpv.ifi.uzh.ch/). The original changelog records a
March 30, 2020 ground-truth time-offset correction; the current site announces
new ground truth on February 2, 2022 and SplitS sequences added in December 2023.
A future intake must freeze the actual downloadable dataset, calibration and
truth files and hashes, not only the landing-page URL or a generic “UZH-FPV” name.

The site describes Snapdragon Flight images/IMU, mDAVIS images/events/IMU and
Leica measurements. These are separate sensor combinations, not interchangeable
inputs. Standard ROS Image and Imu messages make a topic-filtered ROS1 transport
feasible, but camera models/extrinsics, truth body frame and any clock-offset
correction must be verified per selected release. Do not assume a EuRoC ZIP
layout, EuRoC calibration, a dense full six-DoF truth stream or unchanged timing.
Read the current dataset-format description and archive metadata before
implementing a converter. Keep truth and event topics out of the estimator
unless explicitly part of a separately defined experiment.

Two official v3 Snapdragon archives have now been downloaded, ZIP-checked and
converted through the common sensor-event interface without changing their
source layout or decimal timestamp evidence:

| Sequence | Archive SHA256 | Camera / IMU records |
|---|---|---:|
| indoor_forward_3 | `6a4df450ef701880c046d2894876988a228002496fb86645674fde2f82b880a7` | 2,552 / 46,053 |
| indoor_forward_6 | `5da4d454c35b9b20d3566460d6f25f93b446d316eec05c828c861bf901c4f538` | 1,970 / 34,815 |

The separately downloaded calibration ZIP has SHA256
`10d97ef7cb32915d6c287ec7028d7d037de77b00b259c3aa2142fd16907c920a`.
Its 640x480 pinhole/equidistant cameras have separate camera–IMU shifts. Kalibr's
report explicitly says `t_imu = t_cam + shift`; the Schur delay parameter uses
the opposite sign. The checked conversion rigid-inverts `T_cam_imu` to
`T_imu_camera`, preserves raw source timestamps and labels inherited estimator
initialization/range settings as experiment configuration. ROS1 nearest-ns
ROUND_HALF_EVEN quantization is explicit; original decimal seconds and integer
picoseconds remain in the source ledger. This representation is not a claim of
physical picosecond clock accuracy. The cameras are tested independently;
the two distinct time offsets are not collapsed into an ExactTime pair.

Official `uzh-rpg/uzh_fpv_open` commit
`f01b23d06724e5b8050b5820fae413614a486176` uses ground truth as `T_world_IMU`
in its projection script. The public repository explicitly excludes the truth
generation code and does not establish whether image times were already
compensated. This uncertainty remains `NOT_ESTABLISHED`; published-header
scoring is diagnostic only. The truth covers a later interval of each sequence,
so the full source prefix stays in coverage denominators. Source checks and
actual truth hashes are retained in
`.agents/runs/schurvins/uzh-source-audit/audit.json`. Real estimator replay and
artificial perturbation results are separate acceptance items; intake success
cannot qualify high-motion tracking or recovery.

A November 1 source follow-up verifies that the pinned public degroundtruth
script forwards retained messages/stamps and copies retained ZIP members byte
for byte. The pinned text exporter directly serializes camera/IMU header stamps;
that function applies no calibration offset. These steps still do not bind the
downloaded v3 ZIP hashes to specific original bags or exporter invocations, or
establish upstream header processing. Precompensation remains NOT_ESTABLISHED;
raw stamps, configurations and diagnostic-only scoring are unchanged. The
bounded `uzh-source-audit/release-time-preservation-receipt.json` receipt has
SHA256 `0e9ed15092fff9b14e88e7cec4478566a449c97ab66bf208d80069157abd0198`.
This is a dataset provenance gap, independent of RK3576 hardware availability.

## Preserved initial PC baseline

Initial baseline verdict: **PARTIAL**. The independent PC build, two-sequence stereo
reproduction and same-input stereo comparison are complete. The required
SchurVINS monocular trajectory is still **FAIL** on both tested sequences;
it is not replaced by stereo acceptance. All original 44 native replay groups ended; subsequent derivative experiments use new output paths.

### Frozen environment and data

The Linux x86_64 build succeeded for all 25 selected catkin packages using
Ubuntu 18.04 / ROS Melodic / Python 2 transport inside an isolated container;
the host evaluator uses Python 3. Immutable image:
`sha256:b4a9eb291ae6c65fd80d2841bb225c5bf86658f6c0a58cdca710ce1265cfc34a`.
The 14 source archives and 1,124 installed package versions are locked.
The successful build record is `build-pc-locked/installation.json` under
`/home/shiuhou/Projects/rm27-schurvins-research` (the external artifact root).
`installation-symlinks.json` is an explicitly derived manifest additionally
freezing 2 source and 253 devel symbolic links. Original manifests remain intact.
A final audit matched all source archives, 938 source files, 527 devel files,
links, native binary records and package versions against their frozen records.

| Original EuRoC archive | SHA256 | Selected source records |
|---|---|---|
| V1_01_easy | `a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa` | 2,912 images per camera; 29,120 IMU |
| V1_03_difficult | `3a29beda3c9467dcab08bfa181ed924c3397c2bdb3a780baaef6eb12955d82a4` | 2,149 images per camera; 21,500 IMU |

Both archives were reused from the immutable local archive tree. Mono selects
cam0 + IMU; stereo selects cam0 + cam1 + IMU with exact original left/right
integer timestamps. Decoded pixels, raw IMU values and calibration files are
preserved. The original archives/truth are never mounted into either estimator.
Publisher/readback/observer reconciliation does not prove native IMU ingestion.

OpenVINS reuses commit `69488123ed9362dd44b6f28e7f4680abbff1442b`, the
previously built ROS2 Jazzy executable and its explicit header/lifecycle
compatibility patches. Its old V1_01 mono runner and historical 1 cm gate remain
unchanged. New stereo/difficult inputs use the independent
`experiments.openvins_comparison` entry. Each native run uses the same host,
4 CPU / 12 GiB limits and sequential execution. Full-sequence 0.5x accuracy and
separate 1x performance groups are never pooled.

### Execution, failures and integrity

The complete matrix contains **44 actual runs**: 13 `PASS`,
23 `EXECUTION_COMPLETED` and 8 `FAILED`. Those labels retain their original
entry-point meaning; comparison execution is not VIO-P qualification.
All 41 runs with declared artifact manifests verify. The three old OpenVINS
historical-gate failures remain `UNLOCKED`: the unchanged runner stops before
sealing final artifacts. Their raw trajectories and numerical scores are kept,
without claiming the absent final manifests exist.

All eight failures remain visible:

- One early Schur mono startup failure before estimator creation: network-none
  ROS1 could not resolve/connect to its hostname. The recorded transport fix
  confines ROS addresses to loopback and requires output TCPROS readiness.
- Schur mono V1_01 and V1_03: complete selected inputs and all four process exits
  zero, but only one pose at camera 110 / 5.5 s and camera 140 / 7.0 s respectively,
  followed by stage 3 / 0. Both fail `TRAJECTORY_NOT_EVALUABLE`; dependent mono
  repeats are `NOT_RUN`. Numeric single poses do not certify metric initialization.
- Three original OpenVINS V1_01 mono repeats fail `NATIVE_BASELINE_DRIFT` against
  the unchanged 1 cm criterion despite clean native/player/recorder exits.
- Two initial OpenVINS stereo smokes fail `IMU_DELIVERY_MISMATCH`: the observer
  receives 29,119/29,120 V1_01 IMU and 21,487/21,500 V1_03 IMU records. Native
  ingestion loss is unproved. Their original dependent groups are `NOT_RUN`.

The revised `explicit-reliable-publisher-and-observer-paused-match-v1` protocol
uses reliable publishers/observers, records endpoint QoS and waits for observer
matches before resuming playback. Native OpenVINS SensorDataQoS stays unchanged.
Its 20 predeclared runs (V1_01 stereo smoke + five repeats + 1x; V1_03 stereo
smoke + five repeats + 1x; V1_03 mono smoke + five repeats) all completed with
exact selected inputs, clean exits and same-run raw/adapter metric parity.
The 14 recorded runtime-source hashes are identical within this revised matrix.
Earlier Schur cohort provenance utilities varied; per-run source hashes retain
those differences rather than asserting all 44 runs used identical tooling.

The selected Schur compile definition and pinned stereo first-frame control
flow, together with native stage=tracking output, establish that Schur
Forward/Backward were reached. This is not a count of per-frame successful
visual corrections. Mono initialization lacks those first-frame calls and uses
geometric initialization before shared Schur prediction. That asymmetry is a
fault hypothesis, not a proven complete root cause. No estimator equations,
scale, initialization or noise were changed to make reproduction pass.

### Five-repeat accuracy

ATE is fixed-scale SE(3), mean +/- sample standard deviation in metres; RPE is
mean 1-second translation RMSE in metres. The denominator includes every planned
member, including the three scored OpenVINS gate failures.

| Full sequence, 0.5x cohort | Successful / planned | ATE mean +/- std m | Mean 1 s RPE m |
|---|---:|---:|---:|
| Schur V1_01 stereo | 5/5 | 0.040880 +/- 0.004057 | 0.042866 |
| Schur V1_03 stereo | 5/5 | 0.063752 +/- 0.015466 | 0.034213 |
| OpenVINS V1_01 mono, preserved runner | 2/5; all 5 scored | 0.059727 +/- 0.007242 | 0.044702 |
| OpenVINS V1_01 stereo, revised transport | 5/5 | 0.063392 +/- 0.005395 | 0.043946 |
| OpenVINS V1_03 stereo, revised transport | 5/5 | 0.062157 +/- 0.002354 | 0.035096 |
| OpenVINS V1_03 mono, revised transport | 5/5 | 0.064581 +/- 0.000836 | 0.039134 |

Schur stereo has a numeric camera-associated pose for every source camera
(2,912/2,912 V1_01; 2,149/2,149 V1_03), not evidence of successful visual
correction on every frame. OpenVINS visual poses cover camera 112--2911
(2,800/2,912, 96.15%) and camera 144--2148 (2,005/2,149, 93.30%). Initialization
prefixes remain missing coverage; no crop hides them. Exact initialization
completion, native ingestion/reset counts and operationally valid-state ratios
are unknown. Adapter operational validity deliberately stays false.

Each stereo smoke and repeat pair was also scored over its common interval,
while preserving both complete reports:

| Common interval, five paired repeats | Schur ATE mean +/- std m | OpenVINS ATE mean +/- std m |
|---|---:|---:|
| V1_01 camera 112--2911 | 0.041050 +/- 0.003967 | 0.063392 +/- 0.005395 |
| V1_03 camera 144--2148 | 0.063369 +/- 0.015691 | 0.062157 +/- 0.002354 |

No scale or time shift was fitted; original truth hashes and reset epochs remain
explicit. ATE, adjacent/1-second translation/rotation RPE and absolute orientation
errors are retained in evaluation JSON. Original V1_01 orientation truth has
known limitations. The preserved historical adapter03 re-score gives exact
original ATE/RPE/scale parity (ATE 0.057384774420043264 m); this is evaluator
reuse, not another native run. Observed pairwise repeat differences are in the
summary. Schur V1_01 has 1/10 and V1_03 has 8/10 pairs exceeding 1 cm ATE
difference; this diagnostic does not redefine the old OpenVINS gate.

### Configuration differences and limits of comparison

The two algorithms consume the same selected stereo images/IMU, but their
upstream calibration/noise and online-calibration policies differ. Schur cam0
intrinsics begin at
`[458.6548807207614,457.2966964634893,367.2158039615726,248.37534060980727]`;
OpenVINS begins at `[458.654,457.296,367.215,248.375]`. Initial rigid extrinsics
match in the inspected files. OpenVINS enables online intrinsics, extrinsics and
camera/IMU-offset calibration; its retained logs show changes. No equivalent
online calibration was enabled for Schur.

| Configured IMU quantity | Schur mono | Schur stereo | OpenVINS mono/stereo |
|---|---:|---:|---:|
| Gyro noise density | 0.0012 | 0.004 | 0.00016968 |
| Accel noise density | 0.008 | 0.08 | 0.002 |
| Gyro bias random walk | 0.03 | 0.000002 | 0.000019393 |
| Accel bias random walk | 0.1 | 0.00004 | 0.003 |
| Configured IMU rate Hz | 800 | 200 | 200 |

These are source parameters, not measurements or equivalent-noise-model claims.
All actual EuRoC input samples retain approximately 200 Hz cadence. Schur passes
these noise values into `InitImuModel()`; filter gravity is hardcoded as
`[0,0,-9.81007]` in `schur_vins.h`, not established by the mono YAML's gravity
value. Different ROS versions/transport and unknown native ingestion remain
additional confounders. No pure-algorithm winner or same-input mono comparison
is claimed while Schur mono is unevaluable.

### Separate 1x PC timing and resources

| Stereo run | ATE m | Native timer P50/P95 ms | Maximum timer ms | Native CPU / wall s | Max RSS KiB |
|---|---:|---:|---:|---:|---:|
| Schur V1_01 | 0.048925 | 2.330 / 3.044 | 172.435 | 19.92 / 153.55 | 96,116 |
| OpenVINS V1_01 | 0.072028 | 10.520 / 17.781 | 26.370 | 56.63 / 157.87 | 166,724 |
| OpenVINS V1_03 | 0.074965 | 8.940 / 15.610 | 24.240 | 38.82 / 119.87 | 154,760 |

Schur stage=2 Info frontend timing and OpenVINS camera-update timing have
different boundaries. These are not sensor-to-publication latency and do not
establish an algorithm-only speedup. GNU-time resources cover the native
process tree. Whole-container cgroup CPU/memory summaries remain separate and
include transport/ROS/cache; memory.current is not RSS. Schur V1_01 observed
camera lag at Info receipt is P95 zero / maximum one frame, not completion
latency. End-to-end latency and native processing backlog remain `NOT_RUN`.
PC results under a 12 GiB limit cannot qualify the RK3576 4 GB board.

### Evidence locations, commands and final software verification

`comparison-final-audited.json` under the external artifact root re-verifies
all 44 runs and six planned cohorts using the final summarizer. SHA256:
`970ee003ee3d4efc10d1f1104e0f2edeb4b0fb778a07a0a3a67a4dcaa1326cd7`.
It records summarizer SHA256
`6251a1deac02db630d42dfbeef839334455c36377050e6ee26ec0648009b3ba1`.
Its run/cohort results exactly match the earlier retained `comparison-final.json`
(SHA256 `02f5a27f00b32d8075439f8c1135c39a13bb7c6596e38aa415d197b65d607266`).
The earlier summary was made before the mutable-output rejection branch was
added; neither summary rewrites any original run manifest.
`comparison-v101-stereo-{smoke,repeat1..5}-complete.json` and the corresponding
`v103` files retain full/common-interval scores, original truth/input hashes,
evaluator hash and the reused `native_accuracy.py` math-source hash.

Actual invocation forms used from the repository root include:

```bash
python3 -m rm27.perception.vision.localization.experiments.schurvins_runtime \
  --config /home/shiuhou/Projects/rm27-schurvins-research/configs/v101-stereo-half-repeat1.json
python3 -m rm27.perception.vision.localization.experiments.openvins_comparison \
  --config /home/shiuhou/Projects/rm27-schurvins-research/openvins/v101-stereo-reliable-half-repeat1.json
```

Those completed output paths are immutable and will be refused on rerun. Follow
`tools/schurvins/README.md` to create a fresh run ID/output, using the link-frozen
installation. No commit, push, other RM27 repository change or hardware action
was performed. The branch remains `docs/progress-handoff-rule`, baseline HEAD
`a8392c6f1e32e76532bd3ea8aa34ef98e0f06da6`; pre-existing ChArUco and interface
worker changes remain preserved.

After the native matrix, final software hardening added bounded child shutdown,
unconditional uniquely owned Docker cleanup plus verified container absence,
refusal to seal/analyze potentially mutable outputs, exclusive output ownership,
complete adapter/math-source provenance and image/state stamp equality checks.
These changes passed synthetic production-path regressions; a real pinned-image
`true`/cleanup probe also verified the Docker absence protocol. The original
44 native runs were not rerun with these final changes and retain their actual
per-run source hashes. The software suite at completion of that original 44-run matrix, with ROS2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q --ignore=tests/camera_capture.py
```

Result: **296 passed, 1 skipped** (existing ChArUco environment skip).
`git diff --check` passed. No replay controller/estimator or container remains
running. Software regression and cleanup probes do not certify live VIO.

### Remaining acceptance gaps

- **FAIL:** Schur monocular reproduction and same-input monocular comparison.
  Preserve upstream one-pose failures and derivative failures. Monocular metric bootstrap still does not sustain the original
  inlier gate; bounded same-factor and point-lineage diagnoses remain software
  work, not a hardware-only dependency.
- **BLOCKED:** MH_04 download (old ETH SSL timeout; new Research Collection
  DOI `10.3929/ethz-b-000690084` returned HTTP 429 locally). V1_03 is a verified
  difficult-sequence substitute, not evidence that MH_04 ran.
- **PASS, reference scope:** ARM64 target-under-QEMU build, ELF/loader/NEON,
  deployment-package extraction/startup and exact-version full stereo replay.
  The toolchain/sysroot limitation is resolved. Current-code replay and lifecycle
  receipts remain separate from these preserved artifacts.
- **FAIL:** UZH-FPV forward 3/6 mono estimator replays produced only 35/13
  poses before public-GT coverage, with no scorable epoch.
- **NOT_RUN:** Most current native fault/repeat cohorts. Additional space has
  resolved the storage blocker; the full host-state-policy automatic-reinitialization
  group has now completed with audited recovery, as recorded below. Completed
  numerical replays alone do not establish accepted visual recovery. A completed
  OpenVINS artificial 500 ms camera-gap run has numerical results but no audited
  accepted-visual/reset/watchdog recovery evidence.
- **HARDWARE BLOCKED / NOT_RUN:** RK3576 BSP/ABI, native board runtime,
  camera/IMU qualification, live latency/resources/thermal behavior, physical
  controller integration, contact recovery and flight.

## Current pre-hardware implementation and evidence

The current checkout remains on `docs/progress-handoff-rule`, HEAD `a8392c6`.
Uncommitted calibration/interface work is preserved; no commit, push or merge
has been performed. The separately registered `research/m3c-vio` worktree is
outside this task. Future shared-code transfers require explicit source-version
and diff review. Concurrent work on the same host can affect timing; overlapping
runs do not establish an exclusive-host performance ranking.

The latest full software regression executed with sourced Jazzy is **729 passed,
1 skipped** (16.23 seconds, exit 0). The current contract subset passed 516 tests
with clean process-group exit. Targeted test receipts never replace estimator
replay. The single pre-hardware matrix preserves all 75 gates and 67 required
native cohorts. Its current selected acceptance reports 14 PASS / 62 NOT_RUN,
overall **PARTIAL**; required native coverage remains six rejection cohorts
PASS and 61 estimator cohorts NOT_RUN. Current ARM build, core-only emulated
replay, lifecycle and flight-interface receipts are connected with their exact
scopes. Core-only replay cannot close operational-state or recovery gates.
Current-source timeout, host SIGINT and exact-owned native-child SIGKILL probes
also passed their expected-failure cleanup assertions. Original run failures are
retained, including native exit 137 for SIGKILL and unknown individual target
exits after forced host-interrupt removal. Container and every tracked owned
PID/start-time pair were absent. These verify cleanup, not successful trajectories.

The common input path supports EuRoC, genuine UZH text/PNG intake, sealed offline
recordings, deterministic mock sources and explicit rate replay. Sensor identity,
raw and mapped clocks, sequence IDs, calibration source hashes, unknown exposure
and shutter semantics are retained. Tests reject nested truth metadata, clock
rewinds, mapped-time collisions and invalid IMU; no silent sorting or filling is
allowed. Publisher global delivery order is checked against the declared schedule;
observer callback order is assessed per topic. Twenty-two fault profiles have
prepared inputs, including three intentionally invalid IMU profiles rejected
before native startup. Their preparation is not a fault-recovery experiment.

The first complete OpenVINS artificial camera-gap experiment used a declared
500 ms interruption, original source frame IDs and the unchanged full 2,912-frame
denominator. Both cameras delivered 2,902 images; all 29,120 IMU samples remained.
The native output contains 2,790 visual poses and 27,994 propagated states,
recorded separately. Fixed-scale SE(3) ATE is 0.0566455682 m; 1-second RPE is
0.0441958458 m / 0.464356805 degrees. Native and adapter metrics match exactly.
All native/player/observer/recorder/resume/container exits were 0, cleanup was
verified, and 51 sealed artifacts plus 16 source identities were checked.
Evidence: external `prehardware_openvins_fault_gap500ms_v1/`.
Numerical replay is **PC OFFLINE VERIFIED**; state validity remains unknown/invalid
and the stronger recovery gate is **NOT_RUN**, because accepted-visual/reset/
watchdog recovery evidence is unavailable. This single run does not establish
repeatability or an algorithm ranking.

The output path preserves `LocalizationEstimate` v2. Explicit accepted native
visual corrections, snapshot-based IMU propagation and independently scheduled
watchdog ticks retain separate times and provenance. Watchdog ticks never advance
sensor time or last-visual-update time. Missing native initialization/quality
evidence keeps original Schur output invalid. MAVLink wire conversion requires
velocity, world alignment, body extrinsics and healthy current state; unknown
covariance uses the specified wire sentinel and stays null in strict JSON.
The mock receiver and pure conversion are verified. Three production-path tests
also reproduced and fixed a receive-clock rewind that could revive a stale
state; callbacks and watchdog evaluations now share one monotonic receive clock,
and transport emission cannot precede the recorded watchdog time.

The isolated fixed-version PX4 SITL receiver actually received two nontrivial
synthetic ODOMETRY messages, including reset counter 17 to 18 and matching frame,
position, orientation and velocity. Startup, MAVLink stop, owned native SIGINT
exit 0 and process/container absence were verified. The receipt is
`.agents/runs/schurvins/state-flight-mock/sitl/current-signal-verified/receipt.json`.
This is **EMULATED / MOCK VERIFIED / SYNTHETIC_ODOMETRY_RECEPTION_ONLY**.
The minimal lockstep native sample clock remains zero, so PX4 timesync,
measurement-time conversion and actual VIO fusion remain **NOT_RUN**. Earlier
dataman-stop and daemon-shutdown failures are retained.

P0 remains **FAIL**. The upstream V2_02 document-offset-10 and complete-sequence
experiments both produce only one pose and cannot be scored. Diagnostics establish
that the upstream mono geometric commit does not initialize the Schur metric
state. The authorized derivative supplies visual/IMU alignment, atomic map/filter
handoff and reset cleanup, but has not passed a complete mono sequence. One
V1_01 diagnostic completed metric initialization at frame 676 and a true Schur
visual correction at 678, then failed the unchanged 40-current-inlier gate with
25 survivors. A later full run did not pass metric initialization at all.
The actual VO/raw-IMU replay through the production solver, without truth,
reproduces predominantly gravity/bias-bound refusals. A subsequent derivative
uses the explicit configured accelerometer-bias prior only for the initial
unconstrained gravity solve, then retains the fixed-gravity-norm tangent solve,
absolute bias estimation and every original gate. Known-motion and degeneracy
fixtures plus 49 fixed-window/prior diagnostics passed without truth. Its full
V1_01 run initialized at frame 153 but produced only one pose: at frame 156 a
Schur correction increased current mean reprojection residual from 3.07 to
16.4 pixels and removed all current inliers. All four processes exited 0 and
cleanup was verified; trajectory acceptance remains **FAIL**. This is an unresolved
software/algorithm integration gap, not a hardware dependency. No monocular
repeat cohort is promoted until its full smoke succeeds.

The subsequent bounded landmark diagnostic also completed every process with exit 0,
verified container absence and exact published/observer inputs (2,912 cameras and
29,120 IMU). Native frontend completion was 2,910 frames; original camera indices
871 and 873 were missing, and complete native IMU usage remains unobserved.
All ten counterfactual point updates restored every affected field. At the first
update, current inliers fell from 67 to 2; the trial point update recovered 36,
still below the unchanged 40-inlier threshold. Mean residual was 1.562, 8.014 and
4.133 pixels respectively. The full run emitted no pose and remains **FAIL**.
This shows an ordering contribution without establishing a sufficient repair.
A separate complete numerical diagnostic produced one pose and remained FAIL.
Its first eight updates had full-rank positive point blocks, inverse residuals
below 3.1e-11, the expected seven-dimensional visual gauge and alternative-solve
deltas within approximately 1e-8. This does not support a numerical-solver
explosion as the initial cause. A further complete conditional diagnostic
retained one pose; 15 source and 37 output hashes, every exit and cleanup were
verified. At frame 159 selected current inliers were 69 before camera correction,
22 afterwards, 38 after the production point update and 71 after an uncommitted
free conditional update. All seven trials restored point state; free shifts
reached approximately one metre. This is diagnostic evidence, not a qualified
repair. The earlier frame-156 first loss also involved the quality population
error described below; it must not be described as the first sub-40-inlier frame.

The explicit stereo metric-observation build completed fresh full 0.5× smokes:

| Sequence | Poses / GT associations | ATE RMSE m | 1-second RPE m / deg | Numerical replay |
|---|---:|---:|---:|---|
| V1_01_easy | 2,912 / 2,871 | 0.0418451709 | 0.0427646256 / 0.4633445584 | PASS |
| V1_03_difficult | 2,149 / 2,094 | 0.0414588523 | 0.0334370255 / 0.3369506287 | PASS |

Both preserved full input/Info tails, same-run raw/adapter score equality,
zero post-frontend reset calls, four exit-0 processes and verified container
absence. Evidence is external `runs/v101-stereo-metric-observation/` and
`runs/v103-stereo-metric-observation/`. The additional geometry/filter/bootstrap
observations expose calibrated stereo scale and accepted Schur constraints.
They do not change the estimator's tracking-quality criteria. The stronger state
interface remains **FAIL**: V101 accepts 1,675 corrections and rejects 1,236 for
native tracking quality; V103 accepts 1,168 and rejects 975. Numerical replay
success therefore does not establish uninterrupted valid localization. These
smokes use a 2-second receive-age policy and a 15 ms IMU-gap limit; the planned
matrix retains its distinct 500 ms / 20 ms policy. Host concurrency is not
controlled sufficiently for an exclusive-host performance ranking.

Native quality diagnostics established two concrete defects. The old feature-drop
comparison subtracted current mature Schur inliers from a previous population
including seeds: all 1,236 V101 refusals used this mismatched comparison. In V103,
13 early-keyframe frames retained a stale Good value despite fewer than 40 Schur
inliers, with a minimum of seven; those projected states were incorrectly usable.
The independent `tracking-quality.patch` refreshes quality before early returns,
compares consecutive actual Schur-inlier populations, preserves the 40-inlier and
40-drop gates and keyframe exception, and marks bootstrap without a visual
correction insufficient. Source-derived C++ tests exercise the real patched
helper. Old results and their invalidity evidence remain unchanged.

The first complete corrected V101 stereo 0.5× smoke is **PC OFFLINE VERIFIED**:
`runs/v101-stereo-tracking-quality-half/`, 2,912 poses / 2,871 GT associations,
fixed-scale ATE 0.0772194515 m and 1-second RPE 0.0438960304 m /
0.4664575600 degrees. Four processes exited zero; exact camera/IMU readback,
15 source hashes, 39 output hashes and container absence were verified. The
state interface accepted 2,908 corrections, retained four explicit invalid
health transitions and recorded zero evidence errors. Its PASS means interface
evidence integrity, not uninterrupted availability. This single run cannot show
an accuracy improvement or repeatability; comparisons retain all prior results.

That same run now actually passes its recorded raw IMU into the independent
state tracker: 29,120 samples, 29,109 complete v2 records, 29,079 usable records,
200 Hz in sensor time and no rejected samples. The bounded JSONL is 79,362,528
bytes, with source-ordinal, SI units, frame/calibration and zero-offset clock
bindings checked. Visual freshness is not advanced by propagation, native
publication/completion times remain unknown, and offline adapter emission is
labelled separately. This verifies the offline state-output path; it does not
make propagation a new visual observation or qualify flight.

Propagation reporting now separates emitted-record cadence on original IMU
source time from distinct state timestamps within each epoch. The production
quality-loss regression first reported 866.67 Hz for a 200 Hz callback stream:
invalid frozen snapshots were counted against a short advancing-state span.
Read-only audit of `runs/v101-mono-point-order-half/` found 27,569 records but
only 51 usable records and 51 distinct state timestamps. Its original
110,272.056 Hz report is incorrect; record cadence is 200 Hz over 137.84 seconds,
while those state timestamps span only 0.249999872 seconds. The original sealed
artifacts remain unchanged. This corrects reporting, not estimator behavior;
neither cadence measures new visual corrections or continuous availability.

Expected health transitions now remain invalid without masquerading as evidence
corruption. Missing constraints, initialization, epoch/reset association or clock
evidence still fail the interface. Recovery uses two new actual visual
corrections in one epoch with continuous healthy state/Info between them.
Watchdog detection during camera silence retains its observer timestamp and
bounding Info source times; no interpolated sensor time or cross-clock latency
is invented. Detection must lie in the declared half-open source-time fault
interval, or have an explicit same-clock receipt bracket overlapping that
interval, before it can close recovery acceptance. Later loss/recovery remains
visible but cannot substitute for interval association. Receipt comparisons
require nonnegative integer nanoseconds, VERIFIED interpretation and an explicit
non-UNKNOWN clock identity. The same requirements apply to watchdog continuity
between recovery endpoints; unqualified ticks cannot be silently skipped.
Repeated receipts cannot establish an inverted source-time bracket. Missing
evidence stays NOT_RUN.

Four original stereo fault runs are retained in `prehardware_metric_faults_v1/`.
A 500 ms camera gap produced 2,902 poses and ATE 0.0438772718 m; a five-second
gap produced 2,812 poses and ATE 0.9567228691 m. Both numerical replays exited
cleanly, but stronger recovery remains unverified. A 100 ms delivery delay
aborted at the old-camera `getMeasurements` assertion; a 50 ms IMU gap aborted
at the existing strict 15 ms Prediction check. Both native exits were 134,
player exits 1, and cleanup was verified. The isolated `input-health.patch`
rejects old cameras before that prior call and handles IMU faults with explicit
image-thread reset, invalid Info and cleared buffers. Its 25-package build and
production-header tests pass; corrected delivery-delay and IMU-gap
executions are recorded below, without claiming accepted visual recovery.
No sampling gap is interpolated and the 15 ms criterion is unchanged.

The sensor recording/replay path now consumes explicit front/down selection
boundaries. Each recording or source-clock epoch requires an acknowledged
consumer reset binding the exact epoch, camera ID and calibration hash before
its first event. Tests verify independent-camera fault isolation and refusal of
missing/failed/wrong acknowledgements. The CLI uses the existing `replay` command
with `--next-input`; its actual consumer resets sensor output routing. This is
**EMULATED / MOCK VERIFIED** sensor-interface evidence. The existing native recording-session consumer now stops, verifies absence
and restarts a separate estimator with the next calibration. Both actually
executed session transports passed, but both mono estimators produced no pose;
the remaining interface/reproduction failure is software, not a hardware blocker.

The current source passed **729 tests / 1 skip**, with ROS Jazzy sourced
(2026-10-31, 16.23 seconds, exit 0). This includes the summary-only no-reference
fix, whose two new tests first reproduced `KeyError: epochs` and then passed. These are software tests, not estimator or
hardware qualification.

A declared artificial-disturbance replay now retains separate scores for every
epoch and returns `EXECUTION_COMPLETED / PER_EPOCH_FAULT_OBSERVATION` after
complete audited execution. It does not receive baseline continuity PASS or a
combined score. Original-data baselines still require exactly one total,
scorable epoch and no departure from tracking. The pre-hardware gate accepts
this observation status only for the explicit fault-recovery role, rechecks the
prepared input and scope, and separately requires detected loss and two
continuous new accepted visual corrections, with explicit fault-interval
association. A successful reset therefore can
be measured without either hiding its discontinuity or failing merely because
it creates a second epoch.

The quality-only full V101 mono diagnostic remains **FAIL**:
`runs/v101-mono-tracking-quality-half/`, one pose, no scorable epoch. All 2,912
camera and 29,120 IMU records matched, all four processes exited zero, container
absence and 15 source / 38 output hashes were verified. It observed two metric
initializations and 17 native visual corrections; these did not establish a
usable continuous trajectory. Its run manifest SHA256 is
`37452d9bddf185ef33cea734433a0eb3b0f8e8aa799cbff421b3f9b1610d6a53`.
The existing combined-input mono failure also exposed a separate startup
regression: `set_start_` cleared IMU after the interface coverage check, and the
new actual-input guard treated missing new-epoch startup coverage as another
fault. `input-restart-coverage.patch` waits for initial coverage while keeping
established-state, gap, order and nonfinite checks strict. Its independent
25-package build passes. The fresh full `v101-mono-input-boundaries-half`
run also retained only one pose (one metric initialization, six native
corrections), **FAIL / TRAJECTORY_NOT_EVALUABLE**. Four exits, exact 2,912 camera /
29,120 IMU readback, 15 source / 38 output hashes and cleanup were verified
(run SHA256 `7399a05c41e27c7b15826b4cb9fd1b1614ded7ca135f5d015ce45f1bfdaa20ea`).
It emitted no input-health boundary events, so it did not dynamically exercise
the startup-wait repair; that specific regression remains open.

The earliest old-versus-combined monocular VO divergence precedes the first
quality decision. Upstream OpenGV defaults to a wall-time-seeded RANSAC; a
production-header observation probe verified that its seed can be inspected
without changing 1,000 draws. This identifies a variation source, not proof of
causality for that run. No seed, ground truth, scale or inlier threshold was
changed to force success. Independent production-function numerical probes also
did not support the proposed singular compressed-solve failure: 36 bounded
Gaussian cases matched raw-factor Kalman increments to 1.42e-12 relative error.
These negative findings prevent an unsupported solver replacement.

Metric transfer copies nominal FEJ-labelled snapshots, but the reviewed upstream
visual Jacobians use current states and its prediction FEJ correction is
commented out. Copying those fields is not an enabled FEJ consistency guarantee.
Bootstrap joint uncertainty and landmark cross-covariances remain unresolved
software questions; the upstream independent camera/point approximation is not
silently relabelled a proven bug or replaced by an unsourced prior.

The full V103 stereo `input-boundaries` smoke is also **PC OFFLINE VERIFIED**:
2,149 poses over all 2,149 input cameras, 2,094 GT associations, ATE
0.07084389685 m, 1-second RPE 0.03567406877 m / 0.3754823621 degrees, fixed
scale 1. All four exits, exact input readback, 15 source / 39 output hashes and
container absence were verified. State evidence accepted 2,088 corrections,
kept 61 invalid health transitions and found no evidence errors; all 2,149 native
quality decisions were observed. The same-run independent IMU snapshot path
wrote 21,481 complete v2 rows (20,868 usable), 200 Hz sensor time and
58,801,375 bytes. The run SHA256 is
`c0c200a6f91f76f4b2fa9ab9ebb567c201e1b800a715bb49bb0a03e287dc2340`.
This is a single smoke, not five-run repeatability, uninterrupted availability
or an accuracy improvement claim.

## RK3576 hardware bring-up checklist

Every item below starts **NOT_RUN / HARDWARE BLOCKED**. The sensor/driver-specific
capture and live adapter must be bound after selecting the actual hardware;
the listed validation tools already exist. None of these rows is closed by
EuRoC, QEMU, mock ODOMETRY or an old M3C recording.

| Check | Required evidence and existing validation entry | Status |
|---|---|---|
| BSP and devices | Confirm CV00/MV12 and actual 32/64 GB SKU; obtain missing vendor SDK/image and hashes, identify kernel/userspace/loader/ABI, sensor/IMU IDs and driver/mode; enumerate actual media graph instead of using example video-node numbers | NOT_RUN |
| Timing and integrity | Measured sample/exposure anchors, shutter/row timing, gyro/accel pairing, clock mapping with uncertainty and loss ledger; `experiments.sensors validate --input <recording>` | NOT_RUN |
| Calibration | Per-mode camera intrinsics, IMU noise/range/filter, `T_imu_camera` and signed time offset with hashes; existing calibration validators | NOT_RUN |
| Real recording on PC | Sensor-only recording replay and fixed-scale scoring; existing input preparation and `experiments.schurvins_runtime --config <new-config>` | NOT_RUN |
| Board offline replay | Native BSP binary/loader/dependency identity, complete recorded replay, process exits and resource trace; existing ARM64 manifests and replay configuration | NOT_RUN |
| Board live VIO | Sustained physical camera/IMU input, accepted visual corrections, loss/reset and freshness evidence; platform driver/live adapter requires binding | NOT_RUN |
| Resource and endurance | Native CPU/RAM/P95 latency, input/output rates, power, temperature and long-run stability on 4 GB board | NOT_RUN |
| Front and down modes | Independent calibrated mono configurations, actual down-facing tracking, deliberate loss/reset/contact-recovery tests; non-overlap fusion stays refused until supported | NOT_RUN |
| Flight-controller integration | Physical clock/frame/lever-arm and reset-counter verification, real ODOMETRY receiver and separately authorized safe integration; mock/SITL is insufficient | NOT_RUN |

Only a real RK3576 sustained test may use **REAL-TIME ON-BOARD VERIFIED**.
Software gaps above must be closed separately before stating that only hardware
bring-up remains.


The fresh input-boundaries 100 ms camera-delivery-delay replay completed as
`EXECUTION_COMPLETED / ARTIFICIAL_DISTURBANCE_PER_EPOCH_OBSERVATION`, not a
continuous-baseline or recovery PASS. In
`runs/camera_delivery_delay_100ms.half-input-boundaries-delivery-analysis-v1/`,
all four processes exited zero and owned-container absence was verified. The
2,912 messages from each camera and 29,120 IMU messages matched the source
values; each camera retained its one declared source-time inversion. The native
guard explicitly rejected one late old camera, retaining 2,911 poses and 2,870
GT associations. Fixed-scale ATE was 0.0631810406 m; one-second RPE was
0.0432247176 m / 0.4640240861 degrees. Its run manifest SHA256 is
`8345f785c9fa9aa83ab794383766b234c69ca16f36a4d0237da908035a3c12e4`.
The native-state evidence interface passed with 2,906 accepted corrections,
five invalid health transitions and zero evidence errors. Its 29,109 complete
independent-snapshot propagation rows include 29,069 usable rows at 200 Hz
sensor cadence; they are not new visual updates. This is one diagnostic run,
not a five-member repeatability cohort or proof of detected fault recovery.
Read-only reanalysis found the first projected tracking loss 16 seconds after
the declared delay interval ended, followed by two visual corrections beginning
16.050000128 seconds after that end. These observations are retained, with
`fault_interval_association=NOT_OBSERVED`; they cannot close the fault-recovery
gate. A current strict audit also detects the later runtime-source hash change
and retains FAIL for that current-code audit rather than relabelling the old
run. The sealed original execution result is unchanged.

The preceding same-input run remains FAILED / DATASET_INVALID. It exposed
prepared-format `EUROC` versus historical sensor-clock `EuRoC` naming, then an
ordered timestamp association incorrectly applied to delivery order. The host
now validates explicit purpose/label correspondence, binds exact original
integer image timestamps to source IDs without sorting, and uses the maximum
source ID published so far for the observer-receipt lag proxy. That proxy is
not estimator queue latency. Read-only scratch reanalysis of the old raw output
verified 2,911 poses and raw/adapter scoring parity; it did not rewrite the old
run or promote it to current PASS. Follow-up regressions reject floating source
timestamps and preserve the explicit automatic-reinitialization configuration;
configured recovery is still distinct from observed reset events.

Sealed recordings now expose `recording_transport_status` separately from metric
initialization and tracking. A PASS requires complete publisher/observer
reconciliation, frozen recording/install/runtime checks, all four clean exits
and verified cleanup. Missing poses still fail the estimator interface;
reference-free accuracy and reproduction remain NOT_RUN. Eleven cases first
failed and then passed after this separation. The native front/down sessions were actually executed, with transport PASS
and empty native trajectories as detailed below; logical camera names do not
prove real down-looking performance.

The input-boundaries derivative has now actually built 25 packages in the
isolated ARM64 Bionic/Melodic reference at 4 GiB / 4 CPUs. AArch64 ELF and 238
runtime libraries were checked; original reference sources/rootfs stayed
unchanged. This is ARM64 BUILD VERIFIED. The current derivative core-only replay and three lifecycle probes now pass
under QEMU, as detailed below; the separate host-state-policy replay remains NOT_RUN. The PC-only
same-factor observation variant likewise builds all 25 packages and has a
hash-checked persistent installation. Its bounded matrix decoder tests do not
establish a monocular fix; the complete monocular observation replay
retained zero poses and is FAIL, as detailed below.


The corrected full 50 ms IMU-gap replay, with automatic reinitialization explicitly
disabled, completed all four processes and cleanup without the former assertion.
It detected `input_invalid_imu_gap` at camera 400 and reset once. Only the 400
pre-gap poses remain, with 398 accepted corrections, 27,624 explicit invalid
health observations and zero evidence errors. Their single scored epoch has
ATE 0.0269504871 m; this pre-fault result does not describe full-sequence
availability or recovery. The run SHA256 is
`863bba39164db30915694875dfc003ed6d69d7596836a2618cfcdd904c07fbc5`.
Read-only fault-window analysis associates the loss exactly with the declared
fault onset, but finds no two continuous accepted recovery corrections. Its
recovery acceptance remains NOT_RUN; no post-gap success is inferred from
pre-gap numerical accuracy or continued invalid snapshots.
The original automatic-reinitialization group with the full host state policy
remains separate and **BLOCKED / STORAGE**. An independently declared core-only
configuration has now actually completed the same full artificial input at
0.5x with automatic reinitialization enabled. It deliberately omits
`state_output_policy`; it neither replaces that original group nor closes its
operational recovery gates.

For this core-only run, cam0/cam1 each reconciled all 2,912 images and IMU
reconciled all 29,110 samples at publisher and observer. The declared 50 ms
removal window dropped exactly ten original IMU samples; its actual largest
remaining sample interval was 55,000,064 ns. Native per-sample callback payload
equality remains unobserved. All 37 outputs, 16 runtime sources, 946 native
sources, 527 devel files and eight patches matched; archive, GT and calibration
hashes matched. Native observations and both raw/adapter scores were recomputed
in memory with exact parity. All four processes exited zero, container absence
was verified and the sealed run was unchanged. Run SHA256:
`9136b629a0107809d8219ba36ce4054ad4af259057ab2cc9e1eabbb22a74ceb2`.

Camera 400 explicitly detected `input_invalid_imu_gap`; camera 401 exercised
`input_waiting_imu_coverage`; camera 402 completed a new calibrated-stereo
bootstrap, and camera 403 accepted a new visual correction. The two non-startup
reset calls are retained separately. Native epochs 1/3 correspond to adapter
epochs 0/2; these IDs have different definitions and are not renumbered to hide
resets. Detection-to-reinitialization was 0.1 s on the sensor clock;
detection-to-first-accepted-visual-correction was 0.150000128 s on that clock
and 0.202070693 s on the native steady clock. The last pre-gap and first
post-gap accepted corrections were separated by 0.2 s of sensor time. These
are observed offline boundaries, not safe flight timeout limits.

| Adapter / native epoch | Poses / GT associations | Native accepted corrections | ATE RMSE (m) | 1 s RPE (m / degrees) |
|---|---:|---:|---:|---:|
| 0 / 1 | 400 / 379 | 399 | 0.0269504871 | 0.0253378760 / 0.417968079 |
| 2 / 3 | 2510 / 2490 | 2509 | 0.0461643474 | 0.0460655960 / 0.522196694 |

Each epoch uses the unchanged scale-1 SE(3) policy; no combined-epoch score is
reported. Output coverage is 2,910/2,912 (99.93131868%), missing cameras 400/401.
Native quality Good counts are only 398/2,505; accepting a visual constraint
does not itself establish a healthy state. All default adapter states remain
UNKNOWN/invalid. Full host-state projection, watchdog, propagation, health
recovery and flight-controller output are **NOT_RUN** for this configuration.
This is **PC OFFLINE VERIFIED** native numerical reinitialization evidence,
not continuous-state or live-VIO qualification. The read-only audit is retained
under `.agents/runs/schurvins/state-flight-mock/imu-gap-50ms-autoreinit-core-audit.json`
with SHA256 `abb5f4e0bf2cf90ebb306e2b860afd320bfe4b706e7dd2118c4d19233cd00150`.
The overall pre-hardware verdict and required-cohort acceptance remain PARTIAL.


The two sealed recording sessions were also actually executed in order. Each
logical front/down source delivered exactly 161 images and 1,602 IMU samples;
all four processes exited zero, hashes matched and owned-container absence was
verified before the second session started. Both transport statuses are PASS;
both estimators produced no pose and retain NATIVE_OUTPUT_EMPTY, interface
NOT_RUN and overall PARTIAL. Source epochs and calibrations remain separate,
without trajectory stitching. These EuRoC-derived logical IDs do not prove a
physical front/down camera arrangement, metric mono reproduction or real
sensor acquisition. Session SHA256:
`8828e9c686ed881ebb2ae42b4d9f4a4c089c60ecc6cdbba1aa59977f58962c8b`.

The only pre-hardware matrix now explicitly uses the input-boundaries candidate
with the same 75 gates / 67 required cohorts. Fifty-two Schur templates change
only installation, run ID and output; all source/GT/rate/policy requirements
stay fixed. Nine OpenVINS and six rejection configurations are unchanged.
All 52 source/configuration preflights passed. Fresh contract execution passed
516 tests after the fault-window, qualified-clock and propagation-reporting corrections; six
invalid-input gates actually refused their inputs. The selected
result remains PARTIAL: 14 PASS, 62 NOT_RUN, including 61 required native cohorts
not yet run under the new protocol. Four original ARM runtime receipts remain
historical; current derivative core-replay and expected-failure lifecycle
receipts close their functional execution/cleanup roles only. The core replay
did not request the host state policy and does not close operational-state
acceptance. The derivative build receipt is current. Matrix SHA256:
`3ba5107a2e924399ad4ea7b518540410fa654c31abc8f5bdd41e1039bf6e88f1`.
Independent smokes with different state policies cannot close these cohorts.


The full UZH-FPV `indoor_forward_3` and `indoor_forward_6` mono replays now
have actual negative results, rather than merely dataset intake. The runs
`schur-uzh3-mono-half-input-boundaries-v1` and
`schur-uzh6-mono-half-input-boundaries-v1` reconciled every input: respectively
2,552 cameras / 46,053 IMU and 1,970 cameras / 34,815 IMU. All four processes
exited zero, owned-container absence and all 37 output hashes per run were
verified. Each observed one metric initialization, but only 35 and 13 poses,
all before public-GT coverage: zero scorable epochs, both **FAIL /
TRAJECTORY_NOT_EVALUABLE**. No fitted scale, timestamp shift or missing
trajectory was supplied. Run SHA256 values are
`72af9d3a1bbb8751c90db1ae20b8570efaf25743d0c9d2e023b7813fc7722bed` and
`5294900a0300b406522c45ab66f7d39440f1dc53d7c1774aed02ee125c08bde8`.
Camera precompensation remains NOT_ESTABLISHED; these failures do not qualify
high-maneuver tracking or recovery.

The bounded same-factor mono diagnostic also completed: 2,912 camera /
29,120 IMU records matched; all four exits, cleanup and output provenance
were checked. It retained two metric initializations, eight complete factor
snapshots and zero poses (**FAIL / NATIVE_OUTPUT_EMPTY**). Its manifest is
`4f7417a1469aa812a2db7d94ef511fc3062095627988ca170f16fe4bb91fc7d8`.
Production-factor reconstruction passed; one finite-prior numerical diagnostic
remains above its original tolerance and remains FAIL. Neither that observation
nor an unconstrained point solution justifies replacing the upstream landmark
prior or Schur equations.

An independent mono-only point-update-order candidate subsequently ran the
complete V101 sequence. Updating original finite-prior landmarks before the
existing outlier gates rescued the same-run camera-155 inlier population
from 39 to 76 and camera-159 from 39 to 42. Camera 160 already had only 38
pre-update inliers, and retained 27 after the update, below the unchanged
40 threshold. It lost tracking after five poses over 0.2 seconds (5/2,912
input cameras), **FAIL / TRACKING_LOST**; this is not a mono reproduction PASS
or a promoted estimator repair. All four exits were zero, input readback and
38 output hashes matched, and owned-container absence was verified. The
manifest is `c744a71c8826bba0c413050d56d6ba5e7c62447d1a85db2d234985be2b852527`.
Its numerical subtrajectory is diagnostic only; a short-span ATE cannot
establish continuous scale or accuracy.

A separate bounded point-lineage observation run subsequently completed the
full V101 input with the same original gates and estimator mathematics. It
produced only three poses at cameras 158–160 and remains **FAIL /
TRACKING_LOST**. All four processes exited zero; container absence, all 36
outputs, 16 runtime sources, 947 native-source and 527 devel hashes matched.
The run manifest SHA256 is
`b051e0da97153aa38c00cb3c845fc67b51c4a30cb77c233c304817b14e2f07c6`.
Nine updates emitted 8,595 point rows with no cap truncation. Eight complete
same-point transitions were observed; the final missing next-frame observation
is UNKNOWN, not a match failure.

At the first loss, camera 161 had 112 registered points and 81 that met the
existing three-observation maturity gate. Thirty-two actually entered Schur;
15 survived the current 4 px gate. Forty-nine mature current points were
excluded because their prior local-window status was false, and only three
survived: 18 total, below the unchanged minimum 40. The same 66 point IDs
rejected by the camera-159 local 3 px gate were excluded at camera 160; the
same 84 camera-160 rejections were excluded at camera 161. All observed
CheckStatus failures were due to fewer than three local observations, with
none due to the 1,000 m norm gate. Current-observation removal did not globally
delete a landmark: subsequent successful rematching of the same IDs was
observed. This establishes a constraint-starvation chain but does not establish
why the all-window residual first became inconsistent. The trace does not
contain all-window before-update and camera-only residuals, so bootstrap
conditioning, linearization and prior consistency remain unresolved software
work. There is no evidence for altering the prior, relaxing thresholds or
promoting the point-order candidate. The three-pose and five-pose runs used
different time-seeded geometric bootstraps; their counts do not establish a
regression caused by observation instrumentation.


A further read-only numerical check reused the existing same-factor run rather
than starting another estimator. Its 39,729 events decoded into eight complete
snapshots containing 565 captured production factors. Using the pinned source
projection and camera-correction chart, central differences at 1e-6 and 1e-7
matched the saved camera/landmark Jacobians; the maximum relative errors were
1.50081e-9 and 3.37520e-9. All eight snapshots had zero current-observation 4 px
classification flips between the linear prediction and the actual finite
camera update. In the first captured call, 84 of 86 current observations were
inliers before the update, and both the linear prediction and actual camera
update retained only one. Thus this captured camera-only collapse is already
present in the linear model; it is not explained by a sign/chart mismatch or
finite-step threshold flips in these factors. This does not establish the
correctness of the bootstrap prior or cross-covariance.

The maximum native-versus-equivalent-same-P state-update difference was
1.05714e-8; the maximum projected-pixel difference was 4.63898e-6 px. These
are measurements only: the original finite-prior 1e-8 diagnostic remains
**FAIL**, without threshold changes or receipt rewriting. No conclusion is
made about the all-window 3 px gate or the different point-lineage run, which
has a different geometric bootstrap and no matching matrix snapshots.
The bounded reproducible check is
`.agents/runs/schurvins/state-flight-mock/alignment-review/same-factor-candidate/native_geometry_exclusion.py`;
its `native-geometry-exclusion.json` receipt is SHA256
`bbc12318192cba80a544eddb67ede6476ecabd94f022e2532777446bbf6e087f`.
Eight sealed input hashes remained unchanged and output overwrite was refused.
This narrows causal hypotheses; it does not repair or qualify monocular VIO.


The current input-boundaries derivative also passed a complete ARM64 V101 stereo
0.5x replay under the 4 GiB cap: 2,912 poses / 2,871 GT associations, fixed-scale
ATE 0.05564109464 m, one-second RPE 0.04356541945 m / 0.4514006965 degrees.
All original inputs reconciled, all four processes exited zero, container
absence and 17 runtime sources / 41 output hashes were verified. The cgroup
peak was 585,478,144 bytes with zero OOM events, measured in this QEMU run only.
The run manifest SHA256 is
`1ee783855659ed425bc5d85b4e9904bdf2a8241ebb470ba25465113390bf986b`; the
receipt is `arm64/acceptance/input-boundaries-core-current2/receipt.json`, SHA256
`9573b3b1a63a7f532a3de4b5b86a527cab1fdc4e07a2a16532b430f4dc4d4b8b`.
This separately declared core-only group omits the host state-output policy to
bound storage. State projection, watchdog, high-rate propagation and operational
validity are explicitly **NOT_RUN / HOST_STATE_POLICY_NOT_REQUESTED**, with
unknown counts left null. The original state-policy configuration subsequently
ran as the separate full-state group recorded below. This is **EMULATED / MOCK VERIFIED** numerical
functionality, not RK3576 BSP/ABI, live VIO or target performance qualification.


The same derivative ARM64 source freeze passed three actual lifecycle probes:
timeout, host SIGINT and termination of the exact owned native QEMU process.
Each preserved the intentionally failed estimator run, reaped its controller
and verified container absence and absence of the observed owned PIDs. Native
child failure was recorded as exit 137; SIGINT individual child exits remain
UNKNOWN because container removal preceded their native report. Receipts are
under `arm64/lifecycle-probes/{timeout,sigint,nativechild}-input-boundaries-current2/`.
These close the corresponding emulated cleanup checks only, with no recovery,
state-validity or hardware qualification.

### Resumed full-state validation after storage expansion

Additional disk space resolved the storage blocker; the original blocked
receipt and all older core-only runs remain unchanged. The original 50 ms IMU
gap configuration with automatic reinitialization and the full host-state
policy now completed. All four processes exited zero; container absence,
39 output hashes and 16 runtime source hashes were verified. Its manifest is
`e1bc33e3527914fe9f149f614d5e7ab69d41b45d75ec8ce0538748651878c51b`.

The independent strong recovery audit passed **PC OFFLINE VERIFIED**: camera
400 detected the fault; cameras 406 and 407 supplied two consecutive healthy
accepted visual corrections in native epoch 3, with healthy intervening Info,
watchdog and IMU outputs. Qualified recovery began 0.30 seconds after detection,
0.25 seconds after the declared gap ended, in the sensor clock. State evidence
passed with 2,906 corrections, 17,825 health rows / 17,385 usable, and zero
evidence errors. Propagation remains **FAIL** with all 22 gap rejections retained:
29,099 complete rows / 29,048 usable. It neither refreshes visual time nor
establishes continuous localization across reset epochs.

The two epochs remain separately scored: 400 / 2,510 poses, 379 / 2,490 GT
associations; fixed-scale ATE 0.0269504871 / 0.0694828735 m and one-second RPE
0.0253378760 / 0.0467639137 m, respectively. The full-state audit receipt is
`.agents/runs/schurvins/state-flight-mock/imu-gap-50ms-autoreinit-state-audit.json`,
SHA256 `2ca6ec27bc9e72f12293b18e0f4b0c07dea32aa66c1ab71aaed21c55a738e842`.
This result does not replace the original matrix's automatic-reinitialization
false configuration: that configuration also completed, detected the gap, and
produced only the 400 pre-gap poses without recovery. Its strong gate remains
NOT_RUN under the existing audit semantics. The selected result is under
`prehardware_storage_resumed_fault_imu50ms_v1/`; unselected cohorts remain
NOT_RUN. These independent snapshots are not a combined acceptance result.

The original ARM64 full-state configuration also completed under the 4 GiB
cap. Its manifest is
`f551aae782c289305de0fc174bb6c3ce08e654607146f4d6ce92295ac9f27585`;
`arm64/acceptance/input-boundaries-state-current2/receipt.json` is SHA256
`d6ab50dcd2b17370f5ed9216d3a0967d4dad55ad21a54c4eccd9c88dd7bec7dd`.
All 43 outputs / 17 runtime sources matched, all four exits were zero and
container absence was verified. There were 2,912 poses / 2,871 GT associations,
fixed-scale ATE 0.0430221 m and one-second RPE 0.0440174 m / 0.477105 degrees.
The cgroup peak was 577,675,264 bytes with zero OOM events. Candidate-state
evidence passed with 2,908 accepted corrections, 17,766 health rows / 17,289
usable and zero evidence errors. Propagation passed with 29,102 complete rows /
29,074 usable and zero rejections; the independent watchdog's 14,854 ticks
passed completeness checks. Native-quality rejections and invalid health
transitions remain visible. This is **EMULATED / MOCK VERIFIED** functionality,
not continuous availability, RK3576 BSP/ABI or board performance qualification.

The sole matrix now refers to this full-state ARM receipt. Only that gate's
receipt path and receipt hash changed; 75 gates / 67 required cohorts and the
old core receipt are preserved. Matrix SHA256 is
`3937d2626e0a3f5ab6229db2f356e104fb018bc03ee5c201df580478f700ee24`;
the explicit diff is recorded in
`.agents/runs/schurvins/prehardware_input_boundaries_migration/arm-state-receipt-migration.json`.

### Snapshot and queued IMU callback correction

The first resumed matrix V101 stereo smoke completed numerically: 2,912 poses,
fixed-scale ATE 0.06833073869 m and one-second RPE 0.04293018069 m. All four
exits, source reconciliation and cleanup passed. Its stronger state gate
failed on 11 `PROPAGATION_IMU_REJECTED` evidence errors, so the five fixed
repeats remained NOT_RUN. This original result under
`prehardware_storage_resumed_v101_stereo_half_v1/` is preserved.

The same recorded callbacks show correctly ordered 5 ms IMU samples arriving
after a newer visual snapshot. The tracker had treated either a second older
sample or a sample outside the snapshot's anchor-age bound as a source rewind.
It now validates the original IMU order, sequence, frame and clock as before,
and retains the newest sufficiently recent pre-snapshot anchor. Older queued
samples do not modify or reintegrate the corrected state. Duplicates, source
rewinds and gaps still latch invalidity; the existing limits are unchanged.

Seven regression cases cover multiple pending samples with and without an
existing anchor, duplicate/rewound/missing samples and the full native-state
projection path. Five tracker cases failed before the repair. The affected
tests passed 208 cases; the full Jazzy-sourced suite passed **736 tests with
one skip** in 15.38 seconds. A fresh recorded contract run passed 523 cases.
Read-only reprojection of the original smoke now has zero evidence errors and
zero propagation rejections, with 29,109 complete rows, 2,908 corrections and
four retained health transitions. All original artifacts remained unchanged;
this diagnostic does not rewrite the original FAIL or replace fresh replay.
Its receipt is
`.agents/runs/schurvins/state-flight-mock/snapshot-queue-reanalysis.json`, SHA256
`40ae37084e71e61154a30107336a57adcb2898e87ee1136c984fa8cede0a365d`.

The host state-output source changed. Earlier ARM64 and fault-state receipts
retain their exact-version results; they cannot close current-code gates until
their recorded source identities match. The sole matrix's contract receipt
was explicitly refreshed; matrix SHA256 is
`f3fdde83fa2c996918aba73a507dffd95577cf3f286042475f1cbcbf73bd1eaf`.
The unchanged stereo configuration is being rerun in a fresh output group;
its completion is not assumed here.


### Current-code V101 stereo fixed repeat cohort

The fresh full V1_01_easy stereo 0.5x cohort completed: one smoke and the five
predeclared repeats all passed numerical replay and candidate-state evidence.
Every member produced 2,912 poses with complete camera coverage. All 24 native,
player, observer and master exits were zero; all six exact-name containers are
absent. Each member's 39 output artifacts and 16 runtime source identities were
rechecked. Code bytes are identical across the six runs; run-local absolute
runtime-tool paths explain the summary's distinct provenance labels.

| Member | Fixed-scale ATE (m) | One-second translation RPE (m) |
| --- | ---: | ---: |
| Smoke | 0.0559019483 | 0.0432128589 |
| Repeat 1 | 0.0623801896 | 0.0430125570 |
| Repeat 2 | 0.0429328848 | 0.0428428227 |
| Repeat 3 | 0.0523156452 | 0.0426186037 |
| Repeat 4 | 0.0469065262 | 0.0427873927 |
| Repeat 5 | 0.0605780504 | 0.0434407593 |

Five of the ten repeat pairs exceed the unchanged 1 cm ATE-difference diagnostic;
that result remains FAIL. Numerical/state-integrity success does not establish
deterministic repeatability or continuous trusted availability. Every run retains
three explicit invalid native-quality health transitions. Propagation and
independent watchdog evidence passed; propagation does not refresh visual time.
Native GNU-time peak RSS was 89,444--90,104 KiB across all six members. These
0.5x, potentially shared-host measurements are not RK3576 timing or an exclusive
host comparison against OpenVINS.

Evidence is `prehardware_snapshot_queue_v101_stereo_half_v1/` under the external
research root. The immutable audit is
`.agents/runs/schurvins/state-flight-mock/snapshot-queue-stereo-cohort-audit.json`,
SHA256 `4cc4b941acaca22d60dea38026db9cc780fa63bd9655e8fe82c9c3f4af206ac5`.
The selected acceptance snapshot reports 2 PASS / 0 FAIL / 74 NOT_RUN and remains
PARTIAL. It cannot be combined with separate selected snapshots to assert all
required cohorts have passed.

The state-output repair was also exercised by fresh offline/MAVLink loopback
and isolated SITL runs. The receipts are
`.agents/runs/schurvins/state-flight-mock/snapshot-queue-qualified/receipt.json`
(SHA256 `004f0b64b6532666c1864c55daf779b902c45d1baefd697dc552c5a0689821fe`)
and `state-flight-mock/sitl/snapshot-queue-verified/receipt.json` under the same
`.agents/runs/schurvins/` prefix
(SHA256 `1a3653e7219071747c9a663fbbe280e0fcd9161b13b7a5b86827885d416f125c`).
Both passed current-source artifact checks and actual process cleanup. The mock
received one qualified packet and suppressed the stale/invalid output; SITL
received the two nontrivial states and reset transition 17 to 18. An earlier
wrapper's mistaken three-packet expectation is retained as a wrapper-validation
FAIL, without changing the original scenario. These are **EMULATED / MOCK
VERIFIED** interface results; PX4 timesync and real VIO fusion remain NOT_RUN.

Only the two flight-interface receipt paths and hashes changed in the sole
matrix, now SHA256
`f526df7434ad3b49c723ef020e7e7a1c10e8de4e1d7bc0fa71f14e411b1e773f`.
The explicit migration is
`.agents/runs/schurvins/prehardware_input_boundaries_migration/flight-snapshot-migration.json`.
The four selected contracts/mock/SITL/ARM-build evidence gates pass; all other
scope remains declared and unselected. Old ARM runtime receipts still identify
the earlier host state-output version and do not close current-code gates.


### Latest bounded monocular diagnosis and ARM64 replay (2026-11-01)

The full outside-window diagnostic remains **FAIL / TRACKING_LOST**, with only
two poses at original camera indices 1887–1888 (0.05 seconds). All four processes
exited zero and the owned container is absent. Run SHA256 is
`f85e2626d6e6cfa53219336a8b4cb25c9c6502cfa91e7b7c204f1027249ed402`.
The two-point ATE is not a successful monocular reproduction.

The bounded observation reconstructs the recorded geometry and actual point V/g
increments in 15 complete three-stage windows: 86 actual updates and 298 outside
observations. Five windows lack the middle stage, and the complete native point
population was not independently counted. Accordingly the analysis stays FAIL,
with those omissions explicit. Its budget covered the first initialization epoch,
not the second epoch's final tracking loss. Evidence is
`state-flight-mock/alignment-review/full-window-candidate/outside/native-outside-diagnostic.json`
under `.agents/runs/schurvins/`, SHA256
`0c51406095fe4c7eea4a8416e29b69d80d28c8803546f0900bec05e7ecb2e5fa`.
All 13 decoder tests pass; these fixtures do not substitute for native coverage.

Source and saved-run review confirm different camera and landmark information
objectives, with historical observation reuse in successive landmark updates.
A common quadratic objective alone would assume an independent landmark prior
and fixed outside frames; existing camera–landmark correlations and conditional
covariance cannot be ignored. No estimator repair has been promoted, and these
findings do not establish the sole cause of tracking loss.

The fresh full ARM64/QEMU V101 stereo run with the snapshot-queue state fix
passes numerical and candidate-state interface checks: 2,912 poses, 2,906 accepted
corrections, 17,741 recorded states (17,266 usable), 29,102 propagation rows with
zero rejected samples, and 14,829 watchdog ticks. Under the 4 GiB cap the recorded
peak was 577,347,584 bytes. All four process exits are zero and the container is
absent. The receipt's 51 artifact hashes and 23 source identities were rechecked.
Receipt: external `arm64/acceptance/snapshot-queue/receipt.json`, SHA256
`5bc8b589acf87827094533ea861310d9cbc022ae32ae2f4eeb08ab635db876e2`.
This is **EMULATED / MOCK VERIFIED** functionality; invalid health intervals
remain invalid. It does not qualify RK3576 BSP, ABI or real-time performance.

The separate OpenVINS native-observation candidate builds successfully in the
pinned Jazzy environment (three packages), passes its ten helper cases including
sanitizers, and resolves its x86-64 ELF dependencies. Its final audit is
`state-flight-mock/openvins-native-observation/final-audit.json` under the same
evidence root, SHA256
`2863240936885ac202de34beeb54c731ccd23cbe6d2f14ec05ed82fe79eaa760`.
It remains unpromoted: actual accepted-feature measurement-time support,
initialization-time snapshot concurrency, native replay and health integration
need completion. Current OpenVINS projections remain UNKNOWN/invalid.

The subsequent accepted-measurement-support candidate completes those build and
observation-replay steps without changing the canonical installation. Its three
packages build successfully; four x86-64 ELF artifacts and their dependencies
resolve. The independent installation is `openvins/installation-native-support.json`
under the research root, SHA256
`d423ced9ab0dbe44d22b6939ef337f442e9f18f1234683d17bb55b2dc3d9ec64`.
The existing comparison runner enables recording only when this explicitly
selected installation declares the supported observation schema and bounds.
Canonical commands remain unchanged. Clean native exit and complete bounded
footer are required before decoding and sealing the observation report.

Its full V101 mono 0.5x replay completed: 2,800 poses / 2,780 GT associations,
fixed-scale ATE 0.06201257593 m and one-second RPE 0.04533300310 m /
0.4847512917 degrees. All five recorded pipeline/resume exits are zero; the
container is absent. All 49 outputs and 17 runtime identities were rechecked.
Run SHA256 is
`1ae18ca687726df0bb01bebf2918513b456081c15463a0c49122730a79a7b691`.
The trace records one metric-initialization commit, 11,061 actual accepted
visual updates and 2,800 advances of the newest accepted measurement time.
The decoder associates accepted feature timestamps by exact double bits to
earlier integer ROS callback stamps. Reused older observations, propagation,
returned states, ZUPT and independent ticks never refresh visual time. Corrected
state time remains separate from the accepted measurement time. The production
decoder/runner tests pass 100 cases; the full Jazzy suite passes **769 tests,
one skipped**, with no hardware qualification.

Crucially, this new visibility exposes a native-input completeness failure:
the publisher and observer have 29,120 IMU samples, while native callbacks have
28,946. The 174 missing samples occur in 16 bursts; all present callback values
match the original observed samples exactly. Its input completeness remains
**FAIL**, with cause under investigation. This is not a qualified equal-ingestion
algorithm comparison, and health/reset qualification remains **NOT_RUN**.

The current comparison runner now verifies each native IMU callback against the
independently reconciled observer: original integer seconds/nanoseconds, sensor
identity, two finite three-axis vectors, values and order. Missing samples are
reported as complete intervals and fail the run with
`OPENVINS_NATIVE_INPUT_INCOMPLETE`, retaining its numerical score. Duplicate,
reordered, unexpected or mismatched callbacks are rejected. No samples are
interpolated. This only qualifies the native callback boundary, not downstream
integration or image-pixel ingestion. Read-only analysis of the sealed run with
the current helper reproduced all 174 missing samples / 16 intervals without
changing its old status or artifacts; audit SHA256
`f558e6e9b126f639e8fed9e38e06cfeda23b05da73b3a322b9790c5d29a06e29`.

The native-input audit found small BEST_EFFORT IMU buffering and independent
timer gaps correlated with the missing bursts. It did not establish camera
processing or CPU throttling as the unique cause. A separate offline candidate
changes only the IMU subscription to RELIABLE / KeepLast(10000). Its three
packages actually built; 442 source files, 448 installation files and four
x86-64 ELF/dependency closures pass audit. The canonical and previous candidate
remain intact. Installation `openvins/installation-offline-reliable.json`, SHA256
`5cb64082c8854ea06766b91b1c00ee0f4334ffc400029869e033dca4428345b3`;
build audit SHA256
`f10f35d1b664a4a225a3e9a148be3384a94bf2422e60ff488683c0ac5f1bf62c`.
The full native V101 mono 0.5x replay has now completed. Original ZIP,
independently reconciled observer and native IMU callbacks agree on all **29,120**
samples: integer timestamps, order and all six values, with zero missing samples.
All 29,120 buffer-feed completions are observed; downstream integration
completeness remains NOT_RUN. Camera callback stamps/order agree for 2,912
frames; native pixel ingestion remains NOT_RUN. The run has 2,800 poses / 2,780
GT associations, fixed-scale ATE **0.0701150108 m**, and one-second RPE
**0.0452370974 m / 0.4780522904 degrees**. All five child exits are zero,
the exact owned container is absent, and 49 output / 17 runtime / 448 installation
hashes pass independent audit. Run SHA256:
`aa20d1f3176b9ace0654db06cff98f148bf3bb6ca930737ec6dc2ac55e7e0a70`.
Audit: `state-flight-mock/openvins-native-observation/offline-reliable-native-audit.json`,
SHA256 `45d616fafcf1b536f737de14562928c9faed7a92986ff9e22286437a3453815d`.
This is **PC OFFLINE VERIFIED** at those boundaries. Health/reset qualification,
repeatability and equal-ingestion algorithm ranking remain unverified; the old
174-missing-sample failure is preserved.

After the callback-coverage changes, affected tests pass **120 cases** and the
full Jazzy software suite passes **789 tests, one skipped**. Three malformed
observer-shape cases failed before the shape fix and pass afterwards. The
fresh contract receipt passes 633 cases, SHA256
`5c4131c7fc43fb64bb796e48672ef82845c0b4577bc2694a3ca8ee5928acb859`.
Only its `sensor-contracts` receipt was explicitly migrated in the sole matrix;
its SHA256 is now
`22a7d2b917918a8c51604186d594a5c259c8672509f1beb77b11e8555dbb954c`.
Older receipts and selected snapshots retain their original identities.

The audit is `state-flight-mock/openvins-native-observation/measurement-support-native-audit.json`
under the existing `.agents/runs/schurvins/` task root, SHA256
`1768fbb8921882e7e39bf6e9c3b974ac8d531f05b6d7f004c79d37f5b40d4f62`.

### Completed current V103 stereo cohort and ARM lifecycle checks

The current full V103 stereo 0.5x smoke and five predeclared repeats all pass
numerical replay and candidate-state evidence integrity. Each has 2,149 poses /
2,094 GT associations, all four exits zero and verified container absence.
All 234 output hashes and 96 runtime identities match; code bytes agree across
the six members after accounting for run-local absolute helper paths.

| Member | Fixed-scale ATE (m) | One-second translation RPE (m) | Invalid health events |
| --- | ---: | ---: | ---: |
| Smoke | 0.0529379974 | 0.0345561538 | 53 |
| Repeat 1 | 0.0768072196 | 0.0349843284 | 60 |
| Repeat 2 | 0.0614595574 | 0.0360043805 | 54 |
| Repeat 3 | 0.0908870644 | 0.0354089982 | 51 |
| Repeat 4 | 0.0903294250 | 0.0357897447 | 51 |
| Repeat 5 | 0.0598132803 | 0.0375893795 | 52 |

Eight of ten repeat pairs exceed the unchanged 1 cm ATE-difference diagnostic:
**FAIL**. No failures or invalid native-quality transitions were hidden. The
audit is `state-flight-mock/snapshot-queue-v103-stereo-cohort-audit.json`, SHA256
`2deeb1683ea1a528340037ad9d180db90fec0b44cc13d28093e7a356e392b8f6`.
These are shared-host PC offline results, not RK3576 performance or uninterrupted
trusted availability. The selected snapshot remains **PARTIAL: 9 PASS / 67 NOT_RUN**.

The snapshot-queue ARM64 timeout, SIGINT and exact-owned-native-child failure
probes also actually completed. Their expected-failure cleanup receipts pass;
the failed estimator runs remain failed. SIGINT individual child exits remain
unknown where the native report was unavailable. The sole matrix explicitly
migrated its four ARM receipt paths/hashes, retaining 75 gates / 67 required
cohorts, to SHA256
`2d9e20b57a072e812f5a5307a33201de42786d77ba64c77744e5a2dd5bd59a60`.
Separate selected snapshots cannot be combined into a full-scope PASS.

### Mono first-failure observation boundary

The full observation-only V101 mono run
`v101-mono-failure-window-p0-diagnostic-half` completed with all four child exits
zero and verified container absence, but remains **FAIL / TRACKING_LOST**:
only four poses, bundles 155–158. Its run SHA256 is
`c20a0d8b8524500155f63aac53552c0fbfb147b240a03d9f5a26902665be6a6a`.
The deferred decoder independently reconstructed three complete windows
(155, 156, 160), 226 actual point updates and zero truncation. Observation
reconstruction PASS does not qualify the estimator.

The causal audit distinguishes the true first quality failure at bundle159
(101 to 56 inliers; a drop of45 exceeds the unchanged40 limit) from the first
below40 result at bundle160. Actual point updates recover13 and7 inliers,
respectively, so they are not established as the loss cause. Frame/clone
transforms agree after the update; the bundle160 pre-update mismatch follows
the frontend pose overwrite after159's quality failure. The first-two-update
observation selection missed159's full window and the four-matrix cap missed
its solve. The bounded first-eight-window / later-four-matrix candidate has now
been built and run over the complete sequence. It changes no estimator equations,
thresholds, truth or adapter scale, but remains **FAIL / TRAJECTORY_NOT_EVALUABLE**:
one pose at camera176, all four exits zero and verified container absence.
Run SHA256 `7194a2f86ad0d38356072eff5c8ef79c1d3e7ae750894e038cdb4d56a4dd89f1`.
The decoder reconstructs six complete windows (169, 170, 171, 172, 174, 175),
141 actual point updates and zero truncation. The first good window169 has
current inliers 62 before / 12 after camera update / 42 after point update;
window170 immediately has 23 / 0 / 2. Forty-three priors at170 equal169's
posterior, and 200 outside factor identities recur. These observations do not
establish a sole cause or justify removing the point update, which recovered
30 inliers at169. The matrix focus captured only174/175 and again missed the
first failure. Its successful late algebra checks cannot exclude an earlier
problem. Analysis SHA256:
`4444129085b187135da00cf6e827570fb2debb95b4b594f2a62fc22118932505`.
The next isolated candidate now uses the same frame's actual frontend quality
decision, including the unchanged maximum-drop rule, to select deferred matrix
and window buffers. It keeps the global four-matrix cap, a single bounded
6,144,000-byte matrix payload buffer and the original estimator equations,
RNG and thresholds. Exact production geometry/point-update bodies pass normal
and ASan/UBSan tests; all 68 decoder cases pass, including legacy schemas and
missing/mismatched selection evidence. The isolated build passes with exactly
five expected observation-source changes; parent source/devel files and symlinks
are unchanged. Installation SHA256:
`63e84458ea32819f4f40c2fb99b804fe4417f5b049964161861d0c7c9cc7679e`.
The complete `v101-mono-quality-linked-p0-diagnostic-half` replay has finished:
**FAIL / TRACKING_LOST**, 12 poses at camera159–171, only 0.6 seconds and 0.4121%
of source frames. Same-run raw/adapter rescoring agrees exactly; fixed-scale
ATE 0.008797718 m describes only this short retained interval. One-second RPE
has no pairs and remains NOT_RUN. Four child exits are zero, the exact container
is absent, all 36 outputs / 16 runtime / 949 source / 527 devel files and 255
symlinks match, and cgroup peak is 438,886,400 bytes without OOM.
The first actual quality failure at camera160 now has its own complete matrix
and three-stage window: camera159 current inliers 87 → camera-only5 → point82;
camera160 33 → 11 → 31. The unchanged minimum40 and drop51>40 both fail.
Camera161 returns good, so this first rejection is distinct from the later
sustained loss. Eight windows, two matrices and 536 actual point updates have
no truncation. Native IMU callback ingestion remains UNOBSERVED for this run.
Run SHA256 `04eea146efc466849697a9ad1c0379ada9098633b97b24e05f06b808084d9a32`;
independent audit `outside/quality-linked-native-integrity-audit.json`, SHA256
`fc4486c9017a4fd861a9bf38337b194eeae8adb27985c156f3ea96ea3ff1daa1`.
Observation reconstruction passes; monocular reproduction has not been repaired.
The later callback-ledger work changes `native_events.py` and
`schurvins_runtime.py` after this completed audit. The run keeps its exact older
runtime identity; it is not a regression result for those later host changes.
Causal audit SHA256
`ac615153f1120b0748d5ab75e528ec3fdefd0766c231b7e9b20d2d8909f97235`.

### Current stereo 1x cohorts and benchmark regression

The complete 1x smoke and five fixed repeats pass numerical and candidate-state
evidence checks: every run has 2,912 poses / 2,871 GT associations; all 24 child
exits are zero, six owned containers are absent, and 234 output / 96 runtime
hashes agree. The five repeat ATE values are 0.0591499976, 0.0623357259,
0.0659208866, 0.0580086045 and 0.0472656747 m; one-second translation RPE ranges
from 0.0428406533 to 0.0431868239 m. Four of ten repeat pairs exceed the unchanged
1 cm diagnostic (**FAIL**). Each run retains three to five invalid health
transitions; native IMU ingestion remains UNOBSERVED. Native frontend P95 is
2.776–3.081 ms on this shared PC, not a board timing qualification.
Audit: `state-flight-mock/snapshot-queue-v101-stereo-1x-cohort-audit.json`, SHA256
`59f55d830053fcbe02f927cc89ce7935f493de699e4019df509d062c0ac79e40`.
Its selected snapshot is one PASS / 75 NOT_RUN, overall PARTIAL.

The corresponding V103 stereo 1x smoke and all five repeats also complete with
2,149 poses / 2,094 GT each, scale 1, all 24 child exits zero and all six exact
containers absent. Independent raw/adapter rescoring agrees exactly; 234 output,
96 runtime, 946 source / 527 devel file hashes and 2 / 253 symlinks match.
The five repeat ATE values are 0.046098, 0.059568, 0.044447, 0.083807 and
0.074520 m (mean 0.061688 m; sample standard deviation 0.017311 m).
The unchanged 1 cm diagnostic fails **8 / 10 pairs**. Each member retains
51–64 invalid health transitions and 555–594 invalid recorded states. Frontend
P95 is 2.767–3.057 ms, with some maxima near 173–175 ms on the shared PC.
Propagation has no rejected samples and does not refresh visual time; native
IMU ingestion remains UNOBSERVED. Audit SHA256:
`d7977ff8edbd0c413a8465525339fb8523decb939f3df424f4b36206b3e0d362`,
at `state-flight-mock/snapshot-queue-v103-stereo-1x-cohort-audit.json`.
Its selected snapshot is likewise one PASS / 75 NOT_RUN / PARTIAL.

The existing benchmark summary now preserves sealed OpenVINS metric commits,
accepted feature support and IMU callback coverage separately from state health.
The reliable run summarizes one commit, 11,031 accepted update calls and 2,800
new measurement-time advances; legacy receipts without callback coverage remain
NOT_RUN for that field. After these changes, the complete Jazzy suite actually
passes **793 tests, one skipped**. The fresh 637-case contract receipt is
`prehardware_input_boundaries_migration/contracts-native-benchmark-support/receipt.json`,
SHA256 `360427be24fd7fc4a3e281dd6fd8a0f05c2c1ba9dc954beb9f817df3aa344865`.
Its source identities and process-group absence are verified. After the V103
cohort completed, the sole matrix explicitly migrated this contract receipt and
nine OpenVINS cohort configurations to the separately audited offline-reliable
installation. Only installation/run ID/output changed in those configurations;
raw input, truth, playback rate, scoring and repeat protocols are unchanged.
Canonical configurations, installations and sealed results remain intact.
Scope remains 75 gates and 67 required declarations (61 native cohorts plus
six input rejections). Matrix SHA256:
`a10631c0cfc58047540358a7ed5ae5fe48a87ba0374c07ed12e58d429687b122`.
The explicit migration receipt is
`prehardware_input_boundaries_migration/offline-reliable-and-benchmark-receipt-migration.json`,
SHA256 `834c5dcde68578ff55ed55f0e9bbfb85c1e051b9f29ee97810e0e911a5de0ba7`.
Fresh execution of all eight evidence gates and six input rejections gives
**14 PASS / 62 NOT_RUN / PARTIAL**, at external
`prehardware_reliable_benchmark_contracts_v1/result.json`. New OpenVINS cohorts
still need fresh sequential execution; the earlier one-off candidate run does
not substitute for their planned smoke/repeats. Snapshots are not combined into
a claim of complete software acceptance.

### SchurVINS raw IMU callback boundary

The existing native-event decoder and independent runtime now accept a bounded
callback ledger, separate from camera-update events. Each delivered occurrence
must match observer header sequence, integer sensor timestamp, frame-ID bytes
and all six floating-point measurement bit patterns, in order. Legitimate injected
duplicate or out-of-order sensor timestamps remain visible; there is no sorting
or interpolation. Missing, extra or mismatched callbacks fail coverage while
retaining independently auditable numerical trajectory scores. Legacy binaries
without the instrumented header remain NOT_RUN for this observation.

The isolated native candidate reserves storage at interface construction and
seals only after the owned camera/IMU callback threads have joined. Allocation,
I/O, capacity, truncation and unclean shutdown cannot produce a successful
footer. Its helper passes 16 normal and 16 ASan/UBSan checks; the isolated
25-package build succeeds. Only the new observation header and interface differ
from its frozen parent. Installation SHA256:
`118f48b21bff3c1eda593ebacb97599f22656585c0eeaa074ec2060fa9f4d5ac`.
The complete V101 stereo replay passes numerical and interface integrity:
2,912 poses / 2,871 GT associations, fixed-scale ATE 0.06884109063 m and one-second
RPE 0.04320886497 m / 0.46310365273 degrees. Original archive, observer and native
callback agree on all 29,120 IMU occurrences; all buffer insertions return accepted.
The complete mono replay still fails TRAJECTORY_NOT_EVALUABLE: one pose at
camera164, metric initialization at161, complete 29,120 callback coverage but
1,711 accepted buffer insertions and 27,409 explicitly paused dispositions.
Both have four zero child exits, exact container absence, unchanged source/input/
installation/output identities and exact raw/adapter rescoring. Independent
integrity audit SHA256:
`5a60451e899757bfb20a0f35b2818bc6292ca22f478dbbe227991530b7a986cb`.
Its PASS describes evidence integrity and does not override the mono FAIL.

The exact observation patch is now shipped as
`tools/schurvins/native-imu-callback-boundaries.patch`; apply it after the
input-health derivative patches. Existing frozen installations are not rewritten.
The benchmark summary preserves callback coverage even when camera observations
are NOT_RUN. Callback receipt, actual buffer disposition and duration remain
separate from downstream integration (UNOBSERVED), reset mapping and health
qualification (NOT_RUN). New ARM64 compilation is separate and still pending.

The final affected suite passes 715 cases including execution of the shipped
C++ helper's joined/unclean/capacity paths and preservation of signed-zero and
frame bytes. The complete sourced-Jazzy suite passes **843 tests / one skip**;
`git diff --check` and affected Python compilation pass. Fresh contract receipt
SHA256 `e3d439b3dcc454ab0f77fb5956e0022f186dbd89f9e2fa717896000f2cf6915c`.
Only sensor-contracts was explicitly migrated in the sole matrix, now SHA256
`9a409ce5517144e6e10c6b55890ff3404a46a5170c53a2c74799014fec1a4c80`.
The selected sensor and six rejection checks yield **7 PASS / 69 NOT_RUN /
PARTIAL**, with no native replay started by that selection. Old snapshots,
receipts and matrix declarations remain intact; this is not complete acceptance.

### Reliable OpenVINS V101 mono cohort

The smoke and all five fixed repeats complete on the same frozen reliable-input
installation. Each has 2,800 poses / 2,780 GT associations, exact same-run raw /
adapter / saved scores, ATE 0.07011501076521641 m and one-second RPE
0.04523709741003572 m / 0.4780522903551887 degrees. All six numerical trajectory
hashes match; every one of the ten formal repeat pairs passes the original
1 cm criterion with zero difference. Original ZIP, SQLite CDR, observer and all
29,120 native IMU callback occurrences agree in time/order/value bits. Every
member has one metric commit, 11,031 accepted feature updates and 2,800 advances
of visual measurement time, all five child exits zero and exact container
absence. All 17 runtime / 448 installation / 49 per-run output identities pass.
Audit SHA256 `72fd795cc433eb93667d89ce8cfa07b4a1dfab7defcd243aa5f82e2dd38649e1`.
Native camera-update P95 ranges 11.27–11.951 ms and maximum RSS 135,776–139,384
KiB on the shared PC; these are not end-to-end or board performance. Health,
reset, downstream integration and native image-pixel consumption remain NOT_RUN.
Its selected snapshot remains one PASS / 75 NOT_RUN / PARTIAL. The subsequent
ledger-summary display change gives this completed cohort an older exact summary
source identity; the original results and audit are not rewritten as new-code
regression evidence.

### Isolated finite-inwindow causal probe

The separately built NOT_PROMOTED candidate uses the same finite conditional
point normal in camera elimination and landmark back-substitution, retaining
the existing prior and excluding outside observations from its declared local
objective. It changes no gate, RNG, truth or adapter scale. The existing bootstrap
prior, camera–point/interpoint correlations and repeated factor ownership are
not statistically qualified; conditional point covariance is not a full marginal.
342 captured matrix cases and 15 boundary cases pass normal and sanitizer tests.
The combined observation build succeeds for 25 packages, installation SHA256
`2b8f3ceb61e633d0542bd75bdb689d343dd5bab77dc5fd155acfdd99b8d6ef97`.
Full V101 remains **FAIL / TRACKING_LOST**: 1,618 poses / 80.9 seconds, 55.5632%
of source frames, fixed-scale ATE 0.6282166004 m and one-second RPE 0.1469623627 m /
0.5804780393 degrees. All 29,120 callbacks match exactly; 17,832 buffer insertions
are accepted and 11,288 are explicitly paused. Four child exits and exact
container absence are verified. A longer retained trajectory than the one-pose
control is dynamic evidence, not repaired continuous monocular VIO. V202's
supplemental official ten-second offset group is running; its full sequence
and V103 remain pending, with identical declared initialization controls.


### Release serializer qualification correction

The actual upstream Release flags reproduced a signed-zero observation defect:
`std::signbit(-0.0)` can be folded with `-fno-signed-zeros`, so a callback's JSON
value became positive zero while its recorded IEEE bits remained negative zero.
The new shipped-header regression first produced one failure / one pass, then
two passes after integer-bit classification of zero and nonfinite values, with
an IEEE-754 guard. Shipped patch SHA256:
`17c61197943e92c119c3fc93054aa0f0ed87598aa29a2108ae29f0f23f27e221`.
This only repairs observation serialization; it changes no estimator math.
The upstream flags end in `-fno-finite-math-only`; this evidence does not establish
that every finite-value guard is optimized away. Prior patches, negative ARM64
results, builds, native runs and receipts remain intact. The 843-test full suite
and 715-case contract receipt above describe the preceding source version.
Full regression, fresh contract migration, exact-new-header PC/ARM64 builds and
new native regression are pending; older results are not relabeled current.

The finite-inwindow V202 full and V103 full probes have now both completed with
TRACKING_LOST. Their failures remain separate from the supplemental ten-second
V202 start-offset group. Independent complete-input/scoring/cleanup audits now PASS, SHA256
`06119a7f67ec01fc83e94b4424c64e472b781cab1e78bdf26ea83cbafc76a2ba`.
V202 full retains 62 poses / 3.05 s / 2.64055% source frames, ATE 0.213573 m;
V103 full retains 34 poses / 1.65 s / 1.58213%, ATE 0.207324 m. Each has exact
ordered native callback coverage (23,490 / 21,500 occurrences), four zero child
exits and exact container absence. Their scores describe only those short
retained intervals; no failed smoke is replaced by five selected repeats.

The exact shipped serializer PC build succeeds with 25 packages in a separate
workspace. Parent sources, devel artifacts and links are unchanged; only the
observation header differs. Build integrity receipt SHA256:
`ef58ff7b603380fe5f807823e45c94e5519cd05377d1e2a4d7e322a7a051da94`.
New full stereo and mono regression are still pending.

A read-only late-feature audit finds that camera1775 has 53 current points,
41 passing CheckStatus and 39 original-gate inliers. All 38 current points that
pass the three actual solve selectors enter the solve; three more fail local
status and 12 remain immature. Total matches are 148, below the 180 total quota.
Earlier point-level evidence ends at camera169, so it cannot attribute the late
loss to grid occupancy, pruning, promotion or geometry. A bounded 35-frame /
4 MiB survival observation is being prepared without altering gates or estimator
math. The finite-inwindow candidate remains NOT_PROMOTED and mono remains FAIL.


The complete current Jazzy regression now passes **844 tests / one skip**
(17.04 s, exit0), and the expanded current contract suite passes **716 cases**
(15.70 s, exit0), including normal and actual upstream Release serializer tests.
All 32 source identities and the owned process-group absence are verified.
New contract receipt SHA256:
`60f6113837fe940820c4181be66dec3be890761d792024d485a3c6acb2ec5c5d`.
The sole matrix explicitly migrates sensor-contracts; the other 74 declared
gates and all 67 required cohorts remain unchanged. Migration receipt SHA256:
`0fcebb6a0332016e94d1e02dbad70a46a33082c96e44b7acb0a96e63e699f73f`;
matrix SHA256 `0bd872362cfa00de104fdabcda331508decb0772241f325de08677c3e24d900f`.
The fresh sensor plus six invalid-input selections give **7 PASS / 69 NOT_RUN /
PARTIAL**, result SHA256
`8c186942ea61e54f6de682bf7e230d3bce8ed1e2a564b271676d439c994bb048`.
All six rejections have execution_started=false; these tests do not qualify
native tracking or recovery. The initial new nonfinite helper fixture attempted
to seal before the callback destructor; its two test failures were corrected by
ending callback scope before sealing, with no production change.


The exact shipped serializer now also passes an actual ARM64 affected-package
rebuild under 4 CPU / 4 GiB. Its 52 devel ELF paths and 238 loader dependencies
are AArch64; the header hash matches PC:
`25881307abfcd32f329d7b0e3a3c6550ae5c8cdb1b461eb631017c7d278fa406`.
Sixteen callback cases and 17 IEEE-bit cases execute successfully under QEMU
with effective final `-O3` and assertions enabled. This reuses frozen dependency
artifacts; it is not a full dependency rebuild or RK3576 BSP/ABI qualification.
Installation SHA256 `c4820541e2da0098d32d08c800dcd82b433496da91c0996e9758e563209176b3`;
independent audit SHA256
`f389c85f5f5ee83a91357810a6fe42c6e691fa17b909194ac0c6e7d8b9b24d62`.
Parents and all historical negative/helper receipts remain intact.

The exact new PC serializer full-sequence regressions have independent integrity
audit PASS, SHA256
`459816860a5421f1c2b084153e0489c8c02002478bfcc8d35e3ae8d9e5fb2e19`.
Stereo retains all 2,912 camera outputs, ATE 0.04929092237 m and one-second RPE
0.04303296645 m / 0.465562 degrees. Mono remains FAIL / TRACKING_LOST:
three poses / 0.1 s / 0.10302% source frames; no one-second RPE can be measured.
Both have exact 29,120 callback occurrences, four zero child exits and confirmed
container absence. Mono records 1,632 buffer acceptances and 27,488 paused
occurrences. These are callback/buffer facts, not proof of downstream integration
or successful continuous mono tracking. No cross-mode algorithm ranking is made.

A fresh selection of all eight evidence gates yields **4 PASS / 4 FAIL /
68 NOT_RUN / PARTIAL**, at external
`prehardware_current_evidence_release_bits_v1/result.json`. The four failures are
explicit native_events.py source-identity mismatches in older ARM64 replay and
lifecycle receipts. They remain valid historical records; they cannot qualify
current code merely by replacing a recorded hash. Current exact-installation
replay/lifecycle runs and a deployment package are being prepared before any
receipt migration. The six local RK3576 reference/SDK-text hashes are unchanged;
the actual SDK directory still contains only the 73-byte vendor-contact README.
A bounded official-site search/fetch did not establish a downloadable matching
CV00/MV12 BSP; this does not prove no public copy exists. No vendor message was
sent and no other-board SDK was substituted.


### Reliable OpenVINS V101 mono 1x cohort

The full smoke and five fixed repeats at 1x have completed. Each retains 2,800
poses / 2,780 truth associations, fixed-scale ATE 0.07011501076521641 m and
one-second RPE 0.04523709741003572 m / 0.4780522903551887 degrees. The ten
repeat pairs have zero ATE difference under the unchanged 1 cm criterion;
strict numerical trajectory hashes also match the separate 0.5x cohort.
All 29,120 IMU occurrences match original ZIP, SQLite CDR, observer and native
callback timestamps/order/double bits. Each member has five zero child exits
and confirmed owned-container absence; all 49 output hashes per member match.
Native update P95 ranges from 11.1405 to 11.9805 ms and RSS from 129,184 to
135,284 KiB. These are shared-PC offline measurements, not target timing.
Health, reset, downstream IMU integration and native image-pixel consumption
remain NOT_RUN. Independent audit:
`state-flight-mock/openvins-native-observation/offline-reliable-v101-mono-1x-cohort-audit.json`,
SHA256 `1fed47bf24c852f9d192e8326f9f526d68466a5b436cd3adbd9543b869c5f3a4`.
The selected snapshot is 1 PASS / 75 NOT_RUN / PARTIAL; selected snapshots
are not combined into a comprehensive acceptance result.


### Exact serializer ARM64 replay and cleanup qualification

The new 4 GiB full stereo replay has completed with 2,912 poses / 2,871 truth
associations, fixed-scale ATE 0.08679594219 m and one-second RPE
0.04315925872 m / 0.463282 degrees. All 29,120 native IMU callback occurrences,
source pixels at the independent observer, raw/adapter scores, source/devel
files, runtime, output hashes and four zero child exits reconcile. The
independent audit resolves inherited patches from the actual verified deployment
archive; it does not fabricate missing copies in incremental build directories.
Audit `state-flight-mock/schur-native-imu-observation/current-serializer-arm64-native-replay-audit.json`,
SHA256 `b53d140cc0bb052e684b2a7e8bab370d9aa0505f474aa220ec3bf2c266a20c23`.
Cgroup peak is 583,143,424 bytes under 4 CPU / 4 GiB, with no OOM. This is
**EMULATED / MOCK VERIFIED**, not board performance or a repeatability claim.
The candidate state interface records 2,909 corrections / 17,888 output rows
(17,370 usable), 29,102 independently propagated rows with zero rejected samples,
and 14,976 independent watchdog ticks. Propagation does not refresh visual time
and is not a new native visual observation or uninterrupted trusted tracking.

The timeout, SIGINT and exact owned-native-child SIGKILL probes all actually pass
expected-failure cleanup. Their negative estimator runs stay FAILED. Native
child exit137 is observed; unknown individual SIGINT child exits remain unknown.
All owned process/container absence checks pass. The five new ARM build/replay/
lifecycle receipts pass CURRENT audit and are explicitly migrated in the single
matrix, with only receipt paths and hashes changing; all 75 gates and 67 required
cohorts/protocols remain unchanged. Migration SHA256
`a5188e00d288296c9d772d6aeb319bffe2c22b19bb5d4399081699c4d187b7e4`;
matrix SHA256 `391c8045314b644cd467bfae968c50b7e5bfc82948a0778c3c699b1f11157a0c`.
The fresh eight-evidence selection is **8 PASS / 68 NOT_RUN / PARTIAL**, result
SHA256 `69c3507ecfbd9aa13c05cecd0140d143a427c575539a5444f35cd43dd9e38d1b`.
Its source identity precedes the pending OpenVINS camera-parser integration.

### Bounded late mono survival result

The diagnostic full sequence remains **FAIL / TRACKING_LOST**: 1,515 poses,
52.0261% source-frame coverage, ATE 0.95878350477 m and one-second RPE
0.23714245654 m / 0.734073 degrees. Complete input, source/output integrity,
same-run scoring and four zero child exits independently pass. Its 22 captured
frames contain 5,632 point rows but report 7,257 point-overflow calls and five
seed-summary overflows. The footer is capped; exhaustive observation FAIL stays
explicit. Mature current populations reconcile for these frames, and all 104
next-frame inlier-loss events are observed: 40 grid skips, 31 alignment failures,
10 boundary rejections, 21 four-pixel rejections and two invisible points.
These are events, not unique points or a proof of the first failure's cause.

The selected window's first quality decline is camera1763 / bundle1748, with
45 minus 11 losses plus four entrants equaling 38 inliers. The actual first
post-initialization quality failure is camera301 and first tracking exit is
camera1721, both before this fixed window. A bounded trigger/history observer
was subsequently built in an independent child, preserving those results. Its
full V101 replay also remains **FAIL / TRACKING_LOST**: 1,554 poses, 53.3654%
coverage over 77.650000128 s, fixed-scale ATE 0.99956903086 m, and one-second
RPE 0.23470163023 m / 0.602573205 degrees. All 2,912 camera inputs / 29,120
IMU inputs, 38 output hashes, 16 runtime sources, 953 native sources / 527 devel
files, same-run scores, four zero child exits and exact container absence pass.
Integrity receipt SHA256
`74206a48a662a7f69329397a7c02ad75dba91a162cda062027888104930ae25f`;
run SHA256 `6d260acedeed5f4536183dc49c43de75a312e5f711d8e139503f4e368d336b4a`.

Its observer qualification is separately **FAIL**. The first quality trigger
at bundle155 is pre-tracking warmup after metric commit154. The lifecycle hook
precedes the final tracking-exit/reset decisions; unready begin also misses
an epoch-truncation path. An exact-source fixture reproduces pending8 with
no recorded truncation or epoch change. The full run captures 23 frames /
7,873 point rows / 51 promotions without caps, but closes with four post-window
frames pending; absence of caps does not repair the lifecycle gap. Independent
audit SHA256 `8ed286afa8202b2adaaaeda88de6abc12144fc0fe1634d9c07737ed969ee6c7f`.
The sealed child and all failures remain unchanged. The subsequent common-reset
observer child passed fresh independent review, SHA256
`37887a159652d05d3a437b376c0999425198c0ab670a4cfa1b11f8ed89fd1ac0`.
Its complete V101 replay still **FAILS / TRACKING_LOST**: 1,619 poses / 55.5975%
of source frames, fixed-scale ATE 0.678386820853 m and one-second RPE
0.157585338588 m / 0.609272738322 degrees. All four child exits are zero;
38 outputs, 16 runtime identities, native source/devel/patches, complete inputs,
raw/adapter scoring and exact container absence independently pass, integrity
SHA256 `d5813d53a7de3e2ec648f8a5f8fc58df07c47a08b5ce9b1bc5ab34de055fc23d`.
The new observer records the first final-stage exit at bundle1772 with 38
inliers, retaining its 17-frame before/after window, 5,531 point rows and 45
promotions without caps or pending/incomplete frames. The independent dynamic
audit now passes: all 51 phase populations, 17 frames, 5,531 point rows and 45
promotions reconcile exactly, SHA256
`38e77897d5e1cc6527f9569a1dc3aa620bb9bf08608322bbf4f98cf008da97e3`.
Both the first genuine quality failure and final-stage exit are bundle1772,
38 below the original 40-inlier gate. Bundle1771 to1772 loses seven points and
gains two: four grid skips, two patch-margin failures and one current four-pixel
rejection. Nineteen of the 26 corners promoted at1774 mature within the observed
window and become inliers. The older never-matures observation is a local
bottleneck rather than a universal explanation. No mono candidate is promoted; gates, priors,
initialization, solver and random draws remain unchanged. Original fixed-window dynamic audit
SHA256 `e6ca20a0660577644af4512e18115a2fd6f980519741abef8ecea7b66a063a0b`.

An exact-production ownership fixture separately confirms missing retained
host/current observation registration for promoted mono corners. Its narrow
prototype reaches three observations before pruning and retains two afterward,
with MIN_OBS3 unchanged. The actual linked Schur Reset and mono frontend reset
also leave Frame/Point/LocalFeature strong-reference cycles, including the
ordinary registration control. Normal, Release and ASan+UBSan diagnostic
assertions pass, while the native reset lifetime gate explicitly FAILS; LSan
reports 505,034 bytes in 47 allocations. Receipt SHA256
`ddce7e74c937f72da83a99833500c19ac956f541e79f2abf4c33ca4165ef77d2`.
A new isolated child repairs this registration and responsible reset /
destruction ownership. All 25 packages build, and 39 actual-type tests pass
(13 each normal, Release, ASan+UBSan), including ordinary/promoted/pruned reset,
destruction, repeat reset, stereo multimap and metric bootstrap semantics.
Installation SHA256
`1dff9ea95ccf050272ae13719f20d49c011d61a864773494156d1acb57ce47b3`;
incremental patch SHA256
`49f63c68f99bbe679fda959048be83b02289ed4f4cfaa0d32868eb0a38da4c43`.
Fresh independent cumulative review passes, SHA256
`6238f8874c0316ee20bb8ca9d92ffc618c08fe086cbd8afed6e2a00b09178570`.
The exact three-file change leaves solver, initialization thresholds, pruning
and stereo registration unchanged. Parent/source/devel/patch/link/inode
identities and five full-sequence preflights pass. Sanitizers instrument the
exact new methods and fixture; other linked catkin libraries remain Release.
The original build and fixture compile failures remain intact. The new full
V101 mono smoke remains **FAIL / TRACKING_LOST**: 1,942 of 2,912 source frames
(66.6896%), scale1 ATE 0.440638097692 m and one-second RPE
0.109712205684 m / 0.595945216830 degrees. Complete input/source/output,
same-run score parity and four zero exits/absence independently pass, receipt
SHA256 `79d9b1fc6b8ebffe01a0347003bf25bef5eb5617f219d7523917e2640b227c2e`;
run SHA256 `11ffca61b72e4516a78c6c9bd2207c312880d88cce5d8a9b91ac52b3ebf7de96`.
No five-repeat success group follows this failed smoke. Its bounded dynamic
audit passes independently, SHA256
`fa7ca16e8260898ed27bcb0bfe5dc0fc9699820b23504c46b2557b483bd43086`:
17 frames, 6,526 point rows and 116 promotions reconcile without caps or
incomplete populations. Of 78 corner promotions, 16 reach the next frame with
two real retained observations and register a third; 35 mature and 33 become
inliers. This demonstrates the local observation-ownership repair. The first
tracking exit at bundle1784 has 39 inliers and recovers at1785. The terminal
loss begins at2096 and resets at2101, outside the captured point window; its
cause is not isolated by that first-loss window. A separate bounded observer child retains the last
16 frames at the common epoch boundary.
It changes no solver, initialization, threshold or registration behavior.
Fresh independent cumulative review passes, SHA256
`6cf42ee96884b08a2c61b0b27707ffdef9af6e79c24c324c2d8990c1ee773578`:
1,474 source/devel regular files and255 links,44 parent/child traces,3 actual
production fixtures,52 ELF dependency closures and a full-sequence preflight
were checked. Only the two observer helper files and libsvo_common change;
protected estimator code remains byte-identical. Source-window, point and wire
caps explicitly prevent an incomplete population from being labeled complete.

The qualified terminal-observer full V101 replay still **FAILS /TRACKING_LOST**:
2,546 of2,912 poses (87.4313%), scale1 ATE1.117137974043 m and one-second
RPE0.298467537753 m /1.596831065799 degrees. Four children exit0 with exact
owned absence. Complete input/source/output and same-run scoring integrity
passes, SHA256
`df5d217a1f3e15e4ff785bd1d098b430b17926c52b84c68d1fae8eaa1464f9da`;
run SHA256 `71e69c6f25a537eb28d7392d7f9ff7577250f287b916e990843e357c9106124f`.
The sealed observer contains33 frames /14,709 point rows /390 promotions,
including16 newly retained terminal frames. Its footer reports no cap,
incomplete population, omission or pending frame. Independent dynamic
reconciliation passes, SHA256
`c4add59eefbf13486f0a89e7d14d5ba6eaba88212113951da6b49ab60ee915a9`:
all 99 phase populations reconcile. The publisher has all 2,912 images, while
native Info has 2,906; six missing bootstrap inputs remain explicit. Native
bundle IDs are mapped to source timestamps, rather than treated as source IDs.
First loss is native2292/source2298 (41 minus11 plus9 gives39); persistent
terminal loss is native2712/source2718 (46 minus18 plus11 gives39). Terminal
losses include seven grid skips, four patch margins, four alignment failures,
two four-pixel rejections and one unresolved point with no reprojection encounter.
Of341 corner promotions,103 directly register a third observation,225 mature
and181 become inliers. Reset sensor time remains unknown where the native event
contains -1. Greater coverage with larger error is preserved
without calling it a causal improvement across different geometric bootstraps.

The same ownership candidate's full V202 mono smoke also **FAILS /
TRACKING_LOST**: 64 of 2,348 source frames (2.7257%), scale1 ATE
0.225185417940 m and one-second RPE0.277022314032 m /1.178942593914 degrees.
All four children exit0 and the owned container is absent. Full input,
source/output and raw/adapter scoring integrity passes independently, SHA256
`3f30bf0c13f1bbc571e85199ce7418d964414720b2c232c2f0c19e1f4adb8c8d`;
run SHA256 `3b218cb9a1afe53b9679ebb9f2ec3a3825765cf4f83947bd34c3af352853c107`.
The bounded V202 dynamic audit also passes, SHA256
`ea9a4640b48ec81080426a8d2c05a1de8247c98c70a524c9ef0a224d62ec6761`:
14 frames /42 phase populations /4,992 point rows /86 promotions reconcile.
Reset at bundle219 explicitly truncates three remaining post-window frames,
with zero pending frames. The first failure is bundle214, source camera250:
51 inliers lose20 and gain3, resulting in34. These losses comprise9 matcher
alignment failures,5 grid skips,4 four-pixel rejections and2 visibility losses.
Source IDs and native bundle IDs are separately mapped. Although all2,348
images are published and received by the observer,36 bootstrap images have no
native camera event;2,312 Info records alone cannot prove full ingestion.

The full V103 mono smoke also remains **FAIL /TRACKING_LOST**:18 of2,149
source images (0.8376%), scale1 ATE1.602542150217 m. Its output span is shorter
than one second, so one-second RPE remains unavailable. All four children
exit0 and exact owned absence is confirmed. Generic input/source/output and
same-run scoring integrity passes for this run and the V101 stereo smoke,
receipt SHA256
`5f83b8d306afbd73f64027e61bc4fedd5167e0ee6cc90efa9864e5aedec4604d`.
V103 run SHA256
`81cb6ebbc456be3f4b31c7f5a3a641324a224fda6c73522f0a0ed004c8a69555`.
Its bounded dynamic observer independently passes, SHA256
`fae3e0d56076177b0cbf0b3cd9c65103dc9a7f9d73bf96d6f7b180d4d97deaae`:
14 frames /42 phases /4,946 point rows /137 promotions. First loss at native
bundle425, source camera469, loses33 and gains4, taking65 inliers to36.
Reset430 explicitly truncates three post-window frames with zero pending.
Of44 source images without Info,6 have actual native startup-IMU-coverage
waiting events and38 have no native camera event. These categories are retained;
no claim that all44 were dropped by the subscription queue is made.

Five recorded bootstrap windows were reconstructed with exact IMU endpoints
and linked to the frozen Release solver. Accepted/rejected reasons and measured
scalars match within1.57e-12, receipt SHA256
`050a7a6019ecdd5c270fa9ddba8dea1b61a80bcd41b8f512a7a2b5b15036cd9b`.
An exact-header clock-only fixture finds dense JacobiSVD with U/V takes
98.96–99.38% of the190–308 ms whole solve, receipt
`caa2baa37c7caa4426d8ed73e943a5da7dc534246c013cd353150fd128e1d5d4`.
A diagnostic values-only SVD plus QR preserves the observed full-state and
gate results across five real and six control cases (maximum real-window
state difference4.49e-14), but takes111–190 ms, still exceeding the100 ms
0.5x publisher cadence. This supports a processing-pressure hypothesis without
proving callback drops or a unique tracking-failure cause. Receipt
`633eb7f26a6078ac7e97fda7fe2225895f9b3b9fd679c9e1662a441b39e5626d`
retains near-gate errors and limitations. No numerical implementation, queue or
threshold has changed; the candidate remains diagnostic-only.
Earlier diagnostic exit137 events came from an aliased pose-window snapshot
and malformed fixture input, not an established compiler or native-solver OOM.

Further pinned Eigen3.3.4 comparisons retain a concrete negative control:
BDCSVD under upstream fast-math is numerically wrong in24 case-repetitions.
With precise compilation, five real and six control cases across three repeats
preserve gate/reason/rank; the maximum real-window full-state difference is
6.30e-11 and full singular-spectrum absolute difference8.95e-14. Whole-solve
times are44.35–70.22 ms versus the paired original Release medians191.62–312.64 ms.
Receipt SHA256
`962a40884901fbf589df68fdc3189dc505ca1693eff59c1342f4ccff082c0d44`
explicitly limits these results to finite diagnostic fixtures. A separate
precise numeric kernel is being implemented in an isolated child, keeping
original matrix assembly, gates and other compilation flags. An actual object
audit found shared weak Eigen instantiations between the first precise object
and fast objects within libsvo. The kernel therefore gets a dedicated small
shared library, with hidden Eigen symbols and an explicit API; no unsafe
same-library precision claim is retained. Its actual ABI,
linked production behavior, independent cumulative review and fresh native
replays remain required; no solver change is promoted from these fixtures.
An additional prebuild check links the actual parent Release solver and fast
caller to the new production kernel DSO. Its five true windows and nine
controls across three repetitions pass42 finite comparisons, maximum full-state
difference4.152e-13 and spectrum difference8.016e-14. Receipt SHA256
`5f72bf3a4a7c590d8f1423cb449140d759438c66856ece0f1486e2fa2f1c8840`
retains all measurements: candidate45.736–104.937 ms versus paired
parent196.305–513.560 ms. The104.937 ms sample exceeds the100 ms playback
cadence, so these results do not establish bounded callback latency or absence
of backlog. The final full child has its own link and floating-point-environment
qualification; this prebuild receipt does not substitute for it.

The isolated25-package child has installation SHA256
`a7abe00bf279a4248026cdc4aa7e32a4d36e5e06d580590c9bb0f3127ec0bc5d`.
Current-child ownership/reset39 cases, observer and terminal lifecycle tests,
five preflights and four ELF closures pass. A fresh binding audit found29
native-to-fixture helper bindings in the earlier comparator, SHA256
`58d99f42403e06a6ce68271857e0f8018d9ff419bb1dacf6ce7f41309e7ee454`;
that negative is retained. A replacement declaration-only Release probe
exports no Eigen/rm27 arithmetic helpers and dynamically verifies no such
interposition. Its33 actual parent/child comparisons preserve acceptance and
rejection reasons, with maximum16-state difference2.414e-13, receipt SHA256
`3e90823880e25e8625f39f1bb650ee684d33a9414e301a026a02e9c89f4d4303`.
Five true windows each repeated three times take40.959–61.533 ms for the child
and172.311–314.979 ms for the paired parent. These finite measurements do not
guarantee replay cadence or RK3576 performance. Inherited FTZ/DAZ and untested
rank-zero subnormal adjacent thresholds remain explicit; the original1e-6
rejection gate is tested. Independent cumulative qualification now passes,
SHA256 `a4688be523cc416048618e1f6553356897086dd895ccd758761e068530478bd1`.
Fresh complete V101, V202 and V103 mono smokes still fail TRACKING_LOST;
all twelve children exit zero with exact owned absence. Their retained pose
counts are27/2,912,7/2,348 and11/2,149. Fixed-scale ATE is0.100612961329,
0.0521731338256 and0.0443145622596 m respectively; the latter two lack a
one-second RPE interval. Neither short low ATE nor successful initialization
closes continuous mono tracking. The two affected stereo smokes numerically
pass: V1012,912 poses / ATE0.0423660008273 m / one-second RPE0.0432958904817 m;
V1032,149 /0.0659233798231 m /0.0348815066739 m. All eight stereo children exit
zero with owned absence. Independent input/source/raw-adapter/exit integrity
audits now pass for all five full smokes, receipt SHA256
`9b82fc1e4f832dab9f85663a57cdcbab07eb326a62873215ed03f31810180926`.
V101 and V202 bounded dynamic observations pass. V103 retains an explicit
observer FAIL: epoch4 resets before tracking, and epoch5 first loss is outside
the saved observer scope. This missing window is not estimator success. Five
fixed repeats are still required after successful smoke. This child remains
NOT_PROMOTED.

The V101 run has27 poses from2,912 source images. Same-run input/output/score
integrity passes, SHA256
`a434f83ce62efa93a1c1d341a5aa79e67e410db64dcfffaf7e93e6288182f607`.
Its bounded dynamic audit passes48 exact phase populations,5,845 point rows and
102 promotions, SHA256
`5adfeb461f86c38017c6a0dbc0f3f915ccf6c7949578373aa6cac31b8edff01f`.
All2,912 native Info callbacks are present;33 native solve envelopes take
P50 36.133 / P95 65.031 / maximum65.781 ms on this shared PC. Different
bootstraps prevent a controlled accuracy claim. At first loss200, inliers fall
50 to36 (17 lost,3 gained). Twelve lost points exceed4px before correction;
camera and point corrections reduce each residual without bringing it below
the unchanged threshold. The same-bootstrap69-point /257-factor /5-clone
finite objective and68 recorded posterior updates pass, SHA256
`db766b95ebb6811be4a7b6512124b48a185665c929a46641a723fd210007fe38`.
Point745 lacks a posterior observation (NOT_RUN); the older full-map observer
truncates283 points to256 and remains FAIL_TRUNCATED. These checks do not
qualify the statistical bootstrap prior. Pre-update prediction and actual
feature/model consistency remain unresolved software investigations.

The affected V101 stereo smoke **PASSES** numerical replay:2,912 poses,
scale1 ATE0.0581943557923 m and one-second RPE0.0435558539610 m /
0.462469245165 degrees. It has four zero exits/absence and zero state-output
evidence errors, while three native health rejections remain visible; this is
not continuous trusted availability. The V103 stereo smoke also passes numerical replay with2,149 poses,
scale1 ATE0.0565655868909 m and one-second RPE0.0365128091045 m /
0.385023479006 degrees. Four children exit0 with owned absence; state-output
evidence errors are zero, but65 health rejections and71 transitions remain.
Run SHA256
`ed8207167192cdfc49c66b67071ce69d438b5a96f7ebd494e239275353744133`.
The V101 five fixed repeats pass independent integrity and numerical replay,
receipt SHA256
`c1ff591e7b7eb4b80d0ec739298ef280f55be84bd2088d4236c35e0328331b35`.
Each retains 2,912 poses and exact camera/IMU/calibration identities; all20
children exit0 with owned absence. Repeat ATE is0.0483322–0.0788977 m and
one-second translational RPE0.0430282–0.0435507 m. The unchanged1 cm
repeatability diagnostic **FAILS**:5 of10 pairs exceed it, with maximum ATE
difference0.0305656 m. This is within-cohort repeatability, not a cross-algorithm
ranking. All numerical adapter states remain UNKNOWN/invalid. Per-run invalid
health transitions are4/3/4/4/5; zero propagation rejects does not establish
continuous visual validity. V103's original five fixed ownership repeats are
now complete and independently audited, receipt SHA256
`dc10bdb92bca182808a9c6cfd16f599320fcdc2e7cb5527d7393142898f1d7d6`.
All five retain2,149 poses and2,094 GT associations, with20 zero child exits
and owned absence. ATE is0.0502139224–0.1188875017 m and one-second RPE
0.0350699174–0.0379517252 m. The unchanged1cm diagnostic **FAILS** for7 of10
pairs, maximum ATE difference0.0686735793724 m. Their287 health transitions,
262 rejections and3,296 unusable outputs remain visible; all10,745 numerical
adapter rows remain UNKNOWN/invalid. No continuous validity is claimed.
The complete sequences remain the acceptance inputs. No candidate is promoted. Numerical changes between independent
bootstraps do not isolate the repair's causal effect. Neither fixture nor
observer PASS proves metric consistency, full-sequence tracking or recovery.

### OpenVINS camera-consumption integration

The existing comparison runner now enables camera provenance only for the
explicit, validated native capability. It reads the actual hash-locked OpenCV
YAML downsample setting and compares every selected source ZIP image, SQLite
CDR payload and observer record with the subscription, decoded queue, feed and
frontend boundaries. Observation integrity, complete input coverage, frontend
consumption and numerical scoring are separate results. These observations do
not qualify accepted EKF constraints, tracking health or reset behavior; the
adapter retains UNKNOWN / invalid. Installations without the capability retain
their previous path. The immutable candidate metadata remains build-only even
after a replay; fabricated PASS claims are rejected before output creation.

The isolated native candidate also fixes a demonstrated callback lifetime
hazard: the IMU worker captures its callback-local message by value. An ASan
fixture actually reproduced the parent's stack-use-after-return. Camera-worker
join does not establish the separate initializer worker's complete shutdown.
The source installation is SHA256
`fa36fc0935dd093e23d1da57b4b004e7fb39f314645f34bed282efb137a267f3`;
earlier baseline builds and runs retain their exact identities.

At that source checkpoint, full software regression was **923 passed / one skip**, exit0, 18.36 s;
the affected four-file suite is **289 passed**. All 33 recorded code/fixture
hashes are stable across execution and the owned process group is absent.
Contract receipt SHA256
`4839a0e6d5be248f02e88a7c0363738573be49e932a732407ec696e0a00db23d`.
Only sensor-contracts was migrated in the sole matrix; 75 gates / 67 required
cohorts and their protocols are unchanged. That matrix SHA256 was
`51c4917aeafd5ba6eddd553879beaf425d1b51b1a0153904995dc8d4b898aa2e`.
A fresh evidence plus invalid-input selection is **14 PASS / 62 NOT_RUN /
PARTIAL**, result SHA256
`a18ad8df77a99a12e88c5d55823934104b10524f320b45689233f44468d01ca8`.
The selection alone is software/mock and existing ARM reference evidence.
The separate fresh full V101 mono replay has now completed and independently
passes all 16 source/pixel/score/exit checks. All 2,912 images traverse the
subscription, decoded queue, feed, manager and frontend enter/return boundaries;
29,120 native IMU callbacks match original integer times, order and double bits.
It produces 2,800 poses, ATE 0.07011501076521641 m and one-second RPE
0.04523709741003572 m / 0.4780522903551887 degrees, exactly matching the same-run
raw/adapter scores. Numerical trajectory matches the prior reliable baseline.
There are 11,031 accepted visual events and 2,800 distinct visual-time advances;
these do not establish that every image contributed an accepted EKF constraint.
All five child exits are zero, all 50 output hashes pass, and the exact owned
container is absent. The complete ledger has 161,521 events / 81,053,822 bytes,
below the unchanged 200k / 100 MiB limits. This is **PC OFFLINE VERIFIED** for
the stated boundaries; health/reset and complete initializer shutdown remain
unqualified. Independent confirmation SHA256
`bb8edaf9860465c35cabd06fdac952b64610fcbd1f233246555150968f5f8c60`.
An initial audit compared in-memory integer camera keys directly with JSON
string keys and produced a false-negative receipt, retained unchanged. The
same auditor was corrected and all checks repeated without changing run data.

### Full UZH release content lineage

Both pinned official v3 bags have now been downloaded outside Git and fully
compared with the released ZIPs and existing cam0/IMU recordings: 3,061 chunks,
9,140 index entries, 9,044 camera images, 80,868 IMU records, and 39,850 records
in each of the PoseStamped and Odometry truth topics. All decoded camera
pixels, published truth double bits, official 12-decimal IMU projection and
calibration/configuration bindings agree. Fifteen acquisition/parser guard
tests pass; complete artifact/source hashes were checked again after the audit.
The complete content gap is closed for these exact artifacts. Review receipt
SHA256 `e1f4c5e28d1682f736803876f9ffe35785dbe19b547b08bb1d0236f4fa807bd0`;
full-content receipt SHA256
`5c09814debde4ce09192dcdf70020936f454bee361a7d6460a3ebc9929bed88d`.

This is equality through the declared export transforms. Seq6 sensor text
mapped to nearest integer nanoseconds differs from original ROS stamps by
-119 to +119 ns. Every IMU record has at least one original/export double-bit
difference; the largest 12-decimal rounding error is 5.009326287108706e-13.
These differences must not be labeled bit-exact sensor preservation. The
original bag hashes are locally measured, not published official checksums.
Pre-bag timestamp processing, exposure start/midpoint, physical synchronization,
exact truth-generator invocation and physical body registration remain
NOT_ESTABLISHED. Existing UZH estimator FAIL and diagnostic-only scores remain
unchanged; this audit establishes neither VIO recovery nor hardware timing.

### Owned OpenVINS workers and current integration regression

Exact production initializer fixtures reproduce the parent detached-worker
heap-use-after-free on both successful and failed initialization; a separate
publisher fixture reproduces the same lifetime fault. Three ASan failures remain
unaltered. A new isolated candidate owns its publisher, camera feed and
initializer workers and joins them in dependency order before final visualization.
Seventy-two normal / actual Release / ASan+UBSan helper executions pass, along
with the three-package build and four ELF/dependency closures. Parent source,
workspace, raw symlinks, inodes and manifests remain unchanged. The candidate
is terminal shutdown only: solver preemption, reset and health are unqualified.
Installation SHA256 `760ecf0278c36a36e25348065c5957e45f687b8c9afd1fd5147ab86aad09e5cd`;
build/helper review SHA256 `630cae6dfeec2a184e0b2ad665a12fc0e1973245d0a375ba208af2047873b9eb`.

Its first launch was refused before output creation because the camera
capability validator recognized only the earlier shutdown description. The
existing comparison runner now explicitly accepts the new recorded description
as well; unknown capabilities and fabricated PASS claims are still rejected.
The regression reproduces rejection before this change, then passes afterward.
Current full suite is **924 passed / one skip / 16.67 s**, with **334 passed**
in the affected four-file suite. Thirty-three code/fixture identities are stable
and the owned test process group is absent. Receipt SHA256
`cb2b4c2af09a6c8206585a56b4a840c0643c36846c8a182936f3e2c1a0902216`.
Only sensor-contracts receipt/path hashes changed in the sole 75-gate /
67-cohort matrix, now SHA256
`8b27ef2195e44c4ca746767db432682abb841708f7cd7dc7a921b2864b524048`.
A fresh evidence plus invalid-input selection on this matrix completed with
**14 PASS / 0 FAIL / 62 NOT_RUN / PARTIAL**, result SHA256
`861339e7b3cb0d36e2f7c73663846e437e05fa7962b7fb20fe5bd505fe923fd3`.
The new full V101 mono replay completed with
`EXECUTION_COMPLETED / COMPARISON_RUNTIME_AND_METRIC_PASS`, run SHA256
`69b0d21fb3bb3a8d5462a778d48c3c506cb16451e7cfe9380fce29ea415e18f6`.
Independent input/pixel/worker/score/provenance review now passes all 18 checks,
SHA256 `bdda0a7ab8a1f5111635e958a3da0215aa6cd7a8b041ee042e39aa715822f4cf`.
All 2,912 camera frames and 29,120 IMU samples match source/CDR/observer/native
integer times, ordering and pixels/double bits. The 161,652-event ledger closes
with zero dropped/pending events: initializer joined, receipts closed, workers
joined, then footer. The 2,800-pose trajectory is byte-identical to the preceding
camera-worker baseline, SHA256
`564d71e12912aab55c2d4a045394347c157bcf2f14c973fcbd38b79349de541e`;
ATE remains 0.0701150107652 m, one-second RPE 0.0452370974100 m / 0.478052290355
degrees. All 50 output identities, 17 runtime identities and five zero child
exits/absence pass. Fixed repeats and stereo under this installation follow
explicit configuration migration. Existing parent runs retain their own source
identities and qualification limits; UNKNOWN/invalid is unchanged.

The same installation also passes ten actual no-data SIGINT shutdown cases
using the existing `tools/openvins/lifecycle_smoke.py`: five with the publisher
thread enabled and five disabled. Each reaches subscription readiness, exits
zero without timeout or teardown error, and leaves no owned container. Locked
source/build files remain unchanged; receipt SHA256
`8aa836a398fb793a904ffec55bba81913e33d1eccd8a41e061f81965359b6328`.
Zero sensor samples are sent. This checks terminal shutdown only, independently
of successful VIO initialization, estimator reset or continuous state health.

The sole matrix now explicitly migrates nine OpenVINS configurations to this
installation. Both the native audit above and the separate migration review
pass; the latter verifies 18 old/new config locks and 955 artifact hashes,
SHA256 `589e0ea0d6a31cafe96d0294d9b114811e86c118b047707c7a491212d8fcfcec`.
Only installation/run ID/output and corresponding identities/locks change;
all 75 gates, 67 required cohorts, input/truth/calibration, rates, eight
smoke-plus-five-repeat protocols and the single artificial-fault protocol
remain fixed. Matrix SHA256
`464aaa62765c5e37c95bb79987ce40acfce93c80474db7410c96e8e6a7773dec`;
activation receipt SHA256
`cf8fd9f54c9cfca6c7ea200ced7a8ca3a232b0635cd16c17fb0bbf1473967192`.
Activation occurs after the preceding native exits and cleanup. Old matrix,
configurations and results retain their original identities. The first
migrated V101 stereo1x cohort has now completed; the other eight remain
NOT_RUN.

The first new1x cohort preserves all six numerical completions and30 zero
child exits/owned absence. Repeat2 has2,793 poses versus2,800 in each other
member: cam0 source ordinal640 is absent from native subscription and six final
pairs remain queued without feed. Complete ZIP/CDR/observer input therefore
does not prove complete estimator consumption. The original1cm within-cohort
repeatability criterion also fails4/10 pairs, maximum ATE difference
0.0103577427761 m. Independent integrity audit passes while acceptance fails,
SHA256 `d291e3e37937346d156c51e084b22a729d768528430beb8b51811363272e8af0`.
The original selected result is retained unchanged, including its incorrect
strong PASS. That orchestration defect is being corrected: require native
camera consumption, independently reproduce its complete summary with the
existing source/CDR/pixel/native decoder, and apply the original1cm cohort gate.
Two configuration/nested-coverage contradictions were first reproduced as
false PASS and now fail the affected tests; independent cumulative review and
fresh full-suite execution now gives940 passed / one skipped /18.89 s,
33 stable code/fixture identities and owned process-group absence, receipt
SHA256 `93242b24bd3ed58928673b842d20fd6f32e8f8eb5fdfaafe59df678560b7f865`.
The readback boundary explicitly normalizes integer camera object keys to the
strict JSON data model; the initially rejected valid smoke and a cross-file
orchestration fixture failure are retained. Independent cumulative review now
passes350 affected tests and all33 full-suite code hashes, SHA256
`63eb91e0056066cee6a8158efafadf7e1ff805a38296910d62299a298f6a4fa2`.
Actual production read-only audits accept smoke/repeats1/3/4/5 and reject
repeat2; source/CDR/native reconstruction matches its retained FAIL exactly.
The sole matrix explicitly migrates only sensor-contracts to the new940-test
receipt, SHA256
`3e2dd480bc0955b6d3c878a3d4aaec0ab0eaa09923620b4764d08be6ce2e4a37`.
All75 gates/67 required cohorts and the other74 gates/protocols remain equal;
migration SHA256
`b770ef60632e48f8d97a7e01db36cc97e9239e60ebf0d3328084ab52654a459a`.
Fresh contracts plus six explicit malformed-IMU rejections on the existing
runner complete7 PASS /0 FAIL /69 NOT_RUN /PARTIAL. This is a selected
snapshot, not complete software qualification. This change does not rewrite
original numerical runtime outcomes or upgrade UNKNOWN/invalid health.

A separate exact-production helper reproduces the terminal ready-queue defect:
newer IMU data has already been fed while an active worker keeps an older
captured endpoint and discards subsequent wakeups.168 normal/actual Release/
ASan+UBSan fixture cases pass the diagnostic assertions, SHA256
`34b79cee1d414a21fbf491e4407d134749aee99ef6f2315b37cffd94bec564a7`.
A fixture-only terminal drain consumes the ready tail using an already-fed,
strictly ordered watermark without adding IMU samples or changing thresholds.
This does not fix the separate subscription loss or qualify live scheduling,
health, post-shutdown pose export or a permanent runtime derivative.

### Declared stereo dropout

The existing matrix and runner executed the full 0.5x V101 artificial dropout
profile with seed270031, beginning at source elapsed20 s. Left/right retain
2,660 images each from the original 2,912-pair denominator; 252 pairs are
removed and all 29,120 IMU samples are retained. It outputs 2,660 poses /
2,620 GT-overlap poses, fixed-scale ATE 0.04740063113 m and one-second RPE
0.04367864807 m / 0.461955403 degrees. All four child exits are zero.
Thirty-nine output hashes, 16 runtime sources, complete source/devel/patch
identities, parent/prepared inputs, publisher/observer reconciliation, same-run
raw/adapter score parity and exact owned-container absence independently pass.
Integrity receipt SHA256
`211beeed557c60b1f4caf24f1766043eb4fa924b6d105ea4f1c5b18e3387ae64`.
The first audit omitted the runtime-added NOT_RUN callback-ledger field and
produced a false negative; its receipt remains unchanged. The same auditor
was corrected and rechecked the identical sealed run.

The numerical result is **PC OFFLINE VERIFIED**. Strong recovery remains
**NOT_RUN**: accepted post-fault continuous recovery was not established.
The single selected matrix snapshot remains PARTIAL with every stronger gate
open; a trajectory spanning a disturbance does not prove recovery or fault
causation. Native per-sample IMU ingestion remains unobserved for this build.

The separately declared 30 percent profile also completed at both rates. Its
fixed seed270031 and elapsed20 s start retain 2,167 of the original 2,912 pairs
(745 removed); all 29,120 IMU samples remain. Each run produces 2,167 poses.

| Rate | Fixed-scale ATE RMSE (m) | One-second RPE translation (m) / rotation (degrees) | Numerical integrity | Strong recovery |
| --- | --- | --- | --- | --- |
| 0.5x | 0.0554678396866 | 0.0424829602259 / 0.449810220046 | PASS | NOT_RUN |
| 1x | 0.0610526209835 | 0.0425779824392 / 0.462833112858 | PASS | NOT_RUN |

For each, all 39 outputs / 16 runtime identities, native source/devel/patch
closure, original and derived inputs, publisher/observer reconciliation,
raw/adapter score parity, four zero child exits and owned-container absence
were independently checked. Integrity receipts are SHA256
`34e7aa1dbb712699d71f093e53893d7f33c3647e39385f09464cd96e3d943960`
and `99d2972c9954115d417e10f89229360fdb87566680a856908a88ac1d8e2330a7`,
respectively. Both are **PC OFFLINE VERIFIED** for numerical replay; their
separate selected acceptance snapshots remain PARTIAL. These two runs are
distinct rate groups and do not establish recovery or an algorithm ranking.

The remaining declared 10-percent dropout at1x and blur-kernel9 at0.5x/1x
also completed and independently passed the same numerical integrity checks:

| Profile / rate | Poses | Fixed-scale ATE RMSE (m) | One-second RPE (m / degrees) | Integrity receipt SHA256 |
| --- | ---: | ---: | --- | --- |
| Drop10 / 1x | 2,660 | 0.0515403455356 | 0.0429425786734 / 0.466976506239 | `47a0c0e9b2fc7b66bda4f226bc4246875954a206e71dc805733e21237218e41e` |
| Blur9 / 0.5x | 2,912 | 0.0657431545653 | 0.0433136675360 / 0.456401538224 | `d8efffccc5813f8f1201930753e11687068423ab4847350350316d4a4003a0a6` |
| Blur9 / 1x | 2,912 | 0.0548833699807 | 0.0431551818957 / 0.456312102574 | `6402c183facb77567ed769af0a94ceecdbaf2247bf41214e038e465e8f7ed759` |

Each has four zero child exits, exact owned-container absence and all 39
outputs / 16 runtime identities, source/devel/links/patches and same-run scores
verified. Strong recovery remains **NOT_RUN** for all three. The two1x selected
gates finish with PARTIAL / 0 PASS / 0 FAIL / 76 NOT_RUN, result SHA256
`c52b88d0f3f947277b82a1d20f2e439a4e39ac562ebd02c9209f7b623525ba91`.
That command exits1 for incomplete acceptance; all native children exit0.

The declared gain0.25 brightness reduction at1x passes both numerical integrity
and the existing recovery-observation gate. The complete sequence has 2,912
poses, scale1 ATE 0.0575558306803 m and one-second RPE
0.0431142415457 m / 0.457061511375 degrees. Immediately before source elapsed20s,
the projected state is usable and Info quality is good. At the exact fault
onset, Info remains tracking but quality becomes insufficient and the projected
state becomes unusable. After the half-open20–30s disturbance, accepted visual
corrections at30.10s and30.15s have positive constraint counts, good Info and
usable states in the same native epoch1. Thus measured recovery is0.10s after
the fault end, with no claim of a reset or reinitialization. The independent
watchdog remains in its receipt clock; no sensor-time mapping is inferred.

All 39 outputs / 16 runtime identities, full inputs/source/devel/patches,
same-run raw/adapter scores, four zero exits and owned absence pass; receipt
SHA256 `428262fac254ce67f0a4215cbc74a82a785a39fde12c21eea933d4667b8d5d2c`.
The selected result is PARTIAL / 1 PASS / 75 NOT_RUN, SHA256
`29bb3a363273533d965851376a10303ec3398e3c1fc4c0dff5f4413ff5a374dc`.
This is **PC OFFLINE VERIFIED** for the stated native visual recovery
observation. It does not establish continuous valid availability, onboard
latency, flight safety or native high-rate IMU integration.

The separate75-percent texture-mask1x experiment also passes the declared
recovery-observation rule. It retains all2,912 poses with scale1 ATE
0.126904107543 m and one-second RPE0.0477951169527 m /0.481769696056 degrees.
Projected health becomes unusable at the20s onset; accepted visual corrections
at30s and the following frame satisfy continuity in native epoch1 after the
mask ends. This is recovery of visual health without a claimed reset. All39
outputs /16 runtime identities, original/derived inputs, source/devel/patches,
same-run scores and four zero exits/absence pass; integrity receipt SHA256
`92d9bb7712b88c000c2c88687c2459ef4929b66eb1a3105321053e7f03798deb`.
Selected result SHA256
`6d2434a7c0af2e5e25dc4c80cacdc7f1d81424b1edb6dd52986a6d3c90696046`
is PARTIAL /1 PASS /75 NOT_RUN. The larger ATE remains visible; neither an
observed recovery nor a single trajectory establishes accuracy superiority.

The separate100ms camera interruption at1x completes with2,910 poses,
scale1 ATE0.0440133417605 m and one-second RPE0.0431950498212 m /
0.461348931304 degrees. All39 outputs /16 runtime identities, source/devel,
original and derived inputs, same-run scores, four zero exits and exact owned
absence pass, integrity SHA256
`653f88ea979c068013982333551c70208a59bf5d0fed2e37fd1b24f6d6cb23a2`.
Its reported later recovery is96.6s after the disturbance and outside the
association window. The declared recovery gate therefore remains **NOT_RUN**;
a late event does not establish recovery from this injected fault.

### Declared stereo occlusion at1x

The existing matrix/runner completed both100ms and500ms artificial black-image
occlusion gates, seed270031, beginning at source elapsed20s. All2,912 pairs
and29,120 IMU samples are retained; respectively4 and20 camera source events
are altered. Each full run produces2,912 numerical poses in native epoch1.

| Occlusion | Scale1 ATE RMSE (m) | One-second RPE (m / degrees) | Observed recovery after fault end | Integrity receipt SHA256 |
| --- | ---: | --- | --- | --- |
|100ms|0.0568377301302358|0.04413062535998974 /0.45522377036394374|0.0s|`469c0580440cc6bc52205c65870fd01262b71487c850662f12bfe6f006bc58b5`|
|500ms|0.04979899871102561|0.043761747765995014 /0.464571179412383|0.1s|`c02bb9937f582fdffe4fc036b71e063cd84a3f3aae64e9494720492726c1c1f4`|

Both independent audits verify39 outputs /16 runtime identities, complete
source/devel/patch closure, original/derived input and pixel reconciliation,
same-run raw/adapter scores, four zero child exits and owned-container absence.
The existing recovery rule observes failure at the20s onset and two accepted
visual corrections with good Info and usable states in the same epoch after
the fault. Recovery from detection is0.1s /0.6s; this is not a reset or
reinitialization. The independent watchdog remains in its own receipt clock
without a sensor-time mapping. These are **PC OFFLINE VERIFIED** recovery
observations, not continuous availability, onboard performance or flight safety.
The selected snapshot is PARTIAL /2 PASS /0 FAIL /74 NOT_RUN. Its CLI exit1
expresses incomplete acceptance, not a native process crash.
Independent combined audit SHA256
`b7ce3b69a28f505bb8dc97516d3c3527c1353dc805003c0bb494a3a1744bd978`
rechecks39 declared outputs plus run.json,16 runtime identities, original
pixels and all29,108 propagation rows against their correction snapshots and
raw IMU identities. Propagation does not refresh the visual timestamp. Recovery
corrections are camera402/403 with456/376 constraints and412/413 with533/816
constraints, respectively; truth never drives projected health.

### Ownership ARM64 lifecycle closure

The exact ownership ARM64 derivative completed timeout, SIGINT and individual
native-child failure through the existing lifecycle entry points. Independent
receipt SHA256
`adc3f633bca94070601baaf3b399fb943ece7fe05e10f9ef11409e0e7ce34c6d`
verifies115 outputs,161 artifacts,34 code identities,126 recorded PID/starttime
pairs and three absent owned containers, plus source/devel/rootfs,52 AArch64
ELFs,238 loader dependencies and13 deployment patches. All three intentional
negative estimator runs remain FAILED; the killed native child exits137, and
SIGINT individual exits without an execution record remain UNKNOWN. This is
**EMULATED / MOCK VERIFIED** process cleanup, not RK3576 ABI or performance.

### Pre-update geometry observer qualification

The first complete observer child builds25 packages but fails independent
production-boundary qualification. The observer reads seed_mu_range_ without
ownerseed initialization and CHECK-aborts on an unknown native camera index.
Normal/Release versus sanitizer allocator values demonstrate the former; the
actual reprojection fixture demonstrates the latter. Independent early receipt
SHA256 `2fae014203f3bbc95b03e976877334f6e331699cc195ed64a92300b780d4c825`
is FAIL. No dataset replay uses that candidate. A fresh isolated observer-only
child preserves unknown fields as null with provenance and separates lifecycle
completion from truncated population completion. Its36 normal/Release/sanitizer cap/unknown cases,25-package build and cumulative
independent qualification now pass, receipt SHA256
`702f7d5c923a3fc4459cce85445c6ff5c1c5ac41c447678ea4206935a7185908`.
All53 ELF targets,180 inherited flags, full source/devel/link/inode closure,
production transform/matcher, ownership, lifecycle and pristine zero-difference
comparisons were checked. The exact installation is
`a1743b7fc6970a53a1e873aadd623aa860f085001df0ccf037e8ba6134411c9d`;
its new full V101 mono replay completes **FAIL /TRACKING_LOST**, with22 of
2,912 poses (0.7555 percent),1.050000128s coverage, scale1 ATE
0.0450021981126m and just two one-second RPE pairs
(0.145344947053m /0.768219761051 degrees). All four child exits are zero;
run SHA256 `76b70e6a5011b97dc708bb3e24bf3d85f2978ab0f3b19ce75b3b4380873f7d05`.
Full-run numerical/input/process integrity passes, receipt SHA256
`a783ee18d72e40c4b04eea028f2d885c06876a793eea07e058b4783714b9a5d0`.
Independent dynamic audit SHA256
`6e4bc4198636554e62ca5bb346f5b8a02e673796a424a5d5be0821bdcbb50a0f`
is **FAIL_TRUNCATED** for geometry and PASS for the survival populations:
13 of16 retained frames omit the entire over-cap wire payload, including first
loss186. The first loss is49 to39 inliers,15 lost /five gained; twelve losses
are current4px rejections, one patch-alignment failure and two grid skips.
The first-loss geometry cannot be reconstructed from the omitted rows.
The complete metric162 atomic witness retains8 frames /295 points /639 seed
distributions, with no implementation transform/rescaling invariant failure.
That does not establish physical scale, bias or statistical initialization
correctness. Geometry actually retained for189–191 independently reconstructs
483 residuals to1.53e-13 and161 matched observations exactly.

A narrow post-loss addendum, SHA256
`b684429c560114f136fcb32e2866bcb79811b289c595857633e4df353b77ddb2`,
explains135 local-frame/clone pose differences among1,677 observations. All
occur in phase0 for the immediately preceding failed frame: the existing
relocalization fallback restores its visual pose to reference184, before the
next Solve3 resynchronizes it. This is not evidence for the earlier loss186.
Estimator math, calibration, initialization thresholds, RNG and gates are unchanged.

The measured observer defect has a bounded lossless repair path. Read-only
receipt SHA256
`580ae5f94a6dc7749f36ef24d3e91412711708664cc698cf8645ae6d5c26e5ea`
shows that content-exact interning of repeated frame/clone value objects reduces
the three retained frame payloads by70.92–71.68 percent, with byte-exact and
JSON-exact expansion. ID-only interning is invalid because an ID can have two
different pose versions. Omitted first-loss byte lengths remain NOT_RUN.
A new isolated observer-only child retains the same1.5MiB per-frame,64MiB
file,35-frame and metric-witness limits. Its25-package build and45 paired
production-serialization checks /87 executions across normal, Release and
ASan/UBSan pass, receipt SHA256
`95144380729e3faaec0117ed24e07a3db8812bfd4ca7ac05bb784faf92225eed`.
Installation SHA256 is
`5ccf0bc9ce34c7eb40df682cf446277096f4eb44d6e9bc75628270bf42f29d87`.
Only the observer header/implementation change; installed libsvo.so and the
precise bootstrap kernel remain byte-identical to the bounded parent.
Independent cumulative review now passes, receipt SHA256
`38d54c4ab990dfe1377367415e3f7ee657775e00660f9c6d729035518221c3be`.
Fresh retained production regressions cover39 ownership cases,18 terminal
executions and33 pristine numerical comparisons with zero differences. The
strict schema2 reader accepts39 actual streams /87 byte-exact round trips and
rejects29 malformed or ambiguous inputs. All53 ELF,180 build flags and the
source/devel/patch closure are verified. Earlier harness failures remain in
their original receipts; an earlier ASan terminal failure has UNKNOWN cause,
and these checks do not establish production-wide sanitizer coverage.
The frozen child is ready for root's serial native replay using config SHA256
`b73d4b34eb3f4d9e9083c943266e202822bf99bbe6cf9ca359a3b7afc2ecc4d4`.
This enables diagnosis; it is not a tracking repair or promoted candidate.

The fresh full V101 mono replay has now completed with **FAIL / TRACKING_LOST**,
run SHA256
`da2e3b3839bd233065c5962e4a008eba51ac3e2d9dfbe31d71a245d535752e1b`.
It publishes13/2,912 poses over0.6s, with fixed-scale ATE0.0303459710m;
one-second RPE is NOT_RUN because there are no eligible pairs. All four child
processes exit0, exact owned-container absence is verified, and the12GiB cap
records472,354,816 bytes peak /zero OOM. A short score does not qualify the
sequence; the failed smoke does not trigger five repeats.
Independent dynamic observer audit passes, SHA256
`0b1e3ba591f19be1bdff9f7302d9efd6d03bea92485fa1bfaf0770779b01675c`.
All16 geometry frames are complete, including first loss182, with no omitted
wire/rows or cap overflows. It reconstructs4,218 residuals within2.53e-13 and
verifies1,406 matcher bearing/pixel/slot associations. The metric167 witness
covers eight frames,296 points and537 seed distributions; its transformation
invariants pass, without establishing physical scale or bootstrap covariance.
The first-loss transition is71→37 inliers:37 lost and three gained, including
31 actual matcher alignment failures, four grid skips and two current4px
rejections. Different bootstrap histories prevent causal comparison with the
older bounded run. Detailed matcher/initialization diagnosis remains open.

A separate actual-linked replay of the older bounded run's185→186 prediction
also passes, SHA256
`ff93bfb0c9d789f7e247fa3d5577b9d6fd79bc5d7da03bae45134787a3661f6d`.
The12 accepted raw IMU callbacks reproduce all20 saved time/mean-state values
and seven previous-buffer values bit exactly through installed Forward.
This excludes a handoff discrepancy for that single saved step; incoming
covariance, the complete24-step chain and physical model validity are not
verified by this fixture.

Same-run first-loss review, SHA256
`e26b68fffe2a3627b0445efdd068b102c003dc86495837c54fb5b12c74d03c02`,
finds that all31 failed matcher points succeeded at181. Their saved world
points and the182 native camera pose reproduce the recorded initial projection
within1.271e-13 pixels. All pre-loss local frame/clone observations agree;
the observed discrepancies occur only after loss during frontend fallback.
Further source/patch-boundary review, SHA256
`30aebacffb2a77a9face0557a244e4ae7efef94cda05c4453bd7b7f66e88d28e`,
confirms that MetricReady bypasses SVO sparse image alignment. This remains an
unproven convergence hypothesis. Directly retaining a sparse-aligned frame pose
would conflict with Schur's clone from its original predicted state. Exact
matcher replay also lacks actual reference feature type/applicable gradient,
passed depth and eight reference pixels. A minimal observer-only witness is
being prepared to close those gaps before testing a correspondence change.

Actual frozen bootstrap reconstruction also passes, SHA256
`55db298f88b6f8ed61ce9cadcbb2b3b963acc6f595f4d733956d603ac4c32a06`:
all28 prefix decisions and seven reported values match exactly, including27
ACCEL_BIAS_BOUND rejections before167. Units, extrinsics, velocity/bias/rotation
handoff and seven retained frame histories agree. Eight one-ULP controls remain
accepted, with maximum state difference4.23e-12. Supported suffix diagnostics
show window sensitivity: removing the first five poses still passes the
original gates but changes scale0.612868→0.438914. This does not establish
physical scale or justify a new truth-tuned acceptance threshold. Unretained
frame history and initialization uncertainty remain unqualified.

The observation-only pre-loss matrix child is independently qualified, receipt
SHA256 `f7f979573639e64ad7b61390dc1ce6b46298948880cba1c6340ea3bf2715b973`.
Installation `2ddc8052174235e9c05657cc2019fafb96deb1ae31be749cbb0fc27d7d30d63d`
changes only the matrix helper and Schur observation call sites; estimator
mathematics, initialization and thresholds are unchanged. The25-package build,
180 inherited flags,53 ELF files and five preflights pass. Its strict reader
reproduces58 positive and56 rejection controls plus two independent negatives.
It requires the immediately previous actual constrained, quality-good update
in the same epoch, with no intervening processed/no-solve/reset event; bundle
successorship is reported separately. The original four-ticket/two-epoch
matrix protocol and original decoder remain intact. The fresh full V101 replay exits with TRACKING_LOST, run SHA256
`7bbc01a0fa48a3027e8c1d05028aa0f3da8d9b0c4017b08cb12cec258f224a62`:28/2912
poses, source172–199,0.96154% coverage. Scale1 ATE is0.0813856m and one-second
RPE is0.206533m/1.42457deg over only1.35s; this does not pass mono acceptance.
Generic audit SHA256
`1748848338d45134d293197386bc579afe0738e3d5a603464e64142932b4aab0` verifies
39 outputs,16 runtime identities, full input/source/devel lineage, raw/adapter
score parity, four zero child exits and owned-container absence before the
subsequent intentional host-code migration. Actual strict-reader chain199→200
is complete in epoch1. Actual factor-participation audit SHA256
`b48374905c5c3e2d549c94987945a3174e06d0471edf5c52411eac9dbf878392`
finds155 identical measurement identities reused across57 shared points.
For55 points, recorded previous phase2 covariance equals the current point
prior bit-for-bit; two points lack recorded previous covariance and remain
UNKNOWN. All16 retained-clone6x6 covariance blocks also carry exactly. This
proves factor reuse and prior persistence, not statistical double-counting
or a unique failure cause. Correlations and initialization uncertainty remain
unqualified. The original truncated all-map and free-point diagnostic failures
are preserved. This is diagnostic qualification, not a tracking repair or
promotion.

### OpenVINS terminal export candidate

A separately sealed terminal derivative uses a live ROS context through executor
callback quiescence, owned-worker joins, an optional ready-camera drain against
the latest fully returned raw IMU feed, initializer join and receipt closure.
Installation SHA256
`e9f679008628fa8e43c57a263d4e755c9d6d26bc9585528ac21d603be48b8363`
binds exactly three source changes. Three packages/four ELF closure,66 pinned
middleware helper cases, three installed-binary no-input shutdown cases and
three fresh preflights pass. The first fresh full mono0.5x replay exits normally,
run SHA256 `3ee54fcf7cf278f8a84b1c1594d26aa9b9015920bebd7424d601a1b4a453d848`,
with2,800 poses, all2,912 camera callbacks /29,120 IMU callbacks and five zero
child exits plus owned-container absence. Independent full audit passes,
SHA256 `8a14942cd41b137d08757aeb23c8762f7be3194d969cc957fdbc37696f3daecc`:
all2,800 text/export/received observer/recorded bag poses agree; all2,800
native full states match ROS header and pose binary values exactly. The same
trajectory is byte-identical to the older reliable camera-worker baseline:
scale1 ATE0.0701150107652m, one-second RPE0.0452370974100m /
0.478052290355 degrees. All50 outputs and17 runtime identities are verified.
Its terminal drain feeds zero bundles, with zero exports after quiescence,
so actual native export of a pending tail remains NOT_RUN. Health remains
UNKNOWN/invalid. This is **PC OFFLINE VERIFIED**, not flight qualification.
The separate
subscription loss, live wakeup behavior, health/reset and original repeatability
failures remain open.

The new native wire events expose a decoder compatibility defect: three actual
no-input ledgers and a composed pipeline case fail before the decoder change.
The decoder now validates explicit terminal ordering, exact integer IMU endpoint
and state-export flags/byte counts, while retaining legacy summary shape.
Returned drain does not certify full input consumption, and a publish call or
text write does not certify subscriber delivery. Delivery remains NOT_RUN;
UNKNOWN/invalid health is unchanged. Independent review additionally found
callback rejection terminals allowed after executor quiescence. Three actual
helper-derived cases fail before the correction; all callback-only terminals
now obey quiescence, including pointer reuse. Legitimate queued/unforwarded
shutdown terminals remain accepted.188 decoder tests /387 affected tests and
the fresh full regression pass:977 passed /one skipped /18.51s /exit0,34 code/fixture
identities stable and owned process group absent. Receipt SHA256
`ba923ee710d043ba58b92c02f86dcab38c95eedd30211cc30b4dd0eb35cf28d6`.
The preceding973-test receipt remains historical. Independent old-summary
readback preserves smoke PASS and repeat2 FAIL exactly, without new terminal
fields. Final cumulative independent review passes, receipt SHA256
`ed30732a329d00a25d48b5e65efbaba7328b4e95c0283693a90e6fea53a076c5`.
Only the sensor-contracts receipt is explicitly migrated in the sole matrix,
SHA256 `ed4b5b53f6b3149ec16bb872efbe704d0c2483e0f7b5ec887db72afbe227fa88`;
all75 gates/67 required cohorts, other74 gates and protocols are unchanged,
migration SHA256
`6957d57f2ae34cfaaf7777be59e4867271b96db2cc016ebc179a0b8ffa6714fa`.
Fresh selected sensor-contracts plus six malformed-input rejections finish
PARTIAL /7 PASS /0 FAIL /69 NOT_RUN, result SHA256
`c37d5e981c743e7210eb215b050ca3d479c334b83cd63952c86e08d1cfae4b64`.
The fresh full stereo0.5x smoke also completes normally,
run SHA256 `0a099e506448d360cc8feb366ee6f8dc3462c2c81f5075fe70077363e9727b52`.
Independent audit SHA256
`ba32ea50b53e781d24a08817436b302fb97fc455ddef6126bca8ad92548dd00a`
verifies2,912 images per camera,29,120 IMU,54 outputs /17 runtime identities,
five zero child exits and owned absence. All2,800 text/export/full-native-state/
received observer/recorded bag poses agree;27,998 propagated messages are also
received exactly. Scale1 ATE is0.0649570611252m; one-second RPE is
0.0439123945531m /0.469004781055 degrees. Its byte equality to an explicitly
identified older1x stereo smoke is reported only as byte equality across rates.
This run again has zero terminal-fed bundles and no exports after quiescence;
pending-tail delivery is NOT_RUN, not certified by an empty drain.
The fresh stereo1x smoke has also exited normally, run SHA256
`c34873e42b044deddd7e24b0bfde3e123b31050407b70a9b651df0edf6b10b1b`.
It produces2,800 poses with the same scale1 ATE/RPE values as this0.5x run,
full native camera consumption, five zero child exits and owned absence.
Its independent full export audit also passes, SHA256
`240e4b97b8311af2149ae4222529fb5e08dc0fa4f9484c9b51f21fea90c51f78`;
all2,800 native states/text/export/received poses and27,997 propagated messages
are reconciled. The empty terminal drain still does not exercise a pending tail.

Nine replacement immutable configuration templates pass explicit migration
review, SHA256
`ceb5487f98bf407ab0cc82d4de0b451e098419799dfb311a9bb653d9884b5170`.
Fresh18 old/new config-lock reconstructions and all installation dependencies
verify; only installation/run ID/output and their configuration/installation
locks change. All gate IDs,75 gates /67 required cohorts, rates, protocols,
calibration, scoring and original1cm criteria are unchanged. After three native
audits,15 zero child exits, exact owned absence and current34 code identities,
root explicitly activates matrix SHA256
`2ad19d451f2c6ba444591543b64ca4d3cbbf6297c868a69a24c769a94895d80b`,
activation SHA256
`9aa3bcc84e6a23f583536549523524e2ad2c3fc7313e4e99c441df05376f8104`.
The preceding ed4b5b matrix remains in its existing selected snapshot; no old
run is retagged. The first fresh fixed stereo1x repeat cohort below closes its
original1cm gate; nonempty-tail native export and full software acceptance
remain open.

The declared artificial500ms camera interruption at0.5x is also fully replayed,
run SHA256
`66a05acb0f2f784bc35a74ab3fb87ab9d4557804d7bdec5fa0d0e26dc155b9c0`.
Independent integrity audit passes, SHA256
`d83728dd2ee419c63646d29e6fd1a2659ba6facd49b3812cd1d8323a84d21ebe`.
Each camera retains2,902 of the original2,912 images: source ordinals400–409
are intentionally dropped, while all29,120 IMU samples are unchanged. Original
and derived pixels, integer times, sample bits, native input and actual ROS
transport agree. All2,790 text/export/full-state poses and27,998 propagated
messages reconcile;54 outputs /17 runtime identities and five zero child exits
are verified. Fixed-scale ATE is0.0577339077m, one-second RPE0.0442675481m /
0.464432771 degrees. Coverage keeps the original2,912-frame denominator,
112 pre-initialization omissions and ten artificial interior omissions.
The strong fault-recovery gate is still NOT_RUN because native health/reset
and watchdog recovery are unqualified. Terminal feed/late-export counts are
zero. The selected snapshot remains PARTIAL /0 PASS /0 FAIL /76 NOT_RUN,
SHA256 `271742441c7dc1ca4b67ed9baf8bd21e9649defab1bcc2e7d723c9ca9da4c71d`.
The first auditor's field-name error and its negative receipt remain preserved;
only that diagnostic accessor changed before the fresh successful audit.

Readback of all eight existing evidence gates against the active matrix also
passes, result SHA256
`68c98708fd5bea6e7e300127f0ecd5e073172b7e0801d0b4a0dd5c971b661c2e`:
8 PASS /0 FAIL /68 NOT_RUN /PARTIAL. This verifies the locked contract,
mock/SITL and ARM64 build/replay/cleanup receipts and their currency; it does
not rerun those experiments or combine separate selected snapshots into a
complete software qualification.

The active terminal-drain V101 stereo1x cohort has now completed its entire
predeclared smoke plus five repeats through the existing prehardware runner.
Independent full-input, native/full-state/ROS-output, fixed-scale evaluation,
source ancestry and cleanup audit passes, SHA256
`df88833bfda497a9a767944188937f97faf4bb0ac01af06f0cce16d78df72b8c`.
All six members contain2,800 numerical poses; each has scale1 ATE
0.0649570611252m and one-second RPE0.0439123945531m /0.469004781055 degrees.
All ten fixed repeat pairs have zero ATE difference, so the unchanged1cm
requirement passes. All30 child exits are zero and owned absence is verified.
Per-run upstream camera-update P50 is10.46–10.725ms, P95
17.6305–18.192ms, with a maximum observed update of191.49ms. The native
process-tree average CPU is0.388–0.398 cores and maximum RSS173,540–176,908KiB.
The separate whole-container sampled memory.current peak is511–582MB,
including cache. Native same-clock frontend P95 is3.33–3.45ms and decoded-queue
P95 is4.38–4.40ms; these instrumented per-camera intervals are not a split
backend timer or sensor-to-publication latency. Shared-host construction may
run concurrently, so these observations do not establish a fair algorithm
performance ranking or a real-time latency guarantee.
The selected result remains PARTIAL /1 PASS /0 FAIL /75 NOT_RUN, SHA256
`a6f246d7a3288f6a21db062b1aaa3d6b89e769ad47d161b8fa38ffe14fdeacaf`.
The prior failed worker cohort remains a failure under its own identity.
All new members have empty terminal drains; nonempty-tail export remains
NOT_RUN. UNKNOWN/invalid tracking and health/reset qualification are unchanged.

The six malformed-source input gates were also freshly executed against the
active matrix: duplicated, reordered and nonfinite IMU, each at0.5x and1x.
All are rejected before estimator execution and before creating run output.
This separate selected result is PARTIAL /6 PASS /0 FAIL /70 NOT_RUN, SHA256
`254f919949efc1474d49d6bfe41e5d7537c86a0ed2591607fedc9ab25dc46db7`.
The eight-evidence readback, fixed cohort and these six refusals are separate
snapshots and do not establish a combined complete software qualification.

An isolated GCC7.5 compile-guard probe found that `-ffast-math` followed by
`-fno-finite-math-only` bypasses the old precise-kernel macro guard. Existing
production flags explicitly disable unsafe math, so no installed-kernel
numerical defect follows from that negative. A minimal guard-only proposal
adds rejection when `__GCC_IEC_559 == 0`;28 full-translation-unit safe/unsafe
cases on pinned amd64 and AArch64 pass. Proposal audit SHA256
`00f5c20cd1d8535e58153b5749107242872a091268fdc518579c9fa7f249acb3`,
patch SHA256
`2ffd77934f56bf04cf59cf8e0fd720cb54656671c49d51b4c2450e9defa866ab`.
Only that guard delta is authorized in the new matcher-input witness child.
All bytes from the first include onward, mathematics, ABI and compile flags
remain unchanged. Other compilers, absent macros and runtime FTZ/DAZ remain
outside this guard's qualification; all frozen parent sources are preserved.

The existing private precise numerical kernel has separately completed a
standalone AArch64 build and three QEMU executions of28 matrices each.
Numerical audit passes for all84 comparisons, receipt SHA256
`26740ac129571c230c8502499ed72039cb77d5c53fb3e1670563e771a762b0dd`;
overall status remains PARTIAL because the original compile-guard negative
is retained. Four AArch64 ELF files, seven loader-closure entries and458
artifact hashes were rechecked, with equal fast/precise Eigen ABI and hidden
Eigen implementation symbols. All original binary64 matrix values survive
the17-digit text roundtrip exactly. The75 real-window iteration comparisons
have maximum solution difference8.08e-14; the nine artificial rank/near-gate
controls are reported separately, maximum3.83e-6 under the prior relative
fixture tolerance. All three target executions exit zero; owned containers
are absent and source/devel/rootfs/normalized-sysroot identities are retained.
Original ephemeral Docker argv vectors were not sealed before an earlier
harness assertion; command templates, actual outputs/exits and the exact
final sysroot command remain, without reconstructing nonexistent raw argv.
Actual ARM FPCR has FZ enabled and the PC MXCSR has FTZ/DAZ; the compile guard
does not qualify or alter those runtime modes. This verifies only the exact
standalone kernel/caller. A future selected child still requires its complete
ARM build, initialization and dataset replay; RK3576 BSP/ABI and board timing
remain unverified.


The matcher-input witness child has passed cumulative independent source,
build and fixture review, receipt SHA256
`ee2462a4961c60961e38e89d2a6c48cb3b2d367188b69841f6e6d4832c6c06e3`.
Installation SHA256 is
`8bd951bbce41e55273765d52f889ea2908a4d7d223b952bc68229d09cd1bcaaf`;
its four-file patch adds bounded actual Matcher input evidence and the qualified
compile guard, with unchanged numerical kernel objects, flags and thresholds.
The audit includes25 packages,53 ELF files,180 flags, five preflights,36 actual
Matcher-input fixture processes,87 serializer processes,39 ownership cases,
18 terminal cases and33 pristine numerical cases with zero difference.
A retained PIE sanitizer startup exit139 is also reproduced before sanitizer
initialization by a minimal no-VIO control. A fresh helper-only nonPIE terminal
qualification passes all18 cases while retaining ASan, LSan and UBSan; production
flags are unchanged. The original negatives and metadata correction remain
separate evidence, without asserting a proven ASLR mechanism. This qualifies
the diagnostic child for serial replay, not continuous mono estimation.

The current full V101 mono0.5x replay uses configuration SHA256
`69ce4fcdb19aabccd119fd60cd3ca33e15d815c4647f859b331b7f67ceff4bba`,
output `runs/v101-mono-matcher-input-witness-half`. The complete run remains FAILED /TRACKING_LOST, run SHA256
`2127485657a90466a7d3356245bcdf61d08f6e2a1cb9d17dd42fececfccbaead`:54 of2,912
poses, scale1 ATE0.0461413159545m and one-second RPE0.133184325627m /
0.628047958626 degrees. All four child exits are zero and exact owned absence
is checked. No repeats follow this failed smoke. Independent tuple/first-loss
completeness now passes under the receipts below. The compiled baseline interface
has consumed the275-case sealed cohort and qualified runtime binding; independent
review of that baseline remains pending.

The declared V101 mono1x OpenVINS smoke plus five repeats has meanwhile fully
exited through the existing runner, result SHA256
`4f4a00f15ba93f09129ffdeb9a2cfd0432ce39e9d512c54a48f5e58625c244d4`,
PARTIAL /1 PASS /0 FAIL /75 NOT_RUN. Each produces2,800 poses with scale1 ATE
0.0701150107652m and one-second RPE0.0452370974100m /0.478052290355 degrees.
The original ten fixed repeat ATE differences are zero; all six owned containers
are actually absent. Independent full-input/transport/scoring audit now passes, SHA256
`79c21fed1a212373785ae671b2b9454f05a7ecbca99a178ca5d68da876377a99`:30 zero
child exits, six exact owned containers absent,50 outputs and17 runtime identities
per member. Camera-update P50 is8.12–8.74ms and P95 is11.501–12.49ms; the maximum
180.05ms is retained. Native process-tree CPU is0.2949–0.3144 cores on average
and maximum RSS135,016–140,440KiB. Shared-host measurements do not establish a
fair performance ranking or real-time guarantee. Nonempty terminal tails remain
NOT_RUN, health/reset remains UNKNOWN/invalid, and these selected results do not
close the remaining matrix or health/reset gates.


A bounded actual-library bootstrap covariance handoff check passes, SHA256
`44c2bfbc319f0f09f39f541f16c0d5b351a668c95272cf07e4d5be5d9a7a077b`.
Actual InitializeMetric, two Forward calls and three clone augmentations reproduce
the first saved33x33 prior and20 state values bit for bit. Reset removes a
deliberately populated old prior and bias. The metric map reset also clears
point priors before their first new initialization. This excludes a captured
ordering/stale-prior transfer discrepancy in this window; it does not establish
statistical calibration of the diagonal metric/point priors or FEJ consistency.
Current/clone cross-covariance is present and propagated. Bootstrap joint and
camera–point/interpoint uncertainty remain unqualified, without evidence that
changing them would repair the observed Matcher failures. No estimator numerical
change, visual solve or dataset replay is part of this check.


Independent integrity and dynamic audits of the new witness replay pass,
SHA256 `b3fbbc8de3a8694af6bb776163b5c34ff1bb96db92d1fd06211f047ff9f15df1`
and `15a9640831a8d339f12454bcc0b34cff913e7f7ab0fc4b6d9c61a8177cdf584a`.
All16 geometry frames and4,049 actual Matcher input tuples are complete. The
first failure is bundle207:48 to30 inliers,25 lost and seven gained; the lost
population comprises seven alignment failures,11 occupied-grid skips and seven
current4px rejections. Frame/clone discrepancies arise only after this first
failure, beginning208, so they do not explain207. First-loss readback SHA256
is `9fa14ed248938c0c1ea54e442a01c6dd55aa196fdc7e688413d7592dfdc33118`.
The short post-trigger window explicitly records three unavailable frames at
the actual epoch end; the16-frame terminal window is complete. Observer and
integrity PASS do not change the estimator's FAIL verdict.

An actual installed-Matcher baseline has reproduced all129 alignment failures
in207 and all146 successes in206, with enum, search level, returned pixel and
accepted bearing bit exact. The275-case receipt SHA256 is
`ab0397fb89441ddf246a7d781cd03d7a173eed272ff3e2ae8c93d858d7168b03`;
independent review is pending. This establishes restoration of the captured
inputs, not a failure cause. Seven nominal lost-inlier cases are selected in
advance for bounded pixel-seed and iteration-count controls only after that
review. Even recovery of all seven would leave37 below the unchanged40-inlier
requirement; no production matching fallback or threshold change is made.
