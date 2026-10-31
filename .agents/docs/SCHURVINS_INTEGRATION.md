# SchurVINS: M3C deployment and shared reproduction

The current independent M3C branch results are in the
[M3C deployment record](M3C_DEPLOYMENT.md). The following imported record
preserves its original PC/RK3576 observations and historical status; it is not
a new M3C run or an update to the active RK3576 task.

SchurVINS is an authorized VIO candidate for RK3576; OpenVINS remains the
comparison baseline. This work does not qualify a camera/IMU assembly, RK3576
runtime, estimator/controller integration or flight. The existing OpenVINS
VIO-P repeatability and VIO-S0 evidence gates remain unchanged. Historical
MicoAir/PX4 losses are retained evidence, not prerequisites for a new independent
IMU chain.

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

Source references: [`svo_interface.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo_ros/src/svo_interface.cpp),
[`visualizer.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo_ros/src/visualizer.cpp),
[`frame_handler_base.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo/src/frame_handler_base.cpp),
[`schur_vins.cpp`](https://github.com/bytedance/SchurVINS/blob/d8ab6dff20860694d8d879d716d7fd6cb2c2530e/svo/src/schur_vins.cpp).

## ARM64 and RK3576 readiness

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

`tools/schurvins/arm64.py` prepares the rootfs, explicit PRoot/QEMU helpers,
independent source workspace and target manifests. Bionic QEMU 2.11 works with
the selected PRoot; the newer host QEMU/PRoot incompatibility remains retained
negative evidence. The ARM-only FAST patch fixes both architecture selection
and `__ARM_NEON`/`__ARM_NEON__` guards without changing the frozen PC source.
Actual AArch64 and separately cross-compiled FAST/NEON binaries agreed on all
18 test cases / 12,466 corners. This verifies those target code paths, not the
entire estimator. The full 25-package target build is still running; no complete
ARM64 estimator build or replay is claimed at this milestone. Evidence is under
the external root's `arm64/` directory, including `rootfs-elf-scan.json` and
`cross/neon-sysroot/`.

The fixed Bionic GCC 7.5 cross toolchain and a target sysroot are now available;
missing tools are no longer an external blocker. Actual dynamic loader/library
resolution, complete estimator ELF checks, QEMU replay, a deployable package
and the separately labeled 4 GiB resource group remain acceptance items. The
PC amd64 image digest must never be used as an ARM64 rootfs identity.

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
per-run source hashes. The final full software suite, with ROS2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q --ignore=tests/camera_capture.py
```

Result: **296 passed, 1 skipped** (existing ChArUco environment skip).
`git diff --check` passed. No replay controller/estimator or container remains
running. Software regression and cleanup probes do not certify live VIO.

### Remaining acceptance gaps

- **FAIL:** Schur monocular replay and same-input monocular comparison. Preserve
  both one-pose failures; investigate native mono initialization/filter lifecycle
  before an independently recorded fix or a newly declared diagnostic run.
- **BLOCKED:** MH_04 download (old ETH SSL timeout; new Research Collection
  DOI `10.3929/ethz-b-000690084` returned HTTP 429 locally). V1_03 is a verified
  difficult-sequence substitute, not evidence that MH_04 ran.
- **NOT_RUN (pending active build):** full ARM64 estimator verification. Toolchain,
  target rootfs, independent dependency lock and actual NEON tests are available.
  Complete ELF/library checks and target replay must follow a successful build.
- **NOT_RUN:** UZH-FPV estimator replay/disturbances (real input intake complete), RK3576 target build/runtime,
  camera/IMU hardware qualification, live latency/thermal/4 GB memory behavior,
  tracking/reset recovery and flight. Calibration/source feasibility above does
  not close these gates.
