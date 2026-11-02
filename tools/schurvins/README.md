# SchurVINS public-data reproduction tools

This independent entry runs the pinned SchurVINS system with its included SVO2.0
frontend. It does not change the existing `backend=OpenVINS` runner. Actual
results and platform limitations belong in the
[integration record](../../.agents/docs/SCHURVINS_INTEGRATION.md).

The tools require Linux, Docker, Python 3.10+ with NumPy/OpenCV on the host, and
enough external storage for sources, the image, build and dataset. All commands
below run from the repository root. Sources, builds, public datasets and replay
outputs stay outside Git. Every build/run output directory must be new.

## Source and environment preparation

`sources.lock.json` fixes SchurVINS
`d8ab6dff20860694d8d879d716d7fd6cb2c2530e`, eleven direct dependencies and the
Ceres/gflags source archives by commit and SHA256. Normal preparation never
resolves a moving branch. `--resolve-lock` is a separate maintainer operation
which refuses to replace an existing lock; do not use it to reproduce this pin.

```bash
export SCHUR_RESEARCH=/home/shiuhou/Projects/rm27-schurvins-research
python3 -m tools.schurvins.prepare --workspace "$SCHUR_RESEARCH/ws"
docker build -f tools/schurvins/Dockerfile \
  --build-arg BUILD_UID="$(id -u)" --build-arg BUILD_GID="$(id -g)" \
  -t rm27-research/schurvins:melodic tools/schurvins
python3 -m tools.schurvins.build --workspace "$SCHUR_RESEARCH/ws" \
  --image rm27-research/schurvins:melodic \
  --output "$SCHUR_RESEARCH/build-pc-new"
```

The Docker base is the digest-pinned Ubuntu 18.04/ROS Melodic image named in
`Dockerfile`. The resulting image's immutable ID and complete installed package
versions are recorded in `installation.json` and `packages.txt`. `packages.lock`
freezes the complete 1,124-package closure; the image build rejects a different
installed version list. Reuse the recorded image ID for comparisons. A later
apt mirror may no longer provide this closure, so the recipe can fail instead
of silently accepting a different environment. It is not a historical apt snapshot.
Retain/export the built image when reproducing those exact package bytes matters.

`build.py` applies the recorded `platform.patch`, excludes loop/global-map and
unused visualization/benchmark packages, and builds `svo_ros` offline with
4 CPU/12 GiB limits. The patch disables loop closure and selects target architecture flags;
it does not change estimator equations. Ceres/gflags downloads use local
hash-checked archives. System Eigen/glog are selected explicitly. Catkin testing
is enabled because `eigen_checks` requires its gtest definitions; a successful
build does not mean upstream tests were executed.

The manifest freezes source/devel files and symbolic-link names/targets,
executable hash, image ID and patch hashes. Runtime verifies file membership and
link targets before and after execution. Its executable is the real
`devel/.private/svo_ros/lib/svo_ros/svo_node` artifact. Catkin's public alias
targets `/ws` inside the container and need not resolve on the host. Do not
rewrite those aliases or original manifests to make host path checks pass.

## Prepare the full EuRoC input

Use an original ASL-format ZIP containing `mav0/cam0`, `cam1`, `imu0` and
`state_groundtruth_estimate0`. The existing V1_01 archive is usable at:

```bash
export EUROC_ARCHIVE='/home/shiuhou/Projects/archive/worktrees_20261016/rm27-vio-20261007_mixed_experiments/vicon_room1(1)/vicon_room1/V1_01_easy/V1_01_easy.zip'
python3 -m tools.schurvins.dataset --archive "$EUROC_ARCHIVE" \
  --mode mono --output "$SCHUR_RESEARCH/inputs/v101-mono-new"
python3 -m tools.schurvins.dataset --archive "$EUROC_ARCHIVE" \
  --mode stereo --output "$SCHUR_RESEARCH/inputs/v101-stereo-new"
```

Preparation hashes the original archive, copies only selected cameras/IMU and
their sensor calibration files into `sensors.zip`, and records every decoded
image hash, integer timestamp and IMU value in `expected.jsonl`. Stereo requires
identical source timestamps for the two image streams. Nothing is retimed,
interpolated or scaled. Ground truth is excluded from the prepared input.
`input.json` records selected streams, counts, endpoints and artifact hashes.
The native estimator separately loads the pinned upstream EuRoC calibration;
its actual parameters are dumped for each run rather than replaced by the
dataset metadata. The original archive and all existing manifests stay unchanged.

## Run a fresh full-sequence smoke test

Save the following JSON outside Git, choosing paths from the successful build
and preparation above. `output` must not exist. The original archive is only
read by the host for scoring; it is never mounted into the estimator container.

```json
{
  "schema_version": 1,
  "backend": "SchurVINS",
  "purpose": "EUROC_PUBLIC_BASELINE",
  "run_id": "v101-mono-half-new",
  "sequence": "V1_01_easy",
  "mode": "mono",
  "rate": 0.5,
  "input": "/home/shiuhou/Projects/rm27-schurvins-research/inputs/v101-mono-new",
  "installation": "/home/shiuhou/Projects/rm27-schurvins-research/build-pc-new/installation.json",
  "reference_archive": "/home/shiuhou/Projects/archive/worktrees_20261016/rm27-vio-20261007_mixed_experiments/vicon_room1(1)/vicon_room1/V1_01_easy/V1_01_easy.zip",
  "output": "/home/shiuhou/Projects/rm27-schurvins-research/runs/v101-mono-half-new",
  "timeout_s": 900
}
```

```bash
python3 -m rm27.perception.vision.localization.experiments.schurvins_runtime \
  --config "$SCHUR_RESEARCH/configs/v101-mono-half-new.json"
```

For stereo, change `mode`, prepared input, run ID and output together. The
dedicated runner accepts `V1_01_easy`, the locally available `V1_03_difficult`,
`V2_02_medium` and `MH_04_difficult`, with rates 0.5 and
1.0. The primary accuracy group is the full sequence at 0.5x. Keep 1x performance
and any separately defined source-window experiment in distinct groups. After
a complete smoke passes, use five fresh configs with identical settings and
different run IDs/output paths, executing sequentially. Retain every failed run;
do not replace a failed member or hide missing trajectory intervals.

The runtime verifies frozen inputs/installations and uses a network-isolated
4 CPU/12 GiB container with read-only source/build/input mounts. ROS1 communicates
on container loopback. It explicitly overrides `runlc=false`,
`use_global_map=false`, `use_ceres_backend=false` and
`automatic_reinitialization=false`. The latter prevents a frontend restart from
silently stitching states whose Schur filter lifecycle is unqualified. These
differences from the upstream launch are recorded, not called an unchanged
upstream execution. Every Docker exit, timeout or interruption attempts removal
of only that run's uniquely named container and verifies its absence. If the
daemon/inspection cannot confirm absence, output remains unsealed and is not
scored or read by the summary tool. The runner acquires output-directory
ownership only after exclusive creation succeeds, including concurrent attempts.

## Sealed recordings and independent camera sessions

The existing runtime also accepts a sealed platform-independent sensor recording.
This is a separate `SENSOR_RECORDING` transport format; it is never relabeled as
EuRoC. Prepare one independent monocular source epoch with:

```bash
python3 -m tools.schurvins.dataset --dataset SENSOR_RECORDING --mode mono \
  --source-recording /external/recording --source-epoch 0 \
  --native-parameters /external/native-parameters.json \
  --output /external/prepared-recording
```

The recording must pass `experiments.sensors.load_recording`. Intrinsics contain
`[fx, fy, cx, cy, four distortion coefficients]`; supported models are
`pinhole-radtan` and `pinhole-equidistant`. Intrinsics, `T_imu_camera` and the
signed camera–IMU offset need explicit source evidence. Unknown calibration
is refused. Native calibration uses the sourced camera-to-IMU transform and
`delay_imu_cam = -camera_imu_offset_ns / 1e9`; sensing timestamps stay intact.
Exposure, shutter and physical synchronization remain unqualified.

The parameter JSON has `schema_version: 1`,
`purpose: "NATIVE_IMU_EXPERIMENT_PARAMETERS"`, `measurement_binding`,
`imu_params` and `imu_initialization`. The binding names the source IMU ID/frame,
`measurement_kind: "RAW_SPECIFIC_FORCE"`, `native_axes_transform: "IDENTITY"`,
and the SHA256 of the canonical source IMU configuration as
`source_imu_config_sha256`, with a sourced `CODE_SUPPORTED` or `VERIFIED`
evidence record. Supply all native IMU noise, rate, limits and initial-prior
fields explicitly; this entry never supplies EuRoC defaults. These experiment
parameters are recorded independently from hardware calibration evidence.

Use the same runtime command with `purpose: "SEALED_SENSOR_RECORDING_REPLAY"`,
the prepared input, `mode: "mono"`, an explicit sequence label and
`reference_archive: null`. Source sensor IDs, raw timestamps, source sequences,
source epochs, decoded pixels and gaps are checked against the original seal
before execution. The selected camera maps explicitly to native `cam0`.
No-reference scoring and generic operational-state qualification remain
`NOT_RUN`. Even a mechanically successful replay has overall `PARTIAL`, with a
separate `recording_interface_status: "PASS"`; it is not algorithm reproduction.

A separate `recording_transport_status` records exact publisher/observer
reconciliation and clean process/container exit even when metric initialization
or pose output fails. It does not prove estimator ingestion or initialization;
`recording_interface_status` and the overall result retain their stricter gates.
Unknown/reference-free accuracy stays `NOT_RUN`.

For front/down selection, the same runtime entry accepts a
`SEALED_SENSOR_RECORDING_SESSION` JSON containing `schema_version`, `backend`,
`run_id`, a fresh `output`, and ordered `segments` of ordinary recording-run
configurations. Each segment has a unique run ID/output beneath that session.
All inputs are checked before starting, then rechecked at their boundary. Each
segment uses a separate native process session. The next segment waits for
verified removal of the previous owned container, including after failures.
Session epochs bind the separate run manifests; trajectories and native reset
counters are not stitched. Unknown or nonoverlapping two-camera fusion is
refused; logical camera aliases do not establish physical viewing direction.

## Evidence and scoring

Each started run retains `run.json`, commands/tool hashes, ROS parameters,
process logs/exits, native `logs/imu_traj.txt`, input publication/observer JSONL
and raw pose/Info messages. `native-resource.txt` measures the native process
tree; `resources.jsonl` samples the entire container including transport and
ROS. Native Info timing is frontend time. Observer receipt lag is not processing
completion or sensor-to-publication latency. Published/observer input equality
does not establish complete native IMU ingestion.

The runtime hashes its adapter, timestamp/frame contract and reused evaluation
math sources before/after execution. Standalone evaluation also records the
`native_accuracy.py` math-source hash, so a wrapper-only hash cannot be mistaken
for a complete scoring implementation identity. Per-run provenance stays
authoritative when utilities change between separate cohorts.

When checks permit, `online/trajectory.json` preserves the existing
`OnlinePoseRecord`/`LocalizationEstimate` v2 contract. Native stage/quality and
placeholder covariance stay raw evidence; standard tracking/initialization are
unknown and operational validity is false. A numeric pose is not flight approval.
The reviewed wrapper exposes frame-driven visual states, not a separately
qualified high-rate propagated stream. Adapter epoch zero is not a measured
native reset count.
An emitted camera-triggered state also does not prove a successful visual
correction on that frame; per-frame correction/propagation semantics remain
unknown without additional native instrumentation.

Scoring reuses the OpenVINS fixed-scale SE(3), ATE, adjacent and 1-second RPE
functions. It retains original truth hashes and timestamps, reports coverage
before/within/after output, and scores reset epochs separately. No scale or time
offset is fitted. Common intervals supplement each complete run. Original
V1_01 orientation truth has known limitations; keep that caveat with rotation
errors. A separate saved-output comparison can be run without ROS:

```bash
python3 -m tools.schurvins.evaluate --archive "$EUROC_ARCHIVE" \
  --poses "$SCHUR_RESEARCH/runs/v101-mono-half-new/online/trajectory.json" \
  --compare-poses /absolute/path/to/openvins/online/trajectory.json \
  --output "$SCHUR_RESEARCH/comparison-new.json"
```

`PASS` requires actual execution, clean process exits, full selected input
reconciliation, an evaluable epoch, raw/text/adapter parity, the observed native
tail and no observed tracking loss/output gaps. A build or synthetic test pass
alone is not reproduction. Refusal/failure artifacts remain available; partial
numeric evaluation never erases a failed reliability or process gate. The
OpenVINS historical 1 cm repeatability gate remains unchanged and is not an
allowed inter-algorithm accuracy difference.

Aggregate explicitly planned repeat cohorts after all runs end:

```bash
python3 -m tools.schurvins.summarize \
  --root "$SCHUR_RESEARCH/runs" --root "$SCHUR_RESEARCH/openvins" \
  --repeat-group openvins "$SCHUR_RESEARCH"/openvins/v101-mono-half-repeat{1..5} \
  --repeat-group schur_stereo "$SCHUR_RESEARCH"/runs/v101-stereo-half-repeat{1..5} \
  --output "$SCHUR_RESEARCH/comparison-summary-new.json"
```

The summary retains every failed/unevaluable member and verifies that a repeat
cohort shares the same fixed configuration. Schur timing uses stage=2 samples;
paused Info can contain a stale timer. OpenVINS camera-update timing has a
different internal boundary. Native maximum RSS is KiB; whole-container
`memory.current` includes cache and is not RSS. Missing measurements stay
`NOT_RUN`. Repeat trajectory differences remove only the first common pose's
rigid world gauge, matching original camera IDs with no scale correction.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_schurvins_prepare.py tests/test_schurvins_projection.py \
  tests/test_schurvins_runtime.py tests/test_schurvins_transport.py
```

## Common sensors, diagnostics and pre-hardware gates

`experiments.sensors` validates a platform-independent camera/IMU recording,
retaining sensor IDs, sequence/epoch, raw clock evidence, explicit mappings,
payload and calibration hashes. EuRoC and genuine UZH text/PNG intakes use that
same contract. Unknown exposure/synchronization remains unknown. Independent
front/down configurations are supported; non-overlapping stereo is refused.
Mapped integer timestamps must increase within each sensor epoch; quantization
collisions are rejected even when the original timestamp increases.

```bash
python3 -m rm27.perception.vision.localization.experiments.sensors validate --input /absolute/recording
python3 -m rm27.perception.vision.localization.experiments.sensors replay --input /absolute/recording --rate 1
python3 -m rm27.perception.vision.localization.experiments.prehardware \
  --lock-config /absolute/replay-config.json
python3 -m rm27.perception.vision.localization.experiments.prehardware \
  --matrix /absolute/matrix.json --output /absolute/new-acceptance --execute
# A selected invocation retains every other declared gate as NOT_RUN.
python3 -m rm27.perception.vision.localization.experiments.prehardware \
  --matrix /absolute/matrix.json --output /absolute/new-selected-acceptance \
  --execute --gate sensor-contracts --gate schur-fault-imu_duplicate-half
```

The matrix calls existing estimator runners sequentially. It fixes configuration
and artifact identities, distinguishes 0.5x/1x and smoke/five-repeat groups,
preserves failures and refuses old-code receipts as current acceptance. Missing
required gates yield `PARTIAL`; a synthetic receipt cannot substitute for native
monocular/high-motion/recovery evidence. Inspect the formal integration record
for actual executed results, rather than inferring acceptance from these commands.

The single external declaration is currently
`/home/shiuhou/Projects/rm27-schurvins-research/configs/prehardware/matrix.json`.
Its 67 required configurations cover full EuRoC mono/stereo, matching OpenVINS
comparisons, UZH forward 3/6 and 22 independent faults at both playback rates.
One additional OpenVINS stereo 500 ms camera-gap cohort uses the same comparison
runner with separated original-reference and derived-sensor provenance.
`required_cohorts` binds each complete template by `config_file` and
`config_sha256`; an omitted rate, sequence, input or protocol remains `NOT_RUN`.
Baseline `cohort` gates use `repeat_count=5` and
`protocol=SMOKE_PLUS_FIVE_REPEATS`. Complete numerical smoke results still
collect the fixed repeats when newer native health evidence is absent; the
stronger state gate stays open. Valid faults use `repeat_count=0` and
`protocol=SINGLE_RUN`. Invalid IMU perturbations use `kind=input_rejection` and
`protocol=INPUT_REJECTION`: only the existing runtime preflight executes,
`FAULT_INPUT_REJECTED` is required, and the estimator remains `NOT_RUN`.

Native Schur state acceptance requires explicit metric initialization with its
filter commit and actually accepted visual constraints. Numerical pose output
alone retains its original replay result. Recovery also requires an audited
independent watchdog timeline and new post-fault visual correction evidence;
Info callback receipts do not replace timer evidence. Observer ticks have no
invented sensor timestamp. Unknown calibration, exposure, covariance and time
mapping remain unknown, and sensor metadata cannot contain nested truth values.

`tools.schurvins.dataset.derive_prepared` reuses the common sensor disturbance
implementation. It preserves the full original `source-expected.jsonl` scoring
denominator and writes the delivered payload ledger separately. Delays change
only the declared delivery schedule. Future delay-only archives share a frozen,
hash-checked parent archive; transformed data retain their own payload archive.
The storage estimate runs before temporary materialization and reserves 20 GiB
on both affected filesystems. Reconciliation checks the single publisher's
global schedule and each observer topic's payload order separately; concurrent
observer callbacks do not establish a global reception order.
Stereo random dropout uses the explicit `SAME_SOURCE_TIME` policy with one
seeded draw per original verified pair. Fault inputs are artificial experiments,
kept separately from the unmodified public sources.

On 2026-10-31, the affected contract/transport/acceptance tests actually completed
with **263 passed**, including the OpenVINS fault comparison extensions. The
sealed test/process evidence is
`.agents/runs/schurvins/prehardware_interfaces_v9/`. All 22 real EuRoC-derived
fault inputs were prepared and audited: 19 valid sensor recordings and three
intentionally invalid IMU recordings. Their counts, source hashes and profiles
are in the external `configs/faults/matrix-prepared.json`. Independent V1_01
mono OpenVINS comparison input also passed full pixel/IMU/timestamp readback;
the original VIO-P input and 1 cm repeatability gate remain unchanged. These
are input and software checks; the matrix does not infer estimator, recovery,
RK3576 or flight success from them. Historical receipts remain separate when
their source identities cease to match the current code.

The same OpenVINS comparison entry completed a full 0.5× V1_01 stereo replay
with an artificial 500 ms camera interruption. It retained the original
2,912-frame denominator, produced 2,790 visual poses and fixed-scale ATE RMSE
0.0566455682 m, with matching raw/adapter scores and every process exiting zero.
Native reset, watchdog and accepted visual recovery evidence remain unknown;
the numerical replay does not close the recovery gate. See
`../openvins/README.md` for the explicit input and reference separation.

The current 75-gate matrix was also executed with only its eight audited
evidence gates and six invalid-input rejection gates selected. All 14 selected
checks passed, with zero FAIL and 62 NOT_RUN entries including the aggregate
coverage gate. The 67 required experiment cohorts retain six preflight rejection
passes and 61 NOT_RUN results; this invocation starts no estimator. Its
evidence is `.agents/runs/schurvins/prehardware_selected_v6/`. The overall result
remains `PARTIAL` and does not replace native baseline or recovery experiments.

The optional `native_parameters` run-config object permits only explicit
initialization/recovery experiment controls. `state_output_policy` must specify
all `StateOutputPolicy` fields and represents test limits, not flight limits.
`start_offset_s` is an explicit supplemental prefix skip; original camera IDs
and full-sequence coverage stay intact. `memory_limit_gib` selects a distinct
4 or 12 GiB group. Default OpenVINS dispatch is unchanged.

For an explicitly named **experimental RM27 SchurVINS derivative**, prepare a
fresh external workspace and pass `--patch tools/schurvins/native-diagnostics.patch`
and `--patch tools/schurvins/mono-initialization.patch` to the existing build
command. Every build preserves exact applied patch bytes beside its installation
manifest. This derivative is not accepted merely because it compiles or passes
synthetic alignment cases. Keep the upstream source/control and failed runs.

Native begin/end/initialization/correction/reset events are parsed into
`native-observations.json`; numerical pose projections remain invalid by default.
Only complete native metric initialization and actually accepted visual state
evidence can enter the separate v2 state/watchdog interface. Original native
publication/completion timestamps and unexported covariance remain unknown.
High-rate propagation uses an independent corrected snapshot and does not refresh
visual-update freshness or write back to the Schur filter.

For the explicit input-health derivative above, apply
`--patch tools/schurvins/native-imu-callback-boundaries.patch` after the input-health
and input-restart-coverage patches. It adds observation only; retain the full
parent patch list and exact installation identity. `run_inner.py` requests the
separate `native-imu-ledger.jsonl`. The ledger preserves each callback's header
sequence, integer timestamp, frame bytes and six-axis double bits. Its bounded
storage is reserved before subscriptions and sealed after callback-thread joins.
Missing callbacks, truncation or an unclean footer cannot pass completeness.
The runtime compares delivered occurrences in order with independently reconciled
observer rows. No sensor-time sorting or gap filling occurs.

`native-observations.json` and the benchmark summary report callback coverage,
actual buffer-insertion disposition and callback duration separately. Accepted
buffer insertion does not establish downstream integration, reset ownership or
tracking health. Existing binaries without this header retain NOT_RUN; missing
ledgers from instrumented binaries fail while preserving auditable trajectory
scores. This observation does not qualify monocular estimation or live VIO.

`localization.state_output` supplies offline conversion and optional isolated
MAVLink2 loopback. Explicit world-to-NED and IMU-to-body evidence, native velocity
and valid/fresh state are required. This does not connect to a physical flight
controller or prove fusion. See its CLI `--help` and the retained mock evidence.

## ARM64 and later datasets

`arm64.py` provisions a separate Ubuntu 18.04 / ROS Melodic reference rootfs
from a pinned ARM64 image and an independent 1,117-package ARM64 lock. It
extracts pinned local PRoot, libtalloc and QEMU tools; it does not install host
packages or register binfmt. Signed apt verification remains enabled. Provision
and build outputs must be new external directories. Each operation checks the
20 GiB free-space reserve before downloading/building.

```bash
python3 -m tools.schurvins.arm64 provision --root "$SCHUR_RESEARCH/arm64" \
  --output "$SCHUR_RESEARCH/arm64/provision-new"
python3 -m tools.schurvins.arm64 build --root "$SCHUR_RESEARCH/arm64" \
  --workspace "$SCHUR_RESEARCH/arm64/ws" \
  --output "$SCHUR_RESEARCH/arm64/build-new" --memory-gib 12
python3 -m tools.schurvins.arm64 neon --root "$SCHUR_RESEARCH/arm64" \
  --workspace "$SCHUR_RESEARCH/arm64/ws" \
  --output "$SCHUR_RESEARCH/arm64/neon-new"
python3 -m tools.schurvins.arm64 package --root "$SCHUR_RESEARCH/arm64" \
  --workspace "$SCHUR_RESEARCH/arm64/ws" \
  --installation "$SCHUR_RESEARCH/arm64/build-new/installation.json" \
  --output "$SCHUR_RESEARCH/arm64/package-new"
```

Builds execute target compilers explicitly under PRoot/QEMU, inside a 4 CPU
systemd scope with a selected 4 or 12 GiB memory cap. The source/workspace is
independent of frozen PC installations. The recorded portability patches select
AArch64 flags and accept GCC's `__ARM_NEON` macro alongside upstream's
`__ARM_NEON__`; estimator equations are unchanged. The NEON probe compares
scalar and target NEON corner results, rather than treating a configure pass as
compiled-code verification.

`verify` checks the real executable's AArch64 ELF header and asks the target
dynamic loader for its complete library closure, checking each library's ELF
header and hash. `installation.json` additionally freezes source/devel files,
target rootfs regular files and symlink targets, emulator/helper hashes and
target package provenance. Absolute target symlinks are resolved inside the
target rootfs, never against host `/lib`.

For a recorded RM27 derivative, reuse `build` with `--base-installation` and
one `--patch` for each frozen patch, in application order. The reference must
still include its measured build cache. It is checked before copying into a new
independent workspace; the reference source/devel/rootfs remain unchanged.
Incremental compilation mounts the shared rootfs/helpers read-only, disables
container networking, uses 4 CPUs with the selected memory limit, and polls the
20 GiB reserve while compiling. A storage failure stays `BLOCKED` after cleanup.
For example, the completed input-boundaries derivative used:

```bash
python3 -m tools.schurvins.arm64 build --root "$SCHUR_RESEARCH/arm64" \
  --workspace "$SCHUR_RESEARCH/arm64/ws-input-boundaries-new" \
  --output "$SCHUR_RESEARCH/arm64/build-input-boundaries-new" --memory-gib 4 \
  --base-installation "$SCHUR_RESEARCH/arm64/build-clean/installation-runtime.json" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/native-diagnostics.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/mono-initialization.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/stereo-observation.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/tracking-quality.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/input-health.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/native-time-resolution.patch" \
  --patch "$SCHUR_RESEARCH/build-input-boundaries/patches/input-restart-coverage.patch"
```

The existing result is at `arm64/build-input-boundaries/installation.json`, with
workspace `arm64/ws-input-boundaries`, relative to `$SCHUR_RESEARCH`. Its
25-package build and target-loader closure (238 libraries) passed under a 4 GiB
limit. These are `ARM64 BUILD VERIFIED`. The separately declared derivative core-only
full stereo replay and timeout/SIGINT/native-child cleanup probes have also
completed under QEMU (`EMULATED / MOCK VERIFIED`); exact receipts and limitations
are in the integration record. Host-state-policy replay and board qualification
remain separate. Reference-version results do not qualify the derivative. The incremental build command above needs additional disk space and
new output paths; it is not a request to overwrite the retained result.

`replay_command` provides argv for the same frozen `run_inner.py` and
`ros_transport.py` used by the existing replay runner. It does not start an
estimator or implement another runner. The caller validates the installation,
creates an empty `runtime-tmp` directory in its exclusively owned output, and
owns timeout/container cleanup. The pinned amd64 helper image supplies the
host loader for PRoot; it is explicitly distinct from the ARM64 target image.
Rootfs, workspace, transport and sensor inputs are mounted read-only. Output
and guest temporary files are writable only within that run's output. Container
networking is disabled; PRoot requires container-local ptrace and executable
temporary storage, without host binfmt or physical device access.

The deployment archive includes the verified reference rootfs plus source/devel
workspace. It assumes `/ws` inside the target reference environment. It does
not establish compatibility with the still unknown RK3576 BSP, loader, ABI or
drivers. `aarch64-linux.cmake` remains available for cross-compilation with an
explicit `SCHURVINS_SYSROOT`; a standalone cross-compiled probe is not a full
catkin estimator build. Actual build/replay outcomes remain in the integration
record, with build, emulation and board evidence distinguished.

An existing reference deployment archive can be reused without duplicating its
rootfs. The same `package` action accepts both `--base-installation` and
`--base-archive` to create a small `overlay.tar.gz` plus `package.json`:

```bash
python3 -m tools.schurvins.arm64 package --root "$SCHUR_RESEARCH/arm64" \
  --workspace "$SCHUR_RESEARCH/arm64/ws-input-boundaries" \
  --installation "$SCHUR_RESEARCH/arm64/build-input-boundaries/installation.json" \
  --base-installation "$SCHUR_RESEARCH/arm64/package-reference/installation.json" \
  --base-archive "$SCHUR_RESEARCH/arm64/package-reference/reference.tar.gz" \
  --output "$SCHUR_RESEARCH/arm64/package-input-boundaries-new"
```

The retained `arm64/package-input-boundaries` result contains a 7,734,125-byte
overlay with 23 replaced/added workspace paths; `removed_paths` is empty for
this particular package. Packaging checked the actual reference archive,
restored its workspace into bounded temporary staging, applied the overlay and
compared the complete source/devel file hashes and symlink targets with the
derivative installation. Temporary staging was removed. Package outputs must be
separate from source/devel, target userspace/helpers and source evidence.

For installation into a new external reference environment:

1. Verify the base archive and overlay SHA256 against their `package.json`
   records, including the overlay's `base_archive_sha256` and both installation
   manifest hashes. Retain these manifests and the overlay's `metadata/` files.
2. Extract the reference archive into an empty owned destination. Compare its
   `ws/src` and `ws/devel` membership, file hashes and symlink targets with the
   base installation manifest before applying any change.
3. Apply the overlay's declared `removed_paths`, then extract its replacement
   files into that destination. Compare the complete source/devel trees and
   symlink targets with `metadata/installation.json`; preserve the guest `/ws`
   binding and absolute catkin aliases. The overlay contains no rootfs.
4. Verify the AArch64 ELF, target loader and dependency closure in that exact
   reference userspace. Host absolute paths in retained manifests are provenance,
   not relocatable deployment settings. A relocated environment needs its own
   verified installation/configuration; do not rewrite original manifests.

Launch an already verified local ARM64 installation through the existing
`experiments.schurvins_runtime --config ...` entry shown above, selecting its
ARM64 installation manifest, prepared input, explicit memory limit and a fresh
output directory. Stop it with Ctrl+C/SIGINT and inspect the resulting process
exits and owned-container absence; an unverified cleanup cannot pass acceptance.
The prepared derivative configs
`arm64/configs/v101-{mono,stereo}-input-boundaries-half-4g-current2.json` are
preflight-only until actually executed; never reuse an existing output path.
The separate core-only configuration
`arm64/configs/v101-stereo-input-boundaries-core-half-4g-current2.json` has
completed, with evidence at `arm64/acceptance/input-boundaries-core-current2/`.
It omits `state_output_policy`: numerical functionality and clean exit passed,
while host state projection and operational validity remain NOT_RUN.
No command here starts a physical camera/IMU, connects a flight controller or
qualifies a new RK3576 BSP. Actual board installation, startup, ABI, drivers and
real-time performance remain `NOT_RUN`.

Front/down cameras must first be assessed independently. EuRoC ExactTime stereo
support does not prove non-overlapping-camera fusion or hardware synchronization.
UZH-FPV release/truth/calibration intake and labeled disturbances retain separate
input provenance. PC replay, cross-compilation and synthetic checks do not qualify RK3576
real-time behavior, a new camera/IMU chain, contact recovery or flight.
