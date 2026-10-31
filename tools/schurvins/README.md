# SchurVINS public-data reproduction tools

This independent entry runs the pinned SchurVINS system with its included SVO2.0
frontend. It does not change the existing `backend=OpenVINS` runner. Actual
results and platform limitations belong in the
[integration record](../../.agents/docs/SCHURVINS_INTEGRATION.md).
The independent M3C branch adds a [ROS-free C++ runtime](headless/README.md)
and a separate [deployment result](../../.agents/docs/M3C_DEPLOYMENT.md).

The tools require Linux, Docker, Python 3.10+ with NumPy/OpenCV on the host, and
enough external storage for sources, the image, build and dataset. All commands
below run from the repository root. Sources, builds, public datasets and replay
outputs stay outside Git. Every build/run output directory must be new.

## M3C branch software boundary

On `research/m3c-vio`, use the independent external research directory shown
below. The RK3576 worktree and `/home/shiuhou/Projects/rm27-schurvins-research`
are read-only recovery/comparison sources. Board connection is paused at the
user's request. Offline software work does not authorize capture or UART access.

The implemented [ROS-free C++ runtime](headless/README.md) rebuilds the estimator
without ROS and runs ordered ASL camera/IMU inputs. Native process measurements
use `python3 -m tools.schurvins.benchmark --help`; invoke it beside the actual
binary, never around an SSH or Docker client. `headless_evaluate` scores recorded
poses after execution, retaining separate execution, numerical and operational
gates. `python3 -m tools.schurvins.m3c --help` describes the read-only inventory
collector; it is not a camera or UART opener. Current measured evidence is in the
M3C section of the integration record.

## Source and environment preparation

`sources.lock.json` fixes SchurVINS
`d8ab6dff20860694d8d879d716d7fd6cb2c2530e`, eleven direct dependencies and the
Ceres/gflags source archives by commit and SHA256. Normal preparation never
resolves a moving branch. `--resolve-lock` is a separate maintainer operation
which refuses to replace an existing lock; do not use it to reproduce this pin.

```bash
export SCHUR_RESEARCH=/home/shiuhou/Projects/rm27-m3c-schurvins-research
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
  "input": "/home/shiuhou/Projects/rm27-m3c-schurvins-research/inputs/v101-mono-new",
  "installation": "/home/shiuhou/Projects/rm27-m3c-schurvins-research/build-pc-new/installation.json",
  "reference_archive": "/home/shiuhou/Projects/archive/worktrees_20261016/rm27-vio-20261007_mixed_experiments/vicon_room1(1)/vicon_room1/V1_01_easy/V1_01_easy.zip",
  "output": "/home/shiuhou/Projects/rm27-m3c-schurvins-research/runs/v101-mono-half-new",
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

```bash
python3 -m rm27.perception.vision.localization.experiments.sensors validate --input /absolute/recording
python3 -m rm27.perception.vision.localization.experiments.sensors replay --input /absolute/recording --rate 1
python3 -m rm27.perception.vision.localization.experiments.prehardware \
  --lock-config /absolute/replay-config.json
python3 -m rm27.perception.vision.localization.experiments.prehardware \
  --matrix /absolute/matrix.json --output /absolute/new-acceptance --execute
```

The matrix calls existing estimator runners sequentially. It fixes configuration
and artifact identities, distinguishes 0.5x/1x and smoke/five-repeat groups,
preserves failures and refuses old-code receipts as current acceptance. Missing
required gates yield `PARTIAL`; a synthetic receipt cannot substitute for native
monocular/high-motion/recovery evidence. Inspect the formal integration record
for actual executed results, rather than inferring acceptance from these commands.

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

Front/down cameras must first be assessed independently. EuRoC ExactTime stereo
support does not prove non-overlapping-camera fusion or hardware synchronization.
UZH-FPV release/truth/calibration intake and labeled disturbances retain separate
input provenance. PC replay, cross-compilation and synthetic checks do not qualify RK3576
real-time behavior, a new camera/IMU chain, contact recovery or flight.
