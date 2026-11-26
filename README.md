# RM27 VIO research

Current vision-team source: `main`.

```bash
git clone --branch main https://github.com/shiuhou/rm27_vio.git
```

Use a feature branch and open pull requests against `main`.
This branch includes recording/replay, native wrapper source, lock/patch and tests.
The combined Onboard checkout is `shiuhou/rm27_drones` on `main`.
The `codex/vision-team-review` branch remains the original review checkpoint.
Main merges that delivery with the earlier historical research history;
M3C procedures and Schur headless sources retained from that history are
reference material, and SchurVINS remains `USER_PAUSED`. The published source
does not qualify live sensors, native RK3576 execution or valid flight outputs.

## PDF-aligned current scope (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`.
The current research goal is for a UAV to identify a fixed target armor plate
through vision, then make controlled contact with a designated underside pad at a
prescribed attitude relative to the plate. The mission includes separation,
recovery of stable flight and state estimation, and visual reacquisition for
repeated contact in one sortie.
As of 2026-11-11, RK3576 is the sole active RM27 onboard platform.
M3C is retired from development and deployment; its source, data, and validation
records remain historical references. Platform acceptance requires RK3576 evidence.
M3C, MaixCAM2/MaixCDK and AXMODEL platform content, including later author
updates, is historical material (2026-11-23), not a current development or
waiting dependency. Original evidence and source identities remain intact.
Return-to-dock and precision landing below belong to historical baselines, not the
new season's mandatory deliverables. Traditional/RL allocation remains a research
choice. Detailed algorithms/hardware tasks are scheduled separately by the user.

Component responsibility: Continuous camera–IMU state estimation, timing and calibration. OpenVINS remains the current implementation baseline; new RK3576 data qualification is pending. Historical MicoAir logging failures remain evidence, not a permanent dependency of a new independent IMU.

The authorized cross-repo interface batch is synthetic/offline only, with no
learning, firmware execution, hardware operations or Git publication. The root
repository coordinates the single activity plan and derived report. Component
contracts remain self-contained; the shared engineering method is unchanged.

Integration source when in the parent checkout: [../../docs/INTEGRATION.md](../../docs/INTEGRATION.md). The local `interface_worker.py` is a JSONL test endpoint; it does not qualify the underlying estimator/controller.

## Local cross-process interface contract

Run `python -u -m interface_worker` from this repository's root using its isolated
Python environment. Input/output is one JSON object per line; stderr is diagnostic.
Envelope: `version: 1`, `source: coordinator`, increasing integer `seq`,
`input_kind: synthetic`, `kind`, and object `payload`. `kind: hello`, payload `{}`
returns actual loaded source paths and dependency versions. Replies echo kind/seq,
identify this component, and contain `ok` with payload or typed error. Other input
provenance labels are intentionally refused by this synthetic acceptance endpoint.
The application schema below is owned by this repo; JSONL is not the final vehicle bus.

`kind: localization` accepts the native schema v2 `LocalizationEstimate` dictionary
as payload and returns its native `from_dict(...).to_dict()` representation.
Source/publish time and clock evidence, parent/child frames, metric or unknown
scale, velocity/covariance availability, validity and epoch are preserved. This
worker neither runs OpenVINS nor fills missing sensor evidence. Python stdlib only.
Native contract: `rm27/perception/vision/localization/schema.py`.


`main` is exclusively for Camera + IMU visual-inertial odometry research.
OpenVINS is the current mono + IMU baseline. SchurVINS (its included SVO2.0
frontend) is paused; its independent experimental entry and failures are retained.
See [SchurVINS integration](.agents/docs/SCHURVINS_INTEGRATION.md) and
[reproduction tools](tools/schurvins/README.md). Legacy monocular SLAM launchers,
stella_vslam / ORB-SLAM3 experiments, VSL roadmaps and their tests are archived
on `legacy/slam` at `74d44b09c65e2a839b74d834756f0d178207d679`.
They are not supported by the main-branch runner.

## Review the current source with teammates

Read this README, the [requirements](.agents/docs/SPEC.md) and
[current milestones](.agents/docs/ROADMAP.md) first. The active target is RK3576;
M3C / MaixCAM2 / AXMODEL material preserves history. SchurVINS is `USER_PAUSED`.
The owning integration checkout supplies `onboard/`, deployment settings and
the OS04A10 preparation note; those are separate from this estimator component.

| Reading entry | Responsibility |
|---|---|
| [Recording/replay tools](tools/openvins/README.md#sealed-camera--imu-recording--offline-diagnostics) | Validated Camera + IMU recording to native inputs and offline diagnostics |
| [Sensor recording](rm27/perception/vision/localization/experiments/sensors.py) and [data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md) | Raw samples, clocks, calibration, provenance and sealed recording |
| [State output](rm27/perception/vision/localization/state_output.py) and [schema](rm27/perception/vision/localization/schema.py) | Frame/time/validity contract consumed by integration |
| [Recording tests](tests/test_openvins_recording.py) and [runner tests](tests/test_openvins_recording_replay.py) | Input rejection, exact readback and mock-process failure/cleanup behavior |

For a source review, retain `rm27/`, `tools/`, `tests/` (including fixtures),
`interface_worker.py`, `requirements.txt`, README/AGENTS and `.agents/docs/`.
Keep source documents and notices with their code. Some OpenVINS tests reuse
sensor transport helpers in `tools/schurvins/` and sibling test fixtures;
including them does not resume that paused estimator. Main includes
`recording.py`, `recording_replay.py`, their tests and the native wrapper source;
these files are available from its committed tree.

Do not copy `.git`, environments, caches, `.agents/plans/`, `.agents/runs/`,
`artifacts/`, raw datasets, or the local `data/shared_*` links into the source
review. These are not required for the following offline checks. Raw input,
native build and hardware dependencies are listed separately in the
[OpenVINS dependency boundary](tools/openvins/README.md#source-review-and-native-runtime-dependencies).
A source-only snapshot supports review and tests; it does not provide a native
estimator installation or qualified RK3576 Camera + IMU data.

From a separate Linux source copy with Python 3.10 or newer:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  tests/test_openvins_recording.py tests/test_openvins_recording_replay.py \
  tests/test_sensor_inputs.py tests/test_openvins_runtime.py \
  tests/test_openvins_projection.py tests/test_openvins_faults.py -q
```

These tests generate small labeled fixtures and mock child processes; they run
without a camera, IMU, flight controller, Docker, public dataset or OpenVINS
binary. ROS bag-specific cases skip when `rosbag2_py` is absent. The requirements
file declares direct Python dependencies; it is not a frozen environment lock.

## Replay a sealed Camera + IMU recording

The [offline recording entry](tools/openvins/README.md#sealed-camera--imu-recording--offline-diagnostics)
validates shared sensor recordings, binds explicit calibration/clock/IMU
parameters and runs the existing pinned ROS-free OpenVINS runner. It generates
input lineage, native trajectory and diagnostics without a ground-truth
requirement. This is a local recorded-input path; actual RK3576 sensors,
synchronization, accuracy, live operation and flight remain unqualified.
An arbitrary MP4 and flight attitude log are not sufficient inputs.

## Prepare a monocular comparison

**SchurVINS is paused by the user as of November 24, 2026.** OpenVINS remains
the current mono+IMU baseline. The preparation and Schur results below preserve
prior research; a possible future downward-facing camera has not been specified
or qualified and does not restart this work.

Online [upstream support verification](.agents/docs/SCHURVINS_INTEGRATION.md#upstream-support-scope-2026-11-24)
confirms the original SchurVINS implementation supports stereo only. Its
collaborator explicitly states that mono requires code adaptation. The user
has instructed us to stop making that adaptation. The mono preparation and
results below preserve earlier RM27 research; they do not establish official
SchurVINS mono support. A separate third-party OpenVINS reimplementation exists
but has not been adopted or verified here.

For RK3576 testing, the [native diagnostic preparation](tools/schurvins/README.md#rk3576-native-diagnostic-candidate)
packages the frozen ARM64 Schur userspace, full installation provenance and
sensor-only input. Its independent board entry checks native architecture and
resources before a bounded raw replay. The candidate is `PREPARED_NOT_RUN`;
native RK3576 startup and physical Camera+IMU remain unverified. The PC mono
0/1-pose failures are preserved, and OpenVINS remains the baseline.

The [mono VIO comparison protocol](rm27/perception/vision/docs/MONO_VIO_COMPARISON.md)
defines matching cam0 + IMU inputs, playback rates, full coverage, fixed-scale
accuracy and separate runtime/hardware evidence. `python -m tools.prepare_mono_comparison`
checks existing hash-locked EuRoC gate pairs and writes future smoke/repeat configs
without running either estimator. Its result is `PREPARED_NOT_RUN`; installed
runtime, native ingestion and RK3576 qualification remain unverified.
The OpenVINS sealed recording/calibration/clock entry is now available above.
The shared generic comparison lock path remains incomplete and paused with
SchurVINS. See the protocol for historical preparation commands.
The first authorized PC replay is recorded in that protocol: Schur's frozen mono
build produced no evaluable trajectory; OpenVINS completed two independent
environment checks after loading host ROS. The four original smokes remain failed,
their 20 repeats remain unrun, and no RK3576 or repeatability qualification follows.
The [state-to-pose diagnosis](.agents/docs/SCHURVINS_INTEGRATION.md#monocular-state-to-pose-diagnosis-2026-11-23)
traces those 0/1 poses to native Tracking/quality gates: accepted corrections
alone do not meet the 40-current-inlier quality requirement. The separate summary
phase-association bug is fixed; the original estimator failures remain.
The [bounded repair attempt](.agents/docs/SCHURVINS_INTEGRATION.md#bounded-monocular-repair-attempt-2026-11-24)
then tested two isolated update candidates on full 0.5× inputs; both still emitted
one pose. Neither patch was retained in production. Initialization-to-filter
uncertainty and map consistency remain the next design issue; mono and RK3576
qualification are still incomplete.

## Start here

- [Current requirements](.agents/docs/SPEC.md), [design](.agents/docs/DESIGN.md) and [milestones](.agents/docs/ROADMAP.md)
- [Camera + IMU data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
- [Sensor evidence](rm27/perception/vision/docs/IMU_SOURCE_AUDIT.md)
- [Historical PX4–M3C connection evidence](rm27/perception/vision/docs/PX4_M3C_CONNECTION.md)
- [OpenVINS tools](tools/openvins/README.md)
- [Historical handoff](HANDOFF.md), [plan](new_plan.md), [resume guide](rm27/perception/vision/docs/CODEX_RESUME.md) and [bench evidence index](.maixpy/README.md)

VIO-P and VIO-S0 remain PARTIAL. Stage A/B validates transport and synthetic
ODOMETRY only; real VIO, Camera–IMU synchronization/calibration and flight
are not qualified. M3C estimator deployment remains historical and is not an
active work item. The latest physical logger diagnosis
does not constitute a loss fix. No acceptance gate was relaxed by this cleanup.

## Retained shared infrastructure

Camera capture/calibration and image-evidence preparation, timestamp/pose
contracts, rigid transforms and evaluation primitives remain because VIO uses
them. The Python package stays `rm27.perception.vision.localization`.
`experiments.runner` accepts only `backend=OpenVINS`; unknown or legacy
backends are refused before output creation. No alternate SLAM launcher remains.

With an existing compatible Python/NumPy/OpenCV/pytest environment:

```powershell
py -3 -m pytest tests -q --ignore=tests/camera_capture.py
```

The excluded file is an unrelated, untracked Maix hardware program, not a test.
ROS2-specific integration needs its existing sourced environment. Offline tests
do not open a device or authorize a dated capture script.

Raw recordings, public datasets, third-party binaries/builds, environments,
credentials and most dated diagnostic evidence remain outside Git. Published
historical manifests describe their original artifacts, not this refactored
tree. A fresh clone cannot claim a saved-data or hardware replay without them.
Repository cleanup does not update the main drone repository or the Vault.


## Shared real-camera test videos

[Shared video catalog](data/shared_vision/README.md) provides four user-supplied
real drone-camera MP4 fixtures, hashes, reported capture settings, decoding checks
and replay instructions. `data/shared_vision/` is a relative link to the shared
RM27 dataset, with one physical copy; videos are local data, not included in a
standalone clone. Keep derived test outputs separate from these inputs. See the
catalog for exposure-mapping and synchronized-IMU/timestamp limitations.

## Shared M3C and MaixCAM2 board references

[Shared board catalog](data/shared_m3c_board/README.md) provides the user-supplied
M3C/MaixCAM2 schematics and 3D/mechanical files, with a SHA-256 manifest.
Use `data/shared_m3c_board/raw/` from this checkout; all local aliases share one
physical collection. Match the document to the actual board revision. Raw files
are local references supplied separately from Git clones; see the catalog for paths.

## Shared RM27 aircraft model

The current shared simulation aircraft is **ASM2 v1.0.0 (142 g)**.
For new MuJoCo work, use the **v0.1.0 collision proxy** from the
[shared aircraft catalog](data/shared_aircraft/README.md).
Existing experiments retain their pinned model and configuration.
