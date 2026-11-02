# RM27 VIO research

## PDF-aligned current scope (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`. Current goal: repeated fixed-base underside contact,
separation, attitude/estimation recovery and target reacquisition in one sortie.
RK3576 is the new platform direction; its full hardware/data chain is not qualified.
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
OpenVINS remains the comparison baseline. SchurVINS (its included SVO2.0
frontend) is the first RK3576 candidate, with an independent experimental entry.
See [SchurVINS integration](.agents/docs/SCHURVINS_INTEGRATION.md) and
[reproduction tools](tools/schurvins/README.md). Legacy monocular SLAM launchers,
stella_vslam / ORB-SLAM3 experiments, VSL roadmaps and their tests are archived
on `legacy/slam` at `74d44b09c65e2a839b74d834756f0d178207d679`.
They are not supported by the main-branch runner.

## Start here

- [Current handoff](HANDOFF.md)
- [VIO scope and plan](new_plan.md)
- [Resume guide](rm27/perception/vision/docs/CODEX_RESUME.md)
- [Camera + IMU data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
- [Sensor evidence](rm27/perception/vision/docs/IMU_SOURCE_AUDIT.md)
- [PX4–M3C connection](rm27/perception/vision/docs/PX4_M3C_CONNECTION.md)
- [OpenVINS tools](tools/openvins/README.md)
- [Bench/offline evidence index](.maixpy/README.md)

VIO-P and VIO-S0 remain PARTIAL. Stage A/B validates transport and synthetic
ODOMETRY only; real VIO, Camera–IMU synchronization/calibration, M3C estimator
deployment and flight are not qualified. The latest physical logger diagnosis
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
