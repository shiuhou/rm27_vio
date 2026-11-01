# RM27 VIO research

`main` is exclusively for Camera + IMU visual-inertial odometry research.
The current estimator line is OpenVINS. Legacy monocular SLAM launchers,
stella_vslam / ORB-SLAM3 experiments, VSL roadmaps and their tests are archived
on `legacy/slam` at `74d44b09c65e2a839b74d834756f0d178207d679`.
They are not supported by the main-branch runner.

This worktree is the independent `research/m3c-vio` deployment branch. The
[SchurVINS integration record](.agents/docs/SCHURVINS_INTEGRATION.md) contains
its measured PC/ARM64 results and separate M3C acceptance. The
[ROS-free C++ runtime](tools/schurvins/headless/README.md), read-only M3C audit
and native benchmark are reusable on other Linux ARM64 targets. Original RK3576
work remains in its separate worktree. Publication is authorized only to
`origin/research/m3c-vio`; no merge or PR is included.

## Start here

- [M3C laptop 測試與結果回傳操作手冊](.agents/docs/M3C_LAPTOP_HANDOFF.md)
- [Current handoff](HANDOFF.md)
- [VIO scope and plan](new_plan.md)
- [Resume guide](rm27/perception/vision/docs/CODEX_RESUME.md)
- [Camera + IMU data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
- [Sensor evidence](rm27/perception/vision/docs/IMU_SOURCE_AUDIT.md)
- [PX4–M3C connection](rm27/perception/vision/docs/PX4_M3C_CONNECTION.md)
- [OpenVINS tools](tools/openvins/README.md)
- [M3C SchurVINS deployment evidence](.agents/docs/M3C_DEPLOYMENT.md)
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
