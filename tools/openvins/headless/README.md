# ROS-free OpenVINS EuRoC runtime

This directory is the canonical RM27 VIO owner of the ROS-free runner source
for the current RK3576 preparation. On 2026-11-26 the six existing wrapper files
were copied from the historical linked worktree after all six hashes matched
the actually-used PC build receipt. The C++ files, CMake and preparation driver
retain those bytes; this README adds the handover instructions. No M3C hardware
development is resumed, and native RK3576 performance remains unqualified.

[upstream.lock.json](upstream.lock.json) records the upstream URL, immutable
commit, original archive hash, all 202 original/prepared file hashes, imported
wrapper hashes and reference PC library/compiler versions. OpenVINS upstream
is GNU GPL version 3; [OpenVINS-LICENSE](OpenVINS-LICENSE) is copied unchanged
from the pinned archive. Existing upstream copyright headers are retained by
source preparation and in [the exact patch](accepted-updates-mono.patch).
The imported RM27 wrapper files had no separate license notice; this import
does not assign a new license to them.

This runner calls the pinned upstream `VioManager` with real decoded camera
pixels and raw SI IMU samples. Static/dynamic initialization, feature gating,
filter state and covariance remain upstream. It does not load ground truth,
initialize from known state, use ROS, or provide a flight-valid output adapter.

Obtain the separately supplied upstream archive from
`https://api.github.com/repos/rpng/open_vins/tarball/69488123ed9362dd44b6f28e7f4680abbff1442b`.
The expected SHA-256 is
`283dde4096d9380956f6a9c8ed7e54e2d1303ee64af002d9a97f9b2196e677f4`.
The preparation driver rejects a different archive. Extract it into a new
directory and set `--source` to the extracted repository root containing
`ov_core/`, `ov_init/`, `ov_msckf/` and `config/`. Use explicit absolute paths
for the archive, extracted source, new prepared source and new build directory:

```bash
python3 tools/openvins/headless/build.py --source PINNED_SOURCE \
  --archive PINNED_ARCHIVE --output NEW_SOURCE
cmake -S tools/openvins/headless -B NEW_BUILD \
  -DOPENVINS_SOURCE=NEW_SOURCE -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17
cmake --build NEW_BUILD -j2
```

`build.py` prepares source only. CMake compilation and estimator execution are
separate steps; neither is performed by the source-only Python test command.
The patch is the exact three-file transformation already implemented by
`build.py`; do not apply it a second time to the driver's output. For review,
applying it once to the unmodified pinned files must produce the hashes in
`prepared_files_sha256`.

Required libraries: Eigen, OpenCV core/imgproc/imgcodecs/video/features2d/calib3d,
Boost system/filesystem/thread/date_time, Ceres and pthreads. ArUco, visualization,
ROS, matplotlib and simulation executables are excluded. Ceres dynamic-init code
is linked even with the unchanged EuRoC static-init profile. The native and
cross builds consume the identical prepared source; cross Ceres 1.13 absolute
dependency entries are explicitly relocated into the selected target sysroot.
The reference PC build used Eigen 3.4.0, OpenCV 4.6.0, Ceres 2.2.0, Boost 1.83.0,
GCC 13.3.0 and CMake 3.28.3 with C++17. The lock records this reference; it does
not install or freeze a fresh host's dependencies. RK3576 native dependency
resolution and a fresh compilation are `NOT_RUN` in this source handover.

For an independently rebuilt runner, create a new build receipt recording the
actual source/preparation hashes, wrapper hashes, binary SHA-256, architecture,
compiler/dependencies and immutable image identity if Docker is used. Do not
reuse the historical build receipt for a new binary. The sealed-recording
[runtime descriptor](../README.md#sealed-camera--imu-recording--offline-diagnostics)
binds that receipt to the executable before replay. Generated sensor/calibration
inputs from `tools.openvins.recording` are required for real recordings; the
upstream EuRoC YAML in prepared source is not a real-camera calibration.

Preparation verifies the pinned archive SHA-256 and every copied source file
against that archive's regular-file contents before creating output.
`preparation.json` records every original/prepared file hash and the three
explicit changes: mono configuration, and observation-only counters immediately
after the MSCKF and SLAM updater's gated `EKFUpdate` returns. The counters do not
change estimator arithmetic. Mono configuration retains upstream calibration,
noise, online calibration, features, gates and four OpenCV threads; subscriber
and publisher threading are explicitly synchronous for offline transport.

`frames.csv` preserves original camera integer nanoseconds and reports native
initialization, exact state/camera-time match, finite state/covariance checks,
actual visual correction counts/measurement rows/features and timing. An emitted
pose may be propagation-only: `visual_update=0` explicitly distinguishes this.
No generic EKF/ZUPT call, input callback or pose count is treated as accepted
visual-inertial correction. Delayed landmark initialization is not counted as
an ordinary accepted SLAM correction by these counters.

`poses.csv` position is `p_IinG` in meters. Native JPL `q_GtoI` components reordered
to wxyz numerically equal Hamilton `q_ItoG`; do not conjugate them. Original
camera nanoseconds and estimated camera-to-IMU offset are preserved separately.
`timestamp_imu_seconds` follows the native published-state time convention
`camera_seconds + estimated_offset`; it retains upstream double precision.
The evaluator must choose and state the appropriate clock explicitly.

The transport waits for one real IMU sample strictly after each current
camera/IMU endpoint, matching upstream callback readiness, and never pads input.
All original IMU samples are delivered exactly once on full replay. The final
post-camera IMU tail is delivered without inventing additional image updates.

`--rate 0` is unpaced, `0.5` and `1` use the camera timeline. Pending-frame counts
measure scheduled replay lateness, not hardware queues. Callback steady timing
includes image decoding and IMU admission; native stage timing uses the existing
upstream Boost wall-clock timers. Resource/latency results are diagnostics until
the corresponding trajectory and accepted-update gates pass.
Paced offline replay makes the required future IMU sample available immediately
after camera scheduling; it is not live sensor arrival or end-to-end latency.

Existing output directories are refused. Exceptions return nonzero and preserve
partial output; `EXECUTION_COMPLETED` reports transport completion only. Full
trajectory coverage, update continuity, initialization, accuracy, repeatability
and target execution must be assessed separately. QEMU cannot establish RK3576
performance or live sensor qualification.

The source-only suite in the component README exercises adapters and mock
processes. It does not execute this C++ estimator or establish native accuracy,
timing or hardware behavior. Historical native behavioral tests used the
separate `OPENVINS_HEADLESS_TEST_COMMAND`, `OPENVINS_HEADLESS_TEST_DATASET` and
`OPENVINS_HEADLESS_TEST_CONFIG` environment; those tests/results are not part of
this source-only validation.
