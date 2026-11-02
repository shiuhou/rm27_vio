# Pinned native OpenVINS preparation

## Runtime adapter continuation

The experiment runner now accepts the separate OpenVINS VIO config through
`python3 -m rm27.perception.vision.localization.experiments.runner --config ...`.
See `../../rm27/perception/vision/docs/OPENVINS_ADAPTER.md` for reproduction,
frozen installation/input manifests, online partial-v2 semantics and evidence.
PC ROS2 dataset runtime and same-run conversion/SE3 equality are verified, but VIO-P
remains PARTIAL due to historical-run ATE consistency. Do not infer a PASS from
successful process exit alone. Existing output directories are refused.

## Independent SchurVINS comparison

`python3 -m rm27.perception.vision.localization.experiments.openvins_comparison --config CONFIG.json`
supports explicit EuRoC mono/stereo experiments without changing the preserved
V1_01 mono runner or its 1 cm historical repeatability gate. The comparison
purpose is `EUROC_PUBLIC_COMPARISON`; it is not VIO-P qualification. Config v1
requires `run_id`, `backend=OpenVINS`, `archive`, `archive_sha256`, `sequence`,
`mode` (`mono`/`stereo`), `rate` (`0.5`/`1.0`), verified `bag`, frozen
`installation` and a fresh `output` directory.

Prepare inputs with `python3 -m tools.openvins.comparison_transport INPUT.zip NEW_BAG
--sequence SEQUENCE --mode stereo` in a sourced Jazzy environment. Every decoded
left/right image byte, integer timestamp and IMU SI value is read back against
the source. Stereo source timestamps must match exactly. The runtime checks the
actual bag topic table, records both camera observer streams and rejects truth
topics, altered artifacts, existing outputs and source/config drift. Ground
truth is read on the host only after container shutdown. Both visual and IMU
propagated outputs retain unknown tracking/reset/publication evidence and false
operational validity. A pose count does not prove estimator IMU ingestion.

The existing native binary, EuRoC calibration and explicit ROS2 lifecycle patch
are reused under 4 CPU/12 GiB limits. New wrapper/observer hashes are recorded
separately; the frozen historical files remain unchanged. `evaluation.json`
uses the same scale=1 SE(3), original ground truth and per-epoch scoring as
SchurVINS, checks exact same-run raw/adapter metric parity and preserves full
coverage. Native timing, GNU-time process-tree CPU/RSS and sampled cgroup CPU/
memory have distinct scopes. Observer receipt is not processing completion.
Run native experiments sequentially on the same host, and keep 0.5x and 1x
groups separate. PC replay does not establish RK3576 real-time VIO or flight.

The current comparison protocol explicitly offers RELIABLE/VOLATILE camera/IMU
publishers using `comparison_qos.yaml` and uses reliable observer subscriptions.
The player starts paused; each observer input must have a compatible publisher
match and actual publisher/subscriber endpoints are saved before the Resume
service releases samples. The original native installation uses SensorDataQoS,
including BEST_EFFORT IMU; separate native observation derivatives declare their
own pinned QoS and lifecycle changes in the installation manifest. Saved actual
publisher/subscriber endpoints describe each run. Observer readiness alone does
not establish complete native ingestion. The two earlier best-effort-observer
stereo smokes failed IMU completeness and remain failed evidence. Revised
protocol runs use distinct IDs/cohorts and require a new full smoke.

Qualified installations may additionally declare
`input_publisher_match_observation`. The existing comparison entry validates
its helper/closure hashes and enables `RM27_OPENVINS_INPUT_MATCHES` for that
build. Before resuming the paused player, the same strict decoder used after
shutdown requires a bounded atomic native snapshot, no sibling `.pending`, the
actual container child PID, selected mode and all required camera/IMU matches.
The raw snapshot and PID record are sealed with the run. These sequential
reader-match observations do not certify continued matching, complete sample
delivery or tracking; full source/native reconciliation remains mandatory.
Installations without this capability retain their existing protocol.

An independently locked observation build may additionally declare
`native_health_observation`. `native-health.patch` adds compact witnesses at
the actual `fast_state_propagate` success/failure returns; it applies to the
qualified observation derivative, not bare upstream. It changes no estimator
math. Capability schema 2 records the function-local cache time and offset,
so the integration endpoints can be checked without borrowing an asynchronous
main-state offset. The requested IMU callback double and actual integration
endpoints remain separate; the main-filter timestamp is an asynchronous sample,
not a coherent prediction-state snapshot. The existing 200,000-event/100-MiB
limits still apply. A complete
footer with zero dropped events is mandatory; large/stereo runs require their
own budget qualification.

The comparison config can explicitly opt into a separate freshness timeline:

```json
"state_health": {
  "schema_version": 1,
  "policy": {
    "max_visual_age_ns": 300000000,
    "max_imu_gap_ns": 10000000,
    "max_receive_age_ns": 300000000,
    "max_visual_receive_age_ns": 600000000,
    "max_buffer_samples": 4096
  }
}
```

These are PC test deadlines, not flight limits. `native-health.json` and
`online/native-health.jsonl` distinguish initialization commit/ready, accepted
visual corrections, camera-driven IMU propagation, fast prediction and
independent watchdog ticks. Only strictly newer accepted measurement support
refreshes last visual time. Watchdog ticks refresh no sensor/state evidence.
Freshness uses one native event-serialization clock plus the explicit dataset
sensor clock; neither is a hardware receive/publication timestamp or a clock
mapping. `max_buffer_samples` is unused by this native projection, which does
not perform an independent IMU integration.

`CURRENT` qualifies only these evidence deadlines. Operational health remains
`UNKNOWN`, validity false and native reset/epoch unknown. The adapter session's
epoch zero is not a native reset count. The fixed OpenVINS version has no runtime
reset API; stopping or restarting it does not pass reset qualification. Existing
partial v2 trajectories, scores, legacy runner and repeatability gate are
unchanged. Omit `state_health` to preserve the comparison's existing behavior.

For installations declaring camera-consumption observations,
`native_observations.py` checks original integer stamps, pixels and occurrence
lineage through subscription, decoding, frontend return and shutdown terminals.
The optional `terminal_drain` summary validates executor quiescence, an ordered
completed-IMU watermark, drain entry/return and initializer join. It does not
certify that future/unready queued images were fed or that all IMU samples were
integrated. The optional `state_export` summary records context flags and text
byte counts; ROS delivery remains `NOT_RUN` until the recorded bag and observer
are independently reconciled. Legacy ledgers retain their original summary
shape. These observations never upgrade UNKNOWN/invalid health or reset status.
The comparison also requests `camera_transactions` from the native decoder:
actual processing boundaries, the last accepted full state/velocity and merged
accepted measurement support. Counts describe observation uses, not independent
features. Old support, nonvisual updates and IMU propagation never refresh the
last visual measurement time. Epoch/reset, covariance, publication timestamps
and clock mappings stay unknown. This metadata does not alter the trajectory
or fixed-scale scores.
Candidate versions and replay results are in
`../../.agents/docs/SCHURVINS_INTEGRATION.md`; keep their source identities separate.

Artificial comparison uses the same entry with
`purpose=EUROC_ARTIFICIAL_COMPARISON`. Replace the ambiguous `archive` fields
with explicit `original_reference_archive` / `original_reference_archive_sha256`
and `derived_sensor_archive` / `derived_sensor_archive_sha256`, and supply the
existing `prepared_input` manifest. Only the derived sensor bag enters the
estimator container. Reference truth, source hashes, original camera IDs and
the full source denominator remain outside it. Artificial and original results
carry distinct provenance; neither native publication nor propagation proves
recovery or valid tracking.

```bash
python3 -m tools.openvins.comparison_transport /absolute/prepared/sensors.zip /absolute/new-bag \
  --sequence V1_01_easy --mode stereo --prepared-input /absolute/prepared \
  --original-reference-archive /absolute/V1_01_easy.zip --rate 0.5
# Recheck every byte/time without creating another large bag:
python3 -m tools.openvins.comparison_transport /absolute/prepared/sensors.zip /absolute/new-bag \
  --sequence V1_01_easy --mode stereo --prepared-input /absolute/prepared \
  --original-reference-archive /absolute/V1_01_easy.zip --rate 0.5 --verify-only
```

Artificial storage timestamps derive from the common `scheduled_events`
delivery deadlines and the locked playback rate; sensor header timestamps are
unchanged. Delay stays a wall-time delivery delay at either supported rate.
The manifest binds that rate, schedule, calibration and source hashes. Runtime
observer reconciliation checks declared delivery order without sorting receipts.
Unknown reset/watchdog/visual-correction evidence stays `NOT_RUN`, so numerical
fault replay alone cannot close the pre-hardware recovery gate. One V1_01
stereo 500 ms camera-gap run completed at 0.5× with all processes exiting zero,
2,790 visual poses from the original 2,912-frame denominator and fixed-scale
ATE RMSE 0.0566455682 m. Same-run raw/adapter scores agree; operational tracking
and recovery remain `UNKNOWN` / `NOT_RUN`. Its immutable evidence is under the
external research directory `prehardware_openvins_fault_gap500ms_v1/`.
Additional fault bags should be
prepared individually while retaining the 20 GiB space reserve.

## Current ROS2 path

See `../../rm27/perception/vision/docs/ROS2_LIFECYCLE_FIX.md` for current
lifecycle root cause, explicit patch, tests and clean full-sequence result.
`ROS2_NATIVE_PROGRESS.md` preserves earlier data/build/shutdown failure evidence.
The ASL input is now available; the historical download blocker below is resolved.

`asl_transport.py INPUT.zip NEW_OUTPUT_DIRECTORY` runs in a sourced Jazzy
environment. It writes only cam0 + imu0, preserving integer header/storage
timestamps and all decoded pixels / IMU values, then reads the entire bag back
against the source. Existing output directories are rejected; failed output
must not be reused or treated as verified without `source_verification.json`.
Synthetic tests are in `tests/test_asl_transport.py`; real-data counts and hashes
are in the progress report. Ground truth is never included in estimator input.

`Dockerfile.ros2` uses the immutable imported official Jazzy parent image.
`build_native_ros2.sh` expects read-only `/upstream` plus writable `/ws`.
`run_native_ros2.sh` expects read-only `/upstream`, `/ws`, `/data`, fresh writable
`/out`, and isolated container networking. It records stock native node output
at half-rate playback, checks subscriber readiness and signals clean shutdown.
It is a feasibility runner, not a proof of complete IMU callback ingestion.

Pristine pinned source failed under Jazzy due to renamed ROS headers.
`jazzy-headers.patch` records the separate include-only compatibility copy;
never call a compatibility-build run pristine-upstream success. No VIO
algorithm edits are permitted merely to change ROS transport.

The header-only native process also had a confirmed global/static teardown
ordering defect. Apply `ros2-lifecycle.patch` to that source variant; it changes
only ROS2 entry-point ownership from global to main-scoped. Preserve the old
source/build/run. The tested separate copy is `upstream/openvins-jazzy-lifecycle`
with `native-ros2-lifecycle` build, under the existing external evidence root.
Use the designation **pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**.
Do not call it unmodified upstream or imply adapter/accuracy qualification.

`lifecycle_smoke.py` sends zero sensor samples, checks subscription readiness,
then tests clean SIGINT shutdown with publisher thread on and off. Run it in
the ordinary isolated research image after sourcing `/ws/install/setup.bash`,
with `--count 5 --out /evidence/NEW_DIRECTORY`. It refuses existing outputs.
`Dockerfile.gdb` adds only diagnostic packages. `debug/*.gdb` capture the
original crashing thread/all-thread stacks and before/after destructor order.

## Historical ROS1 preparation (not the current path)

**Current preference is native ROS2, not ROS1.** See
`../../rm27/perception/vision/docs/ROS2_DATASET_REVIEW.md`. The ROS1 recipe and
commands below are historical fallback documentation, not the selected next
execution path. Do not convert data just to use them. External evidence moved
to `/home/shiuhou/Projects/rm27-vio-20261007`; old absolute paths below are historical.

Research-only container recipe; runtime is unqualified until the native gate runs.

## Frozen inputs

- Upstream: https://github.com/rpng/open_vins
- Commit: `69488123ed9362dd44b6f28e7f4680abbff1442b`
- Official archive API:
  `https://api.github.com/repos/rpng/open_vins/tarball/69488123ed9362dd44b6f28e7f4680abbff1442b`
- Retrieved archive SHA256:
  `283dde4096d9380956f6a9c8ed7e54e2d1303ee64af002d9a97f9b2196e677f4`
- Base: `ros:noetic-ros-base-focal` digest
  `sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`
- Data: EuRoC `V1_01_easy.bag`, official URL:
  `http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag`

Git clone failed twice; source is an extracted official pinned archive, not a
successful Git checkout. No source patches applied. The archive digest records
retrieved bytes, not a promise that future tarball compression stays identical.

## Resume commands (not evidence of successful build/run)

Run on the Linux host. External evidence directory:
`/home/shiuhou/rm27-vio-20261007`. Use new output directories for retries.
Never mount the main project or flight-controller devices into the container.

```bash
docker build -t rm27-openvins:noetic-20261007 -f tools/openvins/Dockerfile tools/openvins
docker image inspect rm27-openvins:noetic-20261007
```

After successful image build, record its immutable image ID and dependency list
(`dpkg-query -W`). Use that image ID rather than a mutable tag for comparisons.
Create `catkin-native` as the host user before running the following command.
Mount upstream read-only and link it into a new external catkin workspace:

```bash
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v /home/shiuhou/rm27-vio-20261007/upstream/rpng-open_vins-6948812:/upstream:ro \
  -v /home/shiuhou/rm27-vio-20261007/catkin-native:/ws \
  rm27-openvins:noetic-20261007 bash -c \
  'mkdir -p /ws/src && ln -s /upstream /ws/src/open_vins && cd /ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DDISABLE_MATPLOTLIB=ON -j4'
```

Record compiler/OpenCV/Eigen/Ceres versions, CMake cache, build log and hashes of
the native executable/libraries. Build exit code alone is insufficient.

For the native run use an isolated container/ROS master, built workspace,
read-only bag/config/upstream mounts and a new writable output directory. Source
`/ws/devel/setup.bash` inside the container and invoke the stock launch file:

```bash
roslaunch ov_msckf serial.launch config:=euroc_mav \
  max_cameras:=1 use_stereo:=false bag_start:=0 \
  bag:=/data/V1_01_easy.bag dolivetraj:=false \
  dosave:=true path_est:=/out/native_pose.txt \
  dotime:=true path_time:=/out/native_timing.txt
```

Do not set the estimator node's `path_gt`: native code can initialize from it.
Hash the bag and included calibration files; inspect topic counts, header and
bag timestamps, units and usable IMU coverage. Preserve raw outputs and exit /
timeout evidence even on failure. Capture `/usr/bin/time -v` resource output
and name its process-tree scope; Docker client RSS is not estimator RSS.

Native pose export is a saved online stream, not an optimized final trajectory.
Only after native success should the RM27 adapter be implemented and compared
against these exact inputs and configuration.

## Evaluation caveat

Pinned upstream docs warn that original V1_01_easy orientation truth is
inaccurate. Choose and hash the reference explicitly and report that limitation
alongside rotational RPE. Do not silently substitute corrected truth or feed
either truth into estimation.
