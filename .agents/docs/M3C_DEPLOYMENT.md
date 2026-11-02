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
This includes all seven compiled boundary tests. The latest rerun accompanies
the second return at `R/incoming/reviewed-sessions/m3c-20261102T080438Z/host-regression-junit.xml`.
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
