# M3C SchurVINS deployment evidence

For the laptop physically connected to M3C, follow the
[laptop test and return runbook](M3C_LAPTOP_HANDOFF.md). It covers the separate
checked runtime/data transfer, board identity and ABI checks, mono diagnostics,
and raw-result handoff to this development host. Git pull alone does not include
the binary or public dataset. No laptop/board results have been received yet.

This is the independent M3C branch result dated 2026-10-31. The
[shared integration record](SCHURVINS_INTEGRATION.md) describes the imported
SchurVINS research baseline; it is not evidence of execution on M3C.
OpenVINS and the RK3576 worktree remain separate and unchanged by this task.

## Qualification

| Boundary | Result | Evidence / limitation |
|---|---|---|
| Independent PC ROS build | PASS | All 26 packages built from the pinned dependency closure |
| PC ROS-free C++ build | PASS | Real estimator/SVO sources, no ROS dynamic libraries |
| ARM64 ROS-free C++ reference build | PASS | AArch64 ELF, GNU 7.5/Bionic reference sysroot; target ABI compatibility unknown |
| ARM64 execution under QEMU | PASS stereo numerical replay / FAIL mono | Checked runtime: full 2912 images / 29120 IMU; stereo 2911 poses, zero loss; mono one pose and loss |
| Full PC mono V1_01 / V1_03, ROS runner | FAIL | Trajectory not evaluable; process exit zero is insufficient |
| Full PC mono V1_01, ROS-free runtime | FAIL | 2912 cameras, 29120 IMU samples admitted, one pose and one tracking loss |
| Full PC stereo V1_01 at 1× | PASS for numerical replay | 2911 poses, one explicit input-prefix rejection, no observed tracking exit |
| Actual M3C identity / OS / SDK audit | BLOCKED | SSH closed before authentication; user requested continuation without board |
| M3C offline / 0.5× / 1× replay | BLOCKED | No actual board execution evidence |
| Real OS04A10 + IMU VIO | NOT_RUN | Device access, timing and qualified input chain unavailable |
| Physical PX4 UART | NOT_RUN | Safety/configuration confirmations required |
| 60-minute qualified VIO endurance | NOT_RUN | Requires a valid mono baseline and actual target access |

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
existing reference research without changing them. M3C SoC, RAM, Linux, libc,
loader, SDK and device owners remain UNKNOWN until a fresh board audit succeeds.

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

The complete local suite passed **583 tests**, using the existing compatible
OpenCV 5 environment and sourced ROS2, with the checked C++ binary supplied
through `SCHURVINS_HEADLESS_TEST_COMMAND`; no C++ boundary tests were skipped.
The seven compiled boundary tests were rerun after the final test-portability
edit. An earlier system-Python run had 520 passes and one pre-existing ChArUco
API failure; its evidence remains in `R/baseline-software.json`. No test was
removed or weakened. Final results are in `R/final-software-junit.xml` and
`R/headless/final-cpp-boundary-junit.xml`.

The evaluator now requires the native summary schema and typed counts,
parses/compares every frame and emitted pose, verifies source-prefix association,
and preserves integrity failures independently of a numerical score. Real
stereo runs were rescored into separate reviewed results; failed mono stays FAIL.
`R/software-review.json` records the original audit findings. Source protection
is recorded in `R/source-protection-final.json`: original HEAD/branch are
unchanged; other-task concurrent dirty-file changes are observed, not reverted.
The parent drone repository report is deliberately untouched.

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

## Next decision and RK3576 reuse

Further platform performance optimization is premature until monocular metric
initialization and continuous tracking pass. The independent headless build,
ASL boundary, source manifests, fixed-scale evaluation, native phase/resource
benchmark, offline interfaces and deployable reference artifacts can be reused
for RK3576. They do not require the M3C camera SDK. Actual board ABI validation,
sensor timing/calibration and performance remain the next hardware-dependent
steps after the user reconnects the board. No RK3576 branch edit is required.
