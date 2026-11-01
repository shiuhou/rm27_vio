# SchurVINS without ROS

This C++ boundary links the pinned estimator directly. The standalone build
rebuilds the estimator and its SVO/vikit dependency closure without
`SVO_USE_ROS`, ROS messages, roscore, catkin or Python in the executable.
It uses OpenCV 3.2, Eigen, yaml-cpp, glog, Boost, pthread and a pinned
target-architecture opengv binary. OpenCV GUI libraries remain linked because
upstream visualization helpers share translation units with processing code;
normal execution creates no windows and requires no display.

`factory.cpp` retains upstream parameter defaults and option mappings, replacing
ROS parameter access with YAML. Repeated `--params` arguments overlay earlier
files. Runtime forces loop/global-map/Ceres backends and automatic restart off.
`use_imu` is forced on. The estimator equations and mono initializer are unchanged.

Prepare into a new independent directory:

```sh
python3 -m tools.schurvins.headless.build --workspace SOURCE_WS --output NEW_HEADLESS_ROOT
```

The source workspace is read only. Preparation copies the dependency closure,
applies the recorded native-diagnostic and ARM NEON patches, and removes one
unused `vikit/params_helper.h` include from the copied frame handler header.
It records every resulting source hash. Build using the matching dependency
environment and source tree:

```sh
mkdir BUILD_DIR
cd BUILD_DIR
cmake REPO/tools/schurvins/headless -DSCHUR_SOURCE=NEW_HEADLESS_ROOT/src \
  -DSCHUR_DEP_LIB=TARGET_WS/devel/.private/opengv/lib
cmake --build . -- -j2
```

For Bionic ARM64 cross compilation, additionally set `SCHURVINS_CROSS` and
`SCHURVINS_SYSROOT`, pass `-DCMAKE_TOOLCHAIN_FILE=REPO/tools/schurvins/headless/aarch64.cmake`,
and select the ARM64 opengv directory. The reference SDK is a build environment;
the actual target loader, libraries, kernel and SDK still require board audit.

```sh
schurvins_headless --dataset EXTRACTED_ASL_ROOT --mode mono \
  --calib PINNED_SOURCE/svo_ros/param/calib/euroc_mono.yaml \
  --params PINNED_SOURCE/svo_ros/param/vio_mono.yaml \
  --output NEW_RUN_DIR --rate 0.5
```

`--dataset` accepts an ASL root containing `mav0`, or `mav0` itself.
Input requires camera `data.csv` and grayscale-decodable image files plus
`imu0/data.csv` in original SI units. Stereo uses cam0 and cam1 with exact
timestamp pairing. Integer source timestamps are preserved in output; conversion
to the native double-second IMU API retains that upstream precision limitation.

`--rate 0` is unpaced offline execution; `0.5` and `1` schedule against a steady
clock. `--max-frames` explicitly bounds diagnostic runs. No camera is cropped
from the default full sequence. A camera with no strictly older/covering IMU
history is rejected and counted, preventing the native stereo fatal assertion
without fabricating a pre-roll sample. The first V1_01 image has this condition.

Each exclusively created output contains effective parameters, `frames.csv`,
`poses.csv`, native diagnostic events and a summary. Frames include source and
scheduled/processing timestamps, native stage/quality, emission and admitted IMU
count. Queue depth counts overdue scheduled replay frames, not a physical device
queue. Admitted IMU counts do not by themselves prove internal estimator use.
Poses use native `T_W_B` and Hamilton wxyz, gated on native kTracking and exact
camera timestamp association. `EXECUTION_COMPLETED` reports process completion;
metric initialization, accepted visual corrections, uncertainty and operational
validity remain unqualified. Numerical evaluation and reliability gates are
performed separately. No real camera or UART is opened.

The compiled boundary regression tests use a JSON command argv supplied through
`SCHURVINS_HEADLESS_TEST_COMMAND`. They test actual C++ input refusal, stereo
pairing and immutable output behavior; absent that command they report a skip.

Output streams check open, write, flush and close failures. An unavailable/full
filesystem causes a nonzero exit; a failure summary is best effort. The compiled
regression forces a real EFBIG error before estimator startup. Existing measured
binaries and their results remain immutable when this boundary is rebuilt.

Package a recorded ARM64 dependency closure with
`python3 -m tools.schurvins.headless.package --help`. The utility verifies ELF
architecture and every transitive DT_NEEDED library, includes the target loader,
parameter/calibration files and available source/package notices, and records
all content hashes. Its `run` wrapper finds its own directory. The Bionic
reference bundle still needs a fresh target ABI audit before M3C execution.

For recorded interface verification, run
`python3 -m rm27.perception.vision.localization.experiments.schurvins_headless --help`.
Supply the actual run, sensor-only camera CSV, binary, sequence and fresh output.
It validates summary/frame/pose parity and exact camera association before
emitting existing `OnlinePoseRecord` observations. Native steady completion is
preserved, publication/receipt time remains absent, mono scale stays UNKNOWN,
and operational validity remains false. This is not a live MAVLink output source.
