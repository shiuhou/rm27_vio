#!/usr/bin/env bash
# Independent mode-aware comparison. The frozen V1_01 mono wrapper is unchanged.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=119
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/out/ros-log
export PYTHONPATH=/repo:${PYTHONPATH:-}
cd /out
test ! -e native.log
case "$VIO_MODE" in
  mono) stereo=false; cameras=1 ;;
  stereo) stereo=true; cameras=2 ;;
  *) exit 2 ;;
esac
case "$VIO_RATE" in 0.5|1.0) ;; *) exit 2 ;; esac
native_pid=''
recorder_pid=''
player_pid=''
observer_pid=''
sampler_pid=''
cleanup() {
  for task_pid in "$player_pid" "$native_pid"; do
    if [ -n "$task_pid" ] && kill -0 "$task_pid" 2>/dev/null; then
      kill -INT "$task_pid" 2>/dev/null || true
    fi
  done
  for task_pid in "$recorder_pid" "$observer_pid" "$sampler_pid"; do
    if [ -n "$task_pid" ] && kill -0 "$task_pid" 2>/dev/null; then
      kill -TERM "$task_pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
python3 /repo/tools/openvins/observe_comparison_ros2.py --mode "$VIO_MODE" > observer.log 2>&1 &
observer_pid=$!
for attempt in $(seq 1 30); do
  kill -0 "$observer_pid"
  [ -e observer.ready ] && break
  sleep 1
done
test -e observer.ready
/usr/bin/time -v -o native-resource.txt \
  /ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args \
  -p config_path:=/upstream/config/euroc_mav/estimator_config.yaml \
  -p use_stereo:="$stereo" -p max_cameras:="$cameras" -p verbosity:=INFO \
  -p save_total_state:=true -p filepath_est:=/out/state_estimate.txt \
  -p filepath_std:=/out/state_deviation.txt \
  -p record_timing_information:=true -p record_timing_filepath:=/out/timing.txt \
  > native.log 2>&1 &
meter_pid=$!
for attempt in $(seq 1 20); do
  native_pid=$(pgrep -P "$meter_pid" -f run_subscribe_msckf || true)
  [ -n "$native_pid" ] && break
  sleep 0.1
done
test -n "$native_pid"
input_topics=(/cam0/image_raw /imu0)
if [ "$VIO_MODE" = stereo ]; then input_topics+=(/cam1/image_raw); fi
ready=false
for attempt in $(seq 1 30); do
  kill -0 "$native_pid"
  ready=true
  for topic in "${input_topics[@]}"; do
    sensor=${topic//\//_}
    ros2 topic info "$topic" > "readiness${sensor}.txt" 2>&1 || true
    if ! grep -q 'Subscription count: 2' "readiness${sensor}.txt"; then ready=false; fi
  done
  [ "$ready" = true ] && break
  sleep 1
done
test "$ready" = true
for topic in "${input_topics[@]}"; do
  sensor=${topic//\//_}
  ros2 topic info "$topic" --verbose > "endpoints${sensor}.txt"
done
ros2 bag record -o /out/online-output /poseimu /odomimu > recorder.log 2>&1 &
recorder_pid=$!
ready=false
for attempt in $(seq 1 30); do
  kill -0 "$recorder_pid"
  ros2 topic info /poseimu > output-readiness.txt 2>&1 || true
  ros2 topic info /odomimu > odom-readiness.txt 2>&1 || true
  if grep -q 'Subscription count: 2' output-readiness.txt && grep -q 'Subscription count: 2' odom-readiness.txt; then
    ready=true
    break
  fi
  sleep 1
done
test "$ready" = true
ros2 bag play /data --rate "$VIO_RATE" --disable-keyboard-controls \
  --qos-profile-overrides-path /repo/tools/openvins/comparison_qos.yaml \
  --start-paused --wait-for-all-acked 5000 > player.log 2>&1 &
player_pid=$!
# Capture actual offered/requested QoS while the player exists but no samples
# have been released. Observer publisher matching is stronger than graph counts;
# neither proves that native OpenVINS ingests each input sample.
ready=false
for attempt in $(seq 1 30); do
  kill -0 "$player_pid"
  kill -0 "$native_pid"
  kill -0 "$observer_pid"
  if [ -e observer-inputs-matched.json ]; then
    ready=true
    for topic in "${input_topics[@]}"; do
      sensor=${topic//\//_}
      ros2 topic info "$topic" --verbose > "playback-endpoints${sensor}.txt"
      if ! grep -q 'Publisher count: 1' "playback-endpoints${sensor}.txt" || \
         ! grep -q 'Subscription count: 2' "playback-endpoints${sensor}.txt"; then ready=false; fi
    done
    [ "$ready" = true ] && break
  fi
  sleep 1
done
test "$ready" = true
python3 /repo/tools/openvins/sample_comparison_resources.py > resource-sampling.log 2>&1 &
sampler_pid=$!
set +e
timeout 15 ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}' > resume.log 2>&1
resume_status=$?
set -e
printf '%s\n' "$resume_status" > resume.exit
test "$resume_status" -eq 0
set +e
wait "$player_pid"
player_status=$?
set -e
printf '%s\n' "$player_status" > player.exit
test "$player_status" -eq 0
player_pid=''
if kill -0 "$sampler_pid" 2>/dev/null; then kill -TERM "$sampler_pid"; fi
wait "$sampler_pid"
sampler_pid=''
stable=0
previous=''
for attempt in $(seq 1 30); do
  kill -0 "$native_pid"
  current=$(stat -c '%s' state_estimate.txt)
  if [ "$current" = "$previous" ]; then stable=$((stable + 1)); else stable=0; fi
  [ "$stable" -ge 5 ] && break
  previous=$current
  sleep 1
done
kill -INT "$native_pid"
set +e
wait "$meter_pid"
native_status=$?
set -e
printf '%s\n' "$native_status" > native.exit
native_pid=''
# Pinned ROS2 CLI recorder flushes metadata and returns zero on SIGTERM.
kill -TERM "$recorder_pid"
set +e
wait "$recorder_pid"
recorder_status=$?
set -e
printf '%s\n' "$recorder_status" > recorder.exit
recorder_pid=''
kill -TERM "$observer_pid"
set +e
wait "$observer_pid"
observer_status=$?
set -e
printf '%s\n' "$observer_status" > observer.exit
observer_pid=''
test "$native_status" -eq 0
test "$recorder_status" -eq 0
test "$observer_status" -eq 0
test "$stable" -ge 5
