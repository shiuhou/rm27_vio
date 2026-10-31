#!/usr/bin/env python
"""ROS1/Python2-compatible ASL transport and independent observer.

Only cam0[/cam1] and raw IMU are published. No truth is ever loaded here.
JSONL distinguishes publication, observer receipt, and native info callbacks.
"""
from __future__ import print_function
import argparse
import ctypes
from decimal import Decimal, ROUND_HALF_EVEN
import hashlib
import json
import math
import os
import threading
import time
import zipfile


class Timespec(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]


def monotonic_ns():
    value = Timespec()
    if ctypes.CDLL(None).clock_gettime(1, ctypes.byref(value)) != 0:
        raise RuntimeError('CLOCK_MONOTONIC unavailable')
    return value.tv_sec * 10**9 + value.tv_nsec


def rows(text, sensor):
    if sensor not in ('imu0', 'cam0', 'cam1'):
        raise ValueError('unsupported sensor')
    result, previous = [], -1
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        values = line.split(',')
        if len(values) != (7 if sensor == 'imu0' else 2):
            raise ValueError('CSV column count')
        ns = int(values[0])
        if ns <= previous or ns < 0:
            raise ValueError('timestamps must strictly increase')
        previous = ns
        if sensor == 'imu0':
            data = [float(v) for v in values[1:]]
            if not all(not math.isnan(v) and not math.isinf(v) for v in data):
                raise ValueError('nonfinite IMU')
        else:
            data = values[1]
            if data != str(ns) + '.png':
                raise ValueError('camera filename must equal timestamp')
        result.append((ns, data))
    if not result:
        raise ValueError('empty stream')
    return result


def uzh_rows(text, sensor):
    """Read genuine UZH text; preserve raw text in the separate source ledger.

    ROS1 carries integer ns. The explicitly recorded representation conversion
    rounds decimal seconds to nearest ns, with ties to even; it is not a claim
    about the sensor clock's physical accuracy.
    """
    result, previous, previous_id = [], -1, -1
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        values = line.split()
        if len(values) != (8 if sensor == 'imu0' else 3):
            raise ValueError('UZH text column count')
        sample_id = int(values[0])
        ns = int((Decimal(values[1])*10**9).to_integral_value(rounding=ROUND_HALF_EVEN))
        if ns <= previous or ns < 0 or sample_id <= previous_id:
            raise ValueError('UZH source IDs/timestamps must strictly increase')
        previous, previous_id = ns, sample_id
        if sensor == 'imu0':
            payload = [float(v) for v in values[2:]]
            if not all(not math.isnan(v) and not math.isinf(v) for v in payload):
                raise ValueError('nonfinite UZH IMU')
        else:
            payload = values[2]
            if (not payload.startswith('img/') or '..' in payload.split('/')
                    or not payload.endswith('.png')):
                raise ValueError('invalid UZH image path')
        result.append((ns, payload))
    if not result:
        raise ValueError('empty UZH stream')
    return result


def inputs(archive, mode, dataset='EUROC', camera_id='cam0'):
    if mode not in ('mono', 'stereo'):
        raise ValueError('mono/stereo mode required')
    if dataset == 'UZH_FPV':
        if mode != 'mono' or camera_id not in ('cam0', 'cam1'):
            raise ValueError('UZH only independent monocular mode is qualified for transport')
        index = 'left_images.txt' if camera_id == 'cam0' else 'right_images.txt'
        return dict(imu0=uzh_rows(archive.read('imu.txt').decode(), 'imu0'),
                    cam0=uzh_rows(archive.read(index).decode(), 'cam0'))
    if dataset != 'EUROC':
        raise ValueError('unsupported dataset')
    streams = ['imu0', 'cam0'] + (['cam1'] if mode == 'stereo' else [])
    result = {}
    for sensor in streams:
        result[sensor] = rows(archive.read('mav0/' + sensor + '/data.csv').decode(), sensor)
    if mode == 'stereo' and [r[0] for r in result['cam0']] != [r[0] for r in result['cam1']]:
        raise ValueError('stereo requires exactly equal source timestamps')
    return result


def emit(stream, row):
    stream.write(json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n')


def scheduled_events(data, rate, start_offset_s=0, delivery_schedule=None):
    """Schedule delivery while retaining the unmodified sensing timestamps."""
    if rate not in (.5, 1.) or isinstance(rate, bool):
        raise ValueError('explicit 0.5/1 replay rate required')
    offset = Decimal(str(start_offset_s))
    if not offset.is_finite() or offset < 0:
        raise ValueError('nonnegative finite start offset required')
    origin = min(values[0][0] for values in data.values())
    cutoff = origin + int(offset*10**9)
    delays = {}
    if delivery_schedule is not None:
        if (delivery_schedule.get('schema_version') != 1 or
                delivery_schedule.get('time_semantics') != 'DELIVERY_DELAY_ONLY_SOURCE_STAMP_UNCHANGED'):
            raise ValueError('unsupported delivery schedule')
        identities = {(stream, ns) for stream, values in data.items() for ns, _ in values}
        for entry in delivery_schedule['entries']:
            key = (entry['stream'], entry['state_ns'])
            delay = entry['delivery_delay_ns']
            if key not in identities or key in delays or type(delay) is not int or delay < 0:
                raise ValueError('invalid/duplicate delivery event')
            delays[key] = delay
    events = [(int(Decimal(ns-cutoff)/Decimal(str(rate)))+delays.get((stream, ns), 0),
               ns, stream, payload) for stream, values in data.items() for ns, payload in values if ns >= cutoff]
    if not events or any(not any(row[2] == stream for row in events) for stream in data):
        raise ValueError('start offset leaves an empty sensor stream')
    return sorted(events, key=lambda row: (row[0], row[1], row[2] != 'imu0', row[2]))


def play(args):
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import Image, Imu
    rospy.init_node('rm27_schur_player', disable_signals=True)
    with open(os.path.join(os.path.dirname(args.archive), 'input.json')) as f:
        metadata = json.load(f)
    dataset = metadata.get('dataset', 'EUROC')
    with zipfile.ZipFile(args.archive) as archive:
        data = inputs(archive, args.mode, dataset, metadata.get('source_camera_id', 'cam0'))
        pubs = {s: rospy.Publisher('/imu0' if s == 'imu0' else '/' + s + '/image_raw',
                                  Imu if s == 'imu0' else Image, queue_size=10000)
                for s in data}
        deadline = monotonic_ns() + 30 * 10**9
        while any(p.get_num_connections() < 2 for p in pubs.values()):
            if monotonic_ns() > deadline:
                raise RuntimeError('native and observer input subscriptions not ready')
            time.sleep(.05)
        delivery_schedule = None
        schedule_path = os.path.join(os.path.dirname(args.archive), 'delivery-schedule.json')
        if os.path.isfile(schedule_path):
            with open(schedule_path) as f:
                delivery_schedule = json.load(f)
        events = scheduled_events(data, args.rate, getattr(args, 'start_offset', 0), delivery_schedule)
        begin = monotonic_ns()
        counts = {s: 0 for s in data}
        with open(os.path.join(args.out, 'published.jsonl'), 'w') as log:
            for due_offset_ns, ns, sensor, payload in events:
                due = begin + due_offset_ns
                remaining = due - monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1e9)
                msg = Imu() if sensor == 'imu0' else Image()
                msg.header.stamp = rospy.Time(ns // 10**9, ns % 10**9)
                msg.header.frame_id = sensor
                row = dict(state_ns=ns, stream=sensor)
                if sensor == 'imu0':
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = payload[:3]
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = payload[3:]
                    msg.orientation_covariance[0] = -1
                    row.update(gyro=payload[:3], accel=payload[3:])
                else:
                    member = payload if dataset == 'UZH_FPV' else 'mav0/' + sensor + '/data/' + payload
                    pixels = cv2.imdecode(np.frombuffer(archive.read(member),
                                                       np.uint8), cv2.IMREAD_UNCHANGED)
                    if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
                        raise ValueError('mono8 image required')
                    msg.height, msg.width = pixels.shape
                    msg.encoding, msg.step, msg.data = 'mono8', msg.width, pixels.tobytes()
                    row.update(pixels_sha256=hashlib.sha256(msg.data).hexdigest(), width=msg.width,
                               height=msg.height, encoding='mono8', step=msg.step)
                row['publish_call_monotonic_ns'] = monotonic_ns()
                row['scheduler_lateness_ns'] = row['publish_call_monotonic_ns'] - due
                pubs[sensor].publish(msg)
                row['publish_return_monotonic_ns'] = monotonic_ns()
                emit(log, row)
                counts[sensor] += 1
            # Let rospy transport queues drain; this does not prove native IMU ingestion.
            time.sleep(1)
        with open(os.path.join(args.out, 'player-summary.json'), 'w') as log:
            json.dump(dict(counts=counts, rate=args.rate, elapsed_s=(monotonic_ns()-begin)/1e9,
                           estimator_imu_ingestion_verified=False), log)


def observe(args):
    import rospy
    from sensor_msgs.msg import Image, Imu
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from svo_msgs.msg import Info
    rospy.init_node('rm27_schur_observer')
    names = ['imu0', 'cam0', 'pose', 'info'] + (['cam1'] if args.mode == 'stereo' else [])
    streams = {s: open(os.path.join(args.out, s + '.jsonl'), 'w') for s in names}
    watchdog = open(os.path.join(args.out, 'watchdog.jsonl'), 'w')
    watchdog_stop = threading.Event()
    lock = threading.Lock()

    def watchdog_loop():
        # The timer continues without sensor callbacks. It records only its
        # observer clock; no sensor time is invented during a stopped stream.
        while not watchdog_stop.wait(.02):
            emit(watchdog, dict(schema_version=1, trigger='INDEPENDENT_SCHEDULE',
                observed_monotonic_ns=monotonic_ns(),
                clock_source='OBSERVER_CLOCK_MONOTONIC', event='WATCHDOG_TICK', sensor_now=None))
        watchdog.flush()

    watchdog_thread = threading.Thread(target=watchdog_loop)
    watchdog_thread.daemon = True

    def callback(msg, name):
        row = dict(state_ns=msg.header.stamp.to_nsec(), native_sequence=msg.header.seq,
                   observed_monotonic_ns=monotonic_ns(), frame_id=msg.header.frame_id)
        if name == 'pose':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row.update(position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                       covariance=list(msg.pose.covariance))
        elif name == 'info':
            row.update(stage=msg.stage, tracking_quality=msg.tracking_quality,
                       processing_time_s=msg.processing_time, num_matches=msg.num_matches)
        elif name == 'imu0':
            g, a = msg.angular_velocity, msg.linear_acceleration
            row.update(gyro=[g.x, g.y, g.z], accel=[a.x, a.y, a.z])
        else:
            row.update(pixels_sha256=hashlib.sha256(msg.data).hexdigest(), width=msg.width,
                       height=msg.height, encoding=msg.encoding, step=msg.step)
        with lock:
            emit(streams[name], row)
            streams[name].flush()

    subscribers = []
    for name in names:
        topic = {'pose': '/svo/pose_imu', 'info': '/svo/info', 'imu0': '/imu0'}.get(name, '/' + name + '/image_raw')
        kind = {'pose': PoseWithCovarianceStamped, 'info': Info, 'imu0': Imu}.get(name, Image)
        subscribers.append(rospy.Subscriber(topic, kind, callback, callback_args=name,
                                           queue_size=10000, buff_size=32*1024*1024))
    with open(os.path.join(args.out, 'observer-ready'), 'w') as log:
        log.write('subscriptions created; publisher handshake checked separately\n')
    watchdog_thread.start()
    try:
        rospy.spin()
    finally:
        watchdog_stop.set()
        watchdog_thread.join(2)
        if watchdog_thread.is_alive():
            raise RuntimeError('owned watchdog thread failed to stop')
        watchdog.close()
        with open(os.path.join(args.out, 'watchdog-complete.json'), 'w') as log:
            json.dump(dict(schema_version=1, trigger='INDEPENDENT_SCHEDULE',
                           thread_joined=True, clock_source='OBSERVER_CLOCK_MONOTONIC'), log)
        for stream in streams.values():
            stream.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['play', 'observe'])
    parser.add_argument('--mode', choices=['mono', 'stereo'], required=True)
    parser.add_argument('--archive')
    parser.add_argument('--out', required=True)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--start-offset', type=float, default=0.)
    args = parser.parse_args()
    if args.rate not in (.5, 1.):
        parser.error('rate must be 0.5 or 1.0')
    (play if args.action == 'play' else observe)(args)
