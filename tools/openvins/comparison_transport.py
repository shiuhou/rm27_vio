"""Verified EuRoC mono/stereo ROS2 input for independent comparison runs.

The historical cam0-only transport and qualified V1_01 adapter are unchanged.
Only image/IMU topics enter the bag; truth stays in the host-side source ZIP.
"""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

from tools.openvins.asl_transport import sha256_file, split_stamp
from tools.schurvins.ros_transport import inputs


def events_from(archive, mode):
    streams = inputs(archive, mode)
    events = sorted(((ns, sensor, payload) for sensor, rows in streams.items()
                     for ns, payload in rows),
                    key=lambda row: (row[0], {'imu0': 0, 'cam0': 1, 'cam1': 2}[row[1]]))
    for ns, _, _ in events:
        split_stamp(ns)
    return streams, events


def image_pixels(archive, sensor, filename):
    import cv2
    import numpy as np
    pixels = cv2.imdecode(np.frombuffer(archive.read('mav0/' + sensor + '/data/' + filename),
                                       np.uint8), cv2.IMREAD_UNCHANGED)
    if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
        raise ValueError('uint8 mono image required')
    return pixels


def build_bag(archive, output, mode, sequence):
    import rosbag2_py
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import Image, Imu

    archive, output = Path(archive).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if not isinstance(sequence, str) or not sequence or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_' for c in sequence):
        raise ValueError('explicit sequence name required')
    source_hash = sha256_file(archive)
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise ValueError('duplicate ZIP members')
        streams, events = events_from(z, mode)
        types = {sensor: Imu if sensor == 'imu0' else Image for sensor in streams}
        topics = {sensor: '/imu0' if sensor == 'imu0' else '/' + sensor + '/image_raw' for sensor in streams}
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for i, sensor in enumerate(streams):
            writer.create_topic(rosbag2_py.TopicMetadata(id=i + 1, name=topics[sensor],
                                type='sensor_msgs/msg/' + types[sensor].__name__, serialization_format='cdr'))
        for ns, sensor, payload in events:
            m = types[sensor]()
            m.header.stamp.sec, m.header.stamp.nanosec = split_stamp(ns)
            m.header.frame_id = sensor
            if sensor == 'imu0':
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = payload[:3]
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = payload[3:]
                m.orientation_covariance[0] = -1.
            else:
                pixels = image_pixels(z, sensor, payload)
                m.height, m.width = pixels.shape
                m.encoding, m.step, m.data = 'mono8', m.width, pixels.tobytes()
            writer.write(topics[sensor], serialize_message(m), ns)
        del writer
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        counts = {sensor: 0 for sensor in streams}
        for ns, sensor, payload in events:
            if not reader.has_next():
                raise ValueError('bag ended before source')
            topic, data, storage_ns = reader.read_next()
            m = deserialize_message(data, types[sensor])
            if (topic != topics[sensor] or storage_ns != ns or m.header.frame_id != sensor
                    or m.header.stamp.sec * 10**9 + m.header.stamp.nanosec != ns):
                raise ValueError('topic/order/timestamp/frame changed')
            if sensor == 'imu0':
                actual = [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                          m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
                if actual != payload or m.orientation_covariance[0] != -1:
                    raise ValueError('IMU values changed')
            else:
                expected = image_pixels(z, sensor, payload)
                if ((m.height, m.width) != expected.shape or m.encoding != 'mono8'
                        or m.step != m.width or m.is_bigendian != 0 or bytes(m.data) != expected.tobytes()):
                    raise ValueError('image pixels changed')
            counts[sensor] += 1
        if reader.has_next():
            raise ValueError('unexpected extra samples')
        del reader
        calibration = {name: hashlib.sha256(z.read(name)).hexdigest() for name in z.namelist()
                       if name in ['mav0/' + sensor + '/sensor.yaml' for sensor in streams]}
    if sha256_file(archive) != source_hash:
        raise ValueError('source archive changed')
    report = dict(schema_version=1, purpose='EUROC_PUBLIC_COMPARISON', sequence=sequence, mode=mode,
                  source=str(archive), source_sha256=source_hash, verified_counts=counts,
                  readback_exact=True, truth_included=False,
                  source_calibration_sha256=calibration,
                  timestamp_policy='integer ns unchanged; IMU, cam0, cam1 at equal timestamps',
                  topics=list(topics.values()), estimator_ingestion_verified=False,
                  artifacts_sha256={p.name: sha256_file(p) for p in sorted(output.iterdir()) if p.is_file()})
    with (output/'source_verification.json').open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write('\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--mode', choices=['mono', 'stereo'], required=True)
    parser.add_argument('--sequence', required=True)
    args = parser.parse_args()
    print(json.dumps(build_bag(args.archive, args.output, args.mode, args.sequence), indent=2))
