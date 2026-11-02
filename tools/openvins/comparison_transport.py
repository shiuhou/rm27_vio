"""Verified EuRoC mono/stereo ROS2 input for independent comparison runs.

The historical cam0-only transport and qualified V1_01 adapter are unchanged.
Only image/IMU topics enter the bag; truth stays in the host-side source ZIP.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

from tools.openvins.asl_transport import sha256_file, split_stamp
from tools.schurvins.ros_transport import inputs, scheduled_events


def artificial_input(prepared_input, original_reference_archive, mode):
    """Audit a derived sensor archive while keeping the reference host-only."""
    from rm27.perception.vision.localization.experiments.common import checked_file, read_json, require
    prepared = Path(prepared_input).resolve()
    reference = Path(original_reference_archive).resolve()
    manifest = read_json(prepared/'input.json')
    require(manifest.get('purpose') == 'EUROC_PUBLIC_REPLAY' and manifest.get('mode') == mode
            and manifest.get('truth_included') is False
            and manifest.get('derivation', {}).get('experiment_kind') == 'ARTIFICIAL_DISTURBANCE'
            and manifest['derivation'].get('validation', {}).get('status') == 'PASS',
            'COMPARISON_FAULT_INVALID', 'validated sensor-only artificial EuRoC input required')
    checked_file(reference, manifest['original_archive_sha256'])
    for name, digest in manifest['artifacts_sha256'].items():
        require(Path(name).name == name, 'COMPARISON_FAULT_INVALID', 'prepared-local artifact required')
        checked_file(prepared/name, digest)
    archive = prepared/'sensors.zip'
    with zipfile.ZipFile(archive) as derived, zipfile.ZipFile(reference) as original:
        streams, _ = events_from(derived, mode)
        source_streams, _ = events_from(original, mode)
        allowed = {'mav0/'+s+'/data.csv' for s in streams} | {'mav0/'+s+'/sensor.yaml' for s in streams}
        allowed |= {'mav0/'+s+'/data/'+filename for s, rows in streams.items() if s != 'imu0' for _, filename in rows}
        require(set(derived.namelist()) == allowed and len(derived.namelist()) == len(allowed),
                'TRUTH_ISOLATION_FAILED', 'derived archive contains only declared sensors')
        for sensor, rows in streams.items():
            require(len(rows) == manifest['counts'][sensor], 'COMPARISON_COUNT_MISMATCH', sensor)
            member = 'mav0/'+sensor+'/sensor.yaml'
            require(derived.read(member) == original.read(member),
                    'COMPARISON_CALIBRATION_CHANGED', sensor+' artificial calibration differs')
        full = [json.loads(line) for line in (prepared/'source-expected.jsonl').read_text().splitlines()]
        for sensor, rows in source_streams.items():
            source = [row for row in full if row['stream'] == sensor]
            require([row['state_ns'] for row in source] == [ns for ns, _ in rows]
                    and len(rows) == manifest['source_counts'][sensor],
                    'COMPARISON_SOURCE_DENOMINATOR_INVALID', sensor+' full source IDs differ')
            if sensor == 'imu0':
                require(all(row['gyro']+row['accel'] == values for row, (_, values) in zip(source, rows)),
                        'COMPARISON_SOURCE_DENOMINATOR_INVALID', 'full original IMU differs')
    schedule = read_json(prepared/'delivery-schedule.json') if (prepared/'delivery-schedule.json').exists() else None
    return manifest, archive, schedule


def storage_events(streams, rate, delivery_schedule=None):
    """Reuse the common delivery scheduler; retain sensor time separately.

    rosbag's rate rescales storage intervals. Multiplying the common wall-time
    deadline by that rate preserves an injected wall-time delay at playback.
    """
    from fractions import Fraction
    origin = min(rows[0][0] for rows in streams.values())
    output = []
    for due, ns, sensor, payload in scheduled_events(streams, rate, delivery_schedule=delivery_schedule):
        offset = Fraction(due)*Fraction(str(rate))
        if offset.denominator != 1:
            raise ValueError('delivery schedule is not representable by integer storage ns at this rate')
        output.append((origin+int(offset), ns, sensor, payload))
    return output


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


def readback_payload(archive, output, streams, playback):
    """Read every stored message against decoded source bytes and both clocks."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, Imu
    types = {sensor: Imu if sensor == 'imu0' else Image for sensor in streams}
    topics = {sensor: '/imu0' if sensor == 'imu0' else '/' + sensor + '/image_raw' for sensor in streams}
    z = archive
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    counts = {sensor: 0 for sensor in streams}
    for expected_storage_ns, ns, sensor, payload in playback:
        if not reader.has_next():
            raise ValueError('bag ended before source')
        topic, data, storage_ns = reader.read_next()
        m = deserialize_message(data, types[sensor])
        if (topic != topics[sensor] or storage_ns != expected_storage_ns or m.header.frame_id != sensor
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
    return counts


def verify_bag(archive, output, mode, sequence, *, prepared_input=None,
               original_reference_archive=None, rate=.5):
    """Re-audit one immutable bag without creating another large artifact."""
    archive, output = Path(archive).resolve(), Path(output).resolve()
    report = json.loads((output/'source_verification.json').read_text())
    if (report.get('sequence') != sequence or report.get('mode') != mode
            or report.get('source_sha256') != sha256_file(archive)
            or report.get('readback_exact') is not True or report.get('truth_included') is not False):
        raise ValueError('bag source, mode, sequence or sensor-only provenance differs')
    artifacts = report['artifacts_sha256']
    if set(p.name for p in output.iterdir()) != set(artifacts) | {'source_verification.json'}:
        raise ValueError('unexpected bag artifact')
    for name, digest in artifacts.items():
        if Path(name).name != name or not (output/name).is_file() or (output/name).is_symlink() or sha256_file(output/name) != digest:
            raise ValueError('bag artifact hash/type differs')
    schedule = None
    if prepared_input is not None:
        manifest, sensor_archive, schedule = artificial_input(prepared_input, original_reference_archive, mode)
        if (archive != sensor_archive or report.get('purpose') != 'EUROC_ARTIFICIAL_COMPARISON'
                or report.get('playback_rate') != rate or report.get('delivery_schedule') != schedule
                or report.get('prepared_input_sha256') != sha256_file(Path(prepared_input)/'input.json')
                or report.get('original_reference_archive_sha256') != manifest['original_archive_sha256']):
            raise ValueError('artificial bag provenance or declared delivery rate differs')
    elif report.get('purpose') != 'EUROC_PUBLIC_COMPARISON':
        raise ValueError('artificial bag requires explicit reference and prepared input')
    with zipfile.ZipFile(archive) as source:
        streams, events = events_from(source, mode)
        playback = storage_events(streams, rate, schedule) if prepared_input is not None else [(ns,ns,s,p) for ns,s,p in events]
        counts = readback_payload(source, output, streams, playback)
    if counts != report['verified_counts']:
        raise ValueError('readback counts differ')
    return dict(status='PASS',readback_exact=True,verified_counts=counts,
        source_verification_sha256=sha256_file(output/'source_verification.json'),
        qualification='REAL_PUBLIC_INPUT_BYTE_TIMESTAMP_READBACK_ONLY_NO_ESTIMATOR')


def build_bag(archive, output, mode, sequence, *, prepared_input=None,
              original_reference_archive=None, rate=.5, reserve_bytes=20*1024**3):
    import rosbag2_py
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import Image, Imu

    archive, output = Path(archive).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    if not isinstance(sequence, str) or not sequence or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_' for c in sequence):
        raise ValueError('explicit sequence name required')
    artificial = prepared_input is not None or original_reference_archive is not None
    manifest, schedule = None, None
    if artificial:
        if prepared_input is None or original_reference_archive is None:
            raise ValueError('both prepared input and original reference required')
        manifest, sensor_archive, schedule = artificial_input(prepared_input, original_reference_archive, mode)
        if archive != sensor_archive:
            raise ValueError('explicit derived sensor archive differs from prepared input')
    source_hash = sha256_file(archive)
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise ValueError('duplicate ZIP members')
        streams, events = events_from(z, mode)
        ancestor = output.parent
        while not ancestor.exists():
            ancestor = ancestor.parent
        # ROS Image stores decoded pixels, which can greatly exceed PNG bytes.
        # Bound the actual payload and database envelopes before creating output.
        estimate = sum(image_pixels(z,sensor,payload).nbytes for _,sensor,payload in events if sensor != 'imu0')
        estimate += len(events)*4096 + 1024**2
        if shutil.disk_usage(ancestor).free-estimate < reserve_bytes:
            raise ValueError('comparison bag would breach storage reserve')
        playback = storage_events(streams, rate, schedule) if artificial else [(ns, ns, s, p) for ns, s, p in events]
        types = {sensor: Imu if sensor == 'imu0' else Image for sensor in streams}
        topics = {sensor: '/imu0' if sensor == 'imu0' else '/' + sensor + '/image_raw' for sensor in streams}
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for i, sensor in enumerate(streams):
            writer.create_topic(rosbag2_py.TopicMetadata(id=i + 1, name=topics[sensor],
                                type='sensor_msgs/msg/' + types[sensor].__name__, serialization_format='cdr'))
        for storage_ns, ns, sensor, payload in playback:
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
            writer.write(topics[sensor], serialize_message(m), storage_ns)
        del writer
        counts = readback_payload(z, output, streams, playback)
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
    if artificial:
        report.update(purpose='EUROC_ARTIFICIAL_COMPARISON', experiment_kind='ARTIFICIAL_DISTURBANCE',
            original_reference_archive_sha256=manifest['original_archive_sha256'],
            derived_sensor_archive_sha256=source_hash, prepared_input_sha256=sha256_file(Path(prepared_input)/'input.json'),
            profile=manifest['derivation']['profile'], source_counts=manifest['source_counts'],
            delivery_schedule=schedule, playback_rate=rate,
            timestamp_policy='sensor header unchanged; storage timeline from common delivery scheduler scaled for declared playback rate',
            estimator_ingestion_verified=False, estimator_reference_mounted=False)
        if artificial_input(prepared_input, original_reference_archive, mode)[0] != manifest:
            raise ValueError('prepared input manifest changed during bag construction')
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
    parser.add_argument('--prepared-input', type=Path)
    parser.add_argument('--original-reference-archive', type=Path)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    operation = verify_bag if args.verify_only else build_bag
    print(json.dumps(operation(args.archive, args.output, args.mode, args.sequence,
        prepared_input=args.prepared_input, original_reference_archive=args.original_reference_archive,
        rate=args.rate), indent=2))
