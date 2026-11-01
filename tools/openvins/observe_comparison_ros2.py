"""Read-only mono/stereo ROS2 observer for the independent comparison entry."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import time


def write_input_match_evidence(subscriptions, path):
    """Gate playback on compatible publishers matched by every observer input.

    This is DDS discovery evidence for the observer, not native callback receipt.
    The caller invokes this until it returns True and then retains the snapshot.
    """
    counts = {topic: subscription.get_publisher_count()
              for topic, subscription in subscriptions.items()}
    if not counts or not all(count > 0 for count in counts.values()):
        return False
    with Path(path).open('x') as output:
        json.dump(dict(matched_publisher_counts=counts, observed_monotonic_ns=time.monotonic_ns(),
                       scope='compatible observer input publisher matches; not estimator ingestion'), output)
        output.write('\n')
    return True


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, Imu
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mono', 'stereo'], required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('rm27_openvins_comparison_observer')
    names = ['cam0', 'imu0', 'pose', 'odom'] + (['cam1'] if args.mode == 'stereo' else [])
    files = {name: open('/out/' + name + '.jsonl', 'x', buffering=1) for name in names}
    counts, stopping = {name: 0 for name in files}, False

    def stop(*unused):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def receive(name, m):
        row = dict(state_ns=m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
                   observed_monotonic_ns=time.monotonic_ns(), observed_system_ns=time.time_ns(),
                   frame_id=m.header.frame_id)
        if name.startswith('cam'):
            row.update(width=m.width, height=m.height, step=m.step, encoding=m.encoding,
                       is_bigendian=m.is_bigendian, pixels_sha256=hashlib.sha256(bytes(m.data)).hexdigest())
        elif name == 'imu0':
            row.update(gyro=[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z],
                       accel=[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        else:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            row.update(position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                       raw_covariance=list(m.pose.covariance))
            if name == 'odom':
                v = m.twist.twist.linear
                row.update(velocity_local=[v.x, v.y, v.z], child_frame_id=m.child_frame_id)
        files[name].write(json.dumps(row, allow_nan=False) + '\n')
        counts[name] += 1

    # A reliable observer must match the explicitly reliable bag publisher.
    # Native OpenVINS keeps its original input QoS, including BEST_EFFORT IMU.
    imu_qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE)
    image_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    topics = [('cam0', Image, '/cam0/image_raw', image_qos), ('imu0', Imu, '/imu0', imu_qos),
              ('pose', PoseWithCovarianceStamped, '/poseimu', 100), ('odom', Odometry, '/odomimu', 1000)]
    if args.mode == 'stereo':
        topics.append(('cam1', Image, '/cam1/image_raw', image_qos))
    subscriptions, inputs = [], {}
    for name, kind, topic, qos in topics:
        subscription = node.create_subscription(kind, topic, lambda m, n=name: receive(n, m), qos)
        subscriptions.append(subscription)
        if name.startswith('cam') or name == 'imu0':
            inputs[topic] = subscription
    Path('/out/observer.ready').write_text('subscriptions created\n')
    inputs_matched = False
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if not inputs_matched:
                inputs_matched = write_input_match_evidence(inputs, '/out/observer-inputs-matched.json')
    finally:
        for f in files.values():
            f.close()
        Path('/out/observer-counts.json').write_text(json.dumps(counts, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
