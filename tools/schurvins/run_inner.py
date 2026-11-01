#!/usr/bin/env python
"""Run only inside the isolated, sourced Melodic container; outputs in /out."""
from __future__ import print_function
import argparse
import errno
import json
import os
import signal
import socket
import subprocess
import sys
import time
try:
    import xmlrpclib
except ImportError:
    from xmlrpc import client as xmlrpclib

try:
    from .ros_transport import monotonic_ns
except (ImportError, ValueError):
    from ros_transport import monotonic_ns


def stop_process_group(proc):
    """Bounded shutdown, including exit races between poll and killpg."""
    def send(signum):
        try:
            os.killpg(proc.pid, signum)
        except OSError as exc:
            if exc.errno != errno.ESRCH:
                raise
            # The group exited after poll. Reap the owned child below.

    if proc.poll() is None:
        send(signal.SIGINT)
        deadline = monotonic_ns() + 15 * 10**9
        while proc.poll() is None and monotonic_ns() < deadline:
            time.sleep(.1)
        if proc.poll() is None:
            send(signal.SIGKILL)
            deadline = monotonic_ns() + 5 * 10**9
            while proc.poll() is None and monotonic_ns() < deadline:
                time.sleep(.1)
    if proc.poll() is None:
        raise RuntimeError('owned process did not exit after SIGKILL')
    return proc.wait()


def cgroup_evidence():
    """Read actual container limits/counters; missing kernel fields stay absent."""
    evidence = {}
    for name in ['cpu.max', 'memory.max', 'memory.peak', 'memory.events']:
        path = '/sys/fs/cgroup/' + name
        if os.path.isfile(path):
            with open(path) as stream:
                evidence[name] = stream.read().strip()
    return evidence


def main(args):
    # A network-none Docker container has loopback but no hostname interface.
    # Keep every ROS1 XML-RPC address within that isolated loopback network.
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11311'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['RM27_NATIVE_EVENTS'] = '/out/native-events.jsonl'
    os.makedirs('/out/raw')
    os.makedirs('/out/logs')
    os.makedirs('/out/trace/stats')
    children, logs, exits, cleanup_errors = {}, [], {}, {}
    overrides = dict(automatic_reinitialization=False)

    def start(name, command):
        log = open('/out/' + name + '.log', 'w')
        logs.append(log)
        children[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           preexec_fn=os.setsid)
        return children[name]

    began = monotonic_ns()
    cgroup_start = cgroup_evidence()
    failure = None
    try:
        start('master', ['roscore'])
        for attempt in range(100):
            if subprocess.call(['rosparam', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE) == 0:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('ROS master not ready')
        source = '/ws/src/SchurVINS/svo_ros/param/'
        with open('/input/input.json') as f:
            input_manifest = json.load(f)
        subprocess.check_call(['rosparam', 'load', source+'vio_'+args.mode+'.yaml', '/svo'])
        # Explicitly override the mono launch's loop closure and platform paths.
        overrides = dict(runlc=False, use_global_map=False, use_ceres_backend=False,
                         automatic_reinitialization=False, trace_statistics=True,
                         trace_dir='/out/trace/stats', calib_file=source+'calib/euroc_'+args.mode+'.yaml',
                         cam0_topic='/cam0/image_raw', cam1_topic='/cam1/image_raw', imu_topic='/imu0')
        if input_manifest.get('dataset') == 'UZH_FPV':
            if args.mode != 'mono' or input_manifest.get('native_imu_config_complete') is not True:
                raise RuntimeError('UZH needs explicit complete monocular experiment calibration')
            overrides['calib_file'] = '/input/native_calibration.yaml'
        overrides.update(json.loads(args.parameters_json))
        for name, value in overrides.items():
            subprocess.check_call(['rosparam', 'set', '/svo/'+name, json.dumps(value)])
        subprocess.check_call(['rosparam', 'dump', '/out/parameters.yaml', '/svo'])
        # Native restart is disabled: no state discontinuity is silently stitched.
        start('observer', ['python', '/tools/ros_transport.py', 'observe', '--mode', args.mode,
                           '--out', '/out/raw'])
        node = start('native', ['/usr/bin/time', '-v', '-o', '/out/native-resource.txt',
                                'rosrun', 'svo_ros', 'svo_node'])
        for attempt in range(300):
            if node.poll() is not None:
                raise RuntimeError('native exited before readiness: '+str(node.returncode))
            if os.path.isfile('/out/raw/observer-ready'):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('observer not ready')
        # Subscriber registration is not a TCPROS handshake. Query the native
        # node's bus before allowing the first input to trigger publications.
        import rosgraph
        for attempt in range(300):
            if node.poll() is not None:
                raise RuntimeError('native exited before output connections')
            try:
                native_uri = rosgraph.Master('/rm27_replay_control').lookupNode('/svo')
                code, _, buses = xmlrpclib.ServerProxy(native_uri).getBusInfo('/rm27_replay_control')
                connected = {bus[4] for bus in buses if bus[2] == 'o' and bus[5]}
                if code == 1 and {'/svo/pose_imu', '/svo/info'} <= connected:
                    break
            except (rosgraph.MasterError, rosgraph.MasterFailure, socket.error, xmlrpclib.Error):
                pass  # Node registration/connection is allowed the same bounded startup interval.
            time.sleep(.1)
        else:
            raise RuntimeError('native pose/Info output connections not ready')
        player = start('player', ['python', '/tools/ros_transport.py', 'play', '--mode', args.mode,
                                  '--archive', '/input/sensors.zip', '--out', '/out/raw',
                                  '--rate', str(args.rate), '--start-offset', str(args.start_offset)])
        with open('/out/resources.jsonl', 'w') as resource_log:
            while player.poll() is None:
                if node.poll() is not None:
                    raise RuntimeError('native exited during playback: '+str(node.returncode))
                if children['observer'].poll() is not None:
                    raise RuntimeError('observer exited during playback')
                if (monotonic_ns()-began)/1e9 > args.timeout:
                    raise RuntimeError('run timeout')
                # cgroup includes estimator, player, observer and ROS; labels explicit.
                snapshot = dict(observed_monotonic_ns=monotonic_ns())
                for name, path in [('cgroup_cpu','/sys/fs/cgroup/cpu.stat'),
                                   ('cgroup_memory_bytes','/sys/fs/cgroup/memory.current')]:
                    if os.path.isfile(path):
                        with open(path) as f:
                            snapshot[name] = f.read().strip()
                resource_log.write(json.dumps(snapshot)+'\n')
                time.sleep(.25)
        exits['player'] = player.wait()
        if exits['player'] != 0:
            raise RuntimeError('player exit '+str(exits['player']))
        # Bounded native tail drain. Host checks observed source endpoint separately.
        previous, stable = -1, 0
        for attempt in range(100):
            if node.poll() is not None:
                raise RuntimeError('native exited before shutdown')
            size = os.path.getsize('/out/raw/info.jsonl')
            stable = stable+1 if size == previous else 0
            previous = size
            if stable >= 12:
                break
            time.sleep(.25)
        if stable < 12:
            raise RuntimeError('native output did not settle')
    except Exception as exc:
        failure = str(exc)
    finally:
        for name in ['player', 'native', 'observer', 'master']:
            if name in children:
                try:
                    exits[name] = stop_process_group(children[name])
                except Exception as exc:
                    exits[name] = children[name].poll()
                    cleanup_errors[name] = str(exc)
        for log in logs:
            log.close()
        with open('/out/execution.json', 'w') as f:
            json.dump(dict(exits=exits, failure=failure, cleanup_errors=cleanup_errors,
                           mode=args.mode, rate=args.rate,
                           start_offset_s=args.start_offset,
                           elapsed_s=(monotonic_ns()-began)/1e9,
                           cgroup_start=cgroup_start, cgroup_end=cgroup_evidence(),
                           resource_scope='native-resource: estimator process tree; resources.jsonl: entire container including transport and ROS',
                           native_imu_ingestion='UNOBSERVED',
                           automatic_reinitialization=overrides['automatic_reinitialization']), f, indent=2)
    clean = all(exits.get(name) == 0 for name in ['master', 'observer', 'native', 'player'])
    return 1 if failure or cleanup_errors or not clean else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mono','stereo'], required=True)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--timeout', type=float, default=900)
    parser.add_argument('--start-offset', type=float, default=0.)
    parser.add_argument('--parameters-json', default='{}')
    sys.exit(main(parser.parse_args()))
