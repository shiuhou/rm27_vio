#!/usr/bin/env python3
"""Verify an existing PX4 SITL artifact's ODOMETRY reception, never VIO fusion.

Uses a minimal startup script (MAVLink and its RAM dataman dependency), an owned Docker network namespace
with no external interfaces, read-only executable/libraries and an output-only
writable directory. No estimator, commander, simulator or device driver starts.
An archived source/build or a hash-locked portable executable is read-only.
Real-record transport requires two explicit laboratory opt-ins; it does not
promote an unqualified estimator state or start a flight-control module.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import uuid


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_laboratory_records(path, *, enabled):
    """Validate the explicit, non-operational transport boundary before launch."""
    if path is None:
        if enabled:
            raise ValueError('observer-only laboratory mode requires laboratory records')
        return None
    if not enabled:
        raise ValueError('real records require --observer-only-laboratory')
    value = json.loads(Path(path).read_text(), parse_constant=lambda text: (_ for _ in ()).throw(
        ValueError('non-finite laboratory JSON: '+text)))
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value.get('schema_version') != 1
            or value.get('mode') != 'OBSERVER_ONLY_LABORATORY'
            or value.get('qualification') != 'UNQUALIFIED'
            or not isinstance(value.get('records'), list) or not 1 <= len(value['records']) <= 12):
        raise ValueError('bounded unqualified observer-only laboratory records required')
    cases = set()
    for row in value['records']:
        if (not isinstance(row, dict) or set(row) != {'case', 'payload', 'policy', 'source'}
                or not isinstance(row['case'], str)
                or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', row['case'])
                or row['case'] in cases or not isinstance(row['source'], dict) or not row['source']
                or not isinstance(row['policy'], dict) or row['policy'].get('output_eligible') is not False):
            raise ValueError('unique case, source provenance and rejected operational policy required')
        cases.add(row['case'])
        payload = row['payload']
        if (not isinstance(payload, dict) or payload.get('frame_id') != 20
                or payload.get('child_frame_id') != 12 or payload.get('quality') != 0
                or payload.get('reset_counter') != 0 or payload.get('estimator_type') != 2
                or payload.get('pose_covariance') is not None
                or payload.get('velocity_covariance') is not None):
            raise ValueError('laboratory payload requires LOCAL_FRD/BODY_FRD, unknown quality/covariance and session reset0')
        for name, length in [('position_ned_m', 3), ('orientation_ned_body_wxyz', 4),
                             ('velocity_body_m_s', 3), ('angular_velocity_body_rad_s', 3)]:
            vector = payload.get(name)
            if (not isinstance(vector, list) or len(vector) != length
                    or any(type(item) not in (int, float) or not math.isfinite(item) for item in vector)):
                raise ValueError('complete measured finite laboratory vector required: '+name)
        evidence = payload.get('evidence')
        if (not isinstance(evidence, dict) or evidence.get('laboratory_mode') != 'OBSERVER_ONLY_LABORATORY'
                or evidence.get('qualification') != 'UNQUALIFIED'
                or evidence.get('control_use_permitted') is not False):
            raise ValueError('laboratory payload must explicitly prohibit control use')
        reconstruct_payload(payload)
    return value


def reconstruct_payload(raw):
    """Use the existing adapter type without upgrading any validity evidence."""
    from rm27.perception.vision.localization.mavlink import OdometryPayload
    payload = OdometryPayload(raw['time_usec'], tuple(raw['position_ned_m']),
        tuple(raw['orientation_ned_body_wxyz']), tuple(raw['velocity_body_m_s']),
        tuple(raw['angular_velocity_body_rad_s']), raw['reset_counter'], raw['evidence'],
        frame_id=raw['frame_id'])
    if payload.to_dict() != raw:
        raise ValueError('laboratory payload reconstruction differs')
    return payload


def load_artifact_lock(path, image):
    """Read only the declared portable files; never enter another Git checkout."""
    path = Path(path).resolve()
    value = json.loads(path.read_text())
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value.get('schema_version') != 1 or value.get('purpose') != 'PX4_SITL_RECEPTION_ARTIFACT'
            or value.get('qualification') != 'ARCHIVED_FIXED_ARTIFACT_NOT_HARDWARE'
            or value.get('image_id') != image
            or not re.fullmatch(r'[0-9a-f]{40}', value.get('source_head', ''))):
        raise ValueError('fixed-version isolated reception artifact lock required')
    for name in ('binary', 'build_version'):
        item = value.get(name)
        if not isinstance(item, dict) or set(item) != {'path', 'sha256'}:
            raise ValueError('artifact lock file identity required: '+name)
        target = Path(item['path'])
        if not target.is_absolute() or not target.is_file() or digest(target) != item['sha256']:
            raise ValueError('artifact file hash differs: '+name)
    dependencies = value.get('dynamic_dependencies_sha256')
    if not isinstance(dependencies, dict) or not dependencies:
        raise ValueError('artifact dynamic dependency lock required')
    for target, expected in dependencies.items():
        if not Path(target).is_absolute() or not Path(target).is_file() or digest(target) != expected:
            raise ValueError('artifact dependency hash differs: '+target)
    return value


def parse_received(text, expected):
    """Check actual uORB listener fields; listener exit alone proves nothing."""
    if 'TOPIC: vehicle_visual_odometry' not in text:
        raise ValueError('vehicle_visual_odometry topic not observed')
    scalar_keys = {'pose_frame', 'velocity_frame', 'reset_counter', 'quality'}
    for key, values in expected.items():
        if key in scalar_keys:
            continue
        found = re.search(r'^\s*'+re.escape(key)+r':\s*\[([^\]]+)\]', text, re.M)
        if found is None:
            raise ValueError('missing uORB vector: '+key)
        actual = [float(x.strip()) for x in found.group(1).split(',')]
        if values is None:
            values = [None]*3
        if len(actual) != len(values) or any(
                (not math.isnan(x) if y is None else not math.isfinite(x) or abs(x-y) > 2e-5)
                for x, y in zip(actual, values)):
            raise ValueError('uORB vector differs: '+key)
    scalars = {'pose_frame': expected.get('pose_frame', 1),
               'velocity_frame': expected.get('velocity_frame', 3), 'reset_counter': expected['reset_counter']}
    if 'quality' in expected:
        scalars['quality'] = expected['quality']
    for key, value in scalars.items():
        found = re.search(r'^\s*'+key+r':\s*(\d+)', text, re.M)
        if found is None or int(found.group(1)) != value:
            raise ValueError('uORB scalar differs: '+key)


def verify_wire(codec, packet, payload):
    """Assert the actual encoded fields and preserve strict-JSON expectations."""
    import struct
    message = codec.decode(packet)
    expected = payload.to_dict()
    for key, value in [('time_usec', payload.time_usec), ('frame_id', expected['frame_id']),
                       ('child_frame_id', expected['child_frame_id']), ('reset_counter', payload.reset_counter),
                       ('estimator_type', 2), ('quality', 0)]:
        if getattr(message, key) != value:
            raise ValueError('wire scalar differs: '+key)
    vectors = dict(position=[message.x, message.y, message.z], q=list(message.q),
                   velocity=[message.vx, message.vy, message.vz],
                   angular_velocity=[message.rollspeed, message.pitchspeed, message.yawspeed])
    for name, values in [('position', payload.position_ned_m), ('q', payload.orientation_ned_body_wxyz),
                         ('velocity', payload.velocity_body_m_s),
                         ('angular_velocity', payload.angular_velocity_body_rad_s)]:
        actual = vectors[name]
        if actual != [struct.unpack('<f', struct.pack('<f', value))[0] for value in values]:
            raise ValueError('wire float32 values differ: '+name)
    for name in ('pose_covariance', 'velocity_covariance'):
        values = getattr(message, name)
        if len(values) != 21 or not math.isnan(values[0]) or values[1:] != [0.0]*20:
            raise ValueError('unknown covariance wire sentinel differs: '+name)
    return dict(time_usec=message.time_usec, frame_id=message.frame_id, child_frame_id=message.child_frame_id,
                position=vectors['position'], q=vectors['q'], velocity=vectors['velocity'],
                angular_velocity=vectors['angular_velocity'],
                reset_counter=message.reset_counter, quality=message.quality, estimator_type=message.estimator_type,
                pose_covariance_first='NaN', velocity_covariance_first='NaN',
                covariance_sentinel_verified=True)


def module_absence_command(module):
    # Pinned ekf2_main's status silently returns0 for no instances. Its stop
    # command emits "not running" only if neither selector nor any instance
    # existed; a present instance emits "stopping" and must fail this check.
    # This probe never starts an estimator or commander.
    if module not in ('commander', 'ekf2'):
        raise ValueError('unsupported isolated module absence probe')
    return 'stop' if module == 'ekf2' else 'status'


def verify_module_absence(module, command, exit_code, text):
    if (command != module_absence_command(module)
            or (module == 'ekf2' and exit_code != 0)
            or re.search(r'\bstopping\b|\bmodule locked\b', text, re.I)
            or re.search(r'^(?:INFO|WARN)\s+\[(?:px4|ekf2)\]\s+not running\s*$', text, re.M) is None):
        raise RuntimeError('native module absence is not established: '+module)


def run_inner():
    """Called only in the no-network Docker container by the outer entry."""
    from rm27.perception.vision.localization.mavlink import MavlinkOdometryCodec, OdometryPayload

    work = Path('/work')
    (work/'bin').mkdir()
    for name in ['px4', 'px4-ver', 'px4-dataman', 'px4-mavlink', 'px4-listener', 'px4-shutdown',
                 'px4-commander', 'px4-ekf2']:
        (work/'bin'/name).symlink_to('/artifact/px4')
    instance = '73'  # Private /tmp and network namespace prevent host collisions.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
    startup = work/'startup.sh'
    startup.write_text('#!/bin/sh\nset -eu\n'
        '/work/bin/px4-ver --instance 73 all\n'
        '/work/bin/px4-dataman --instance 73 start -r > /work/dataman-start.txt 2>&1\n'
        'set +e\n'
        f'/work/bin/px4-mavlink --instance 73 start -u {port} -o 9 -t 127.0.0.1 -m minimal -r 1000 > /work/mavlink-start.txt 2>&1\n'
        'start_result=$?\n'
        'printf \"%s\\n\" \"$start_result\" > /work/mavlink-start-exit.txt\n'
        'exit 0\n')
    command = ['/work/bin/px4', '-d', '-i', instance, '-s', str(startup), '-w', str(work)]
    if os.environ.get('RM27_SITL_TRACE') == '1':
        command = ['/usr/bin/strace','-f','-o','/work/startup-trace.txt',
                   '-e','trace=clone,clone3,sched_setscheduler,sched_setparam,setpriority,prlimit64',*command]
    elif os.environ.get('RM27_SITL_DEBUG_WORKER') == '1':
        command = ['/usr/bin/gdb','-q','-nx','-batch','-ex','set confirm off',
                   '-ex','set pagination off','-ex','run','-ex','thread apply all bt 12',
                   '-ex','quit','--args',*command]
    laboratory_path = os.environ.get('RM27_SITL_LABORATORY_RECORDS')
    laboratory = load_laboratory_records(laboratory_path,
        enabled=os.environ.get('RM27_SITL_OBSERVER_ONLY_LABORATORY') == '1')
    scope = 'REAL_VIO_OBSERVER_ONLY_UNQUALIFIED_RECEPTION' if laboratory else 'SYNTHETIC_ODOMETRY_RECEPTION_ONLY'
    report = dict(status='FAIL', evidence_class='EMULATED / MOCK VERIFIED',
        verification_scope=scope, qualification='UNQUALIFIED' if laboratory else 'SYNTHETIC_INTERFACE_ONLY',
        control_use_permitted=False, arming='PROHIBITED_COMMANDER_NOT_STARTED',
        ekf_accepted='NOT_RUN', ekf_fused='NOT_RUN',
        vio_fusion='NOT_RUN', flight_qualification='NOT_RUN', hardware='NOT_RUN',
        px4_clock_time_sync='NOT_RUN; minimal lockstep clock is not driven',
        startup_command=command, network='Docker none; owned loopback only', cases=[])
    process = None
    log_path = work/'px4.log'
    try:
        codec = MavlinkOdometryCodec()
        report['codec'] = codec.provenance
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=work, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            ready = False
            for _ in range(40):
                if process.poll() is not None:
                    raise RuntimeError('PX4 exited before MAVLink startup')
                status = subprocess.run(['/work/bin/px4-mavlink', '--instance', instance, 'status'],
                    cwd=work, capture_output=True, text=True, timeout=2)
                # This archived PX4 routes client module output to its server
                # log. Verify the owned instance's exact bound port there.
                status_text = status.stdout + status.stderr + log_path.read_text()
                if status.returncode == 0 and f'transport protocol: UDP ({port},' in status_text:
                    ready = True; break
                time.sleep(.05)
            if not ready:
                raise RuntimeError('owned MAVLink instance did not become ready')
            start_exit = work/'mavlink-start-exit.txt'
            report['mavlink_start_command_exit_code'] = int(start_exit.read_text().strip()) if start_exit.exists() else None
            report['mavlink_startup_status'] = dict(exit_code=status.returncode, stdout=status.stdout)
            def verify_modules_stopped(phase):
                checks = {}
                for module in ('commander', 'ekf2'):
                    offset = log_path.stat().st_size
                    probe_command = module_absence_command(module)
                    checked = subprocess.run(['/work/bin/px4-'+module, '--instance', instance, probe_command],
                        cwd=work, capture_output=True, text=True, timeout=2)
                    native = log_path.read_bytes()[offset:].decode('utf-8', errors='replace')
                    text = checked.stdout + checked.stderr + native
                    name = 'module-status-'+module+'-'+phase+'.txt'
                    (work/name).write_text(text)
                    verify_module_absence(module, probe_command, checked.returncode, text)
                    checks[module] = dict(status='NOT_RUNNING', command_exit_code=checked.returncode,
                                          probe_command=probe_command, evidence=name, native_log_start_offset=offset,
                                          semantic='NATIVE_NOT_RUNNING_WITNESS; no module started')
                return checks
            if laboratory:
                report['modules_before_reception'] = verify_modules_stopped('before')
            cases = []
            if laboratory:
                for row in laboratory['records']:
                    payload = reconstruct_payload(row['payload'])
                    cases.append((row['case'], payload, row))
            else:
                for reset in (17, 18):
                    payload = OdometryPayload(time.time_ns()//1000,
                        (1.25, -2.5, .75), (math.sqrt(.5), 0., 0., math.sqrt(.5)),
                        (.1, .2, .3), (.01, .02, .03), reset,
                        dict(source='DECLARED_SYNTHETIC_INTERFACE_INPUT_NOT_VIO', clock='UNIX'))
                    cases.append(('reset-'+str(reset), payload, None))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.bind(('127.0.0.1', 0))
                for case, payload, source in cases:
                    packet = codec.encode(payload)
                    wire = verify_wire(codec, packet, payload)
                    (work/('odometry-'+case+'.mavlink')).write_bytes(packet)
                    for _ in range(5):
                        sender.sendto(packet, ('127.0.0.1', port)); time.sleep(.02)
                    native_log_offset = log_path.stat().st_size
                    received = subprocess.run(['/work/bin/px4-listener', '--instance', instance,
                        'vehicle_visual_odometry', '-n', '1'], cwd=work,
                        capture_output=True, text=True, timeout=4)
                    native_output = log_path.read_bytes()[native_log_offset:].decode('utf-8', errors='replace')
                    listener_output = received.stdout + received.stderr + native_output
                    (work/('odometry-'+case+'.txt')).write_text(listener_output)
                    if received.returncode != 0:
                        raise RuntimeError('uORB listener exited unsuccessfully')
                    expected = dict(position=payload.position_ned_m, q=payload.orientation_ned_body_wxyz,
                        velocity=payload.velocity_body_m_s, angular_velocity=payload.angular_velocity_body_rad_s,
                        reset_counter=payload.reset_counter)
                    if laboratory:
                        # PX4 LOCAL_FRD pose maps to vehicle_odometry POSE_FRAME_FRD=2.
                        expected.update(pose_frame=2, velocity_frame=3, quality=0,
                            position_variance=[None, 0., 0.], orientation_variance=[0., 0., 0.],
                            velocity_variance=[None, 0., 0.])
                    parse_received(listener_output, expected)
                    report['cases'].append(dict(status='PASS', case=case, reset_counter=payload.reset_counter,
                        packet_sha256=hashlib.sha256(packet).hexdigest(), payload=payload.to_dict(),
                        decoded_wire=wire, source=source,
                        message_received='PASS', ekf_accepted='NOT_RUN', ekf_fused='NOT_RUN',
                        listener_exit_code=received.returncode,
                        listener_evidence_source='CLIENT_OUTPUT_AND_OWNED_NATIVE_LOG_COMMAND_INTERVAL',
                        native_log_start_offset=native_log_offset,
                        time_semantic=('RECORDED_UPSTREAM_HEADER_WIRE_TIME_ONLY; PX4 clock mapping unqualified'
                                       if laboratory else 'UNIX wire timestamp; native time-sync conversion not qualified'),
                        observed_native_timestamps={key:int(re.search(r'^\s*'+key+r':\s*(\d+)', listener_output, re.M)[1])
                                                    for key in ('timestamp', 'timestamp_sample')},
                        covariance_semantic='unknown MAVLink sentinel; no uncertainty/fusion claim'))
            if laboratory:
                report['modules_after_reception'] = verify_modules_stopped('after')
            stop = subprocess.run(['/work/bin/px4-mavlink', '--instance', instance, 'stop-all'],
                                  cwd=work, capture_output=True, text=True, timeout=4)
            # dataman's standalone stop joins a poll waiting on the deliberately
            # stopped simulator clock. Native shutdown owns all module threads.
            # The archived daemon shutdown RPC exits before returning its
            # trailer to the client. Use its normal owned-process SIGINT entry
            # and audit the actual native return code, never invent client zero.
            os.killpg(process.pid, signal.SIGINT)
            report['stop'] = dict(mavlink_exit_code=stop.returncode, dataman='NATIVE_PROCESS_SHUTDOWN',
                                  shutdown_signal='SIGINT')
            report['process_exit_code'] = process.wait(timeout=8)
            if stop.returncode != 0 or report['process_exit_code'] != 0:
                raise RuntimeError('native stop/exit not clean')
            report['reception_status'] = 'PASS'
            if report.get('mavlink_start_command_exit_code') != 0:
                raise RuntimeError('MAVLink start command failed despite later receiver observation')
            report['status'] = 'PASS'
    except Exception as exc:
        report['failure'] = type(exc).__name__+': '+str(exc)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=4)
            report['cleanup_signal_exit_code'] = process.returncode
        report['owned_native_process_absent'] = process is None or process.poll() is not None
        report['output_sha256'] = {p.name:digest(p) for p in work.iterdir() if p.is_file()}
        if log_path.exists():
            report['output_sha256']['px4.log'] = digest(log_path)
        (work/'sitl-result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    artifact = parser.add_mutually_exclusive_group(required=True)
    artifact.add_argument('--px4-source', type=Path)
    artifact.add_argument('--artifact-lock', type=Path,
                          help='hash-locked portable executable/version/dependencies; no source Git operations')
    parser.add_argument('--image', required=True, help='existing immutable Docker image ID, sha256:...')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--laboratory-records', type=Path,
                        help='unqualified recorded VIO payloads; requires explicit observer-only opt-in')
    parser.add_argument('--observer-only-laboratory', action='store_true',
                        help='only isolated receiver transport, never operational health, fusion or control')
    diagnostics=parser.add_mutually_exclusive_group()
    diagnostics.add_argument('--trace-startup', action='store_true', help='bounded thread/scheduler diagnostic only')
    diagnostics.add_argument('--debug-worker', action='store_true', help='bounded owned-process thread backtrace')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', args.image):
        raise ValueError('immutable Docker image ID required')
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    laboratory = load_laboratory_records(args.laboratory_records, enabled=args.observer_only_laboratory)
    source = None
    artifact_identity = None
    if args.artifact_lock is not None:
        artifact_identity = load_artifact_lock(args.artifact_lock, args.image)
        binary = Path(artifact_identity['binary']['path']).resolve()
        version = Path(artifact_identity['build_version']['path']).resolve()
        head = artifact_identity['source_head']
    else:
        source = args.px4_source.resolve()
        binary = source/'build/px4_sitl_default/bin/px4'
        version = source/'build/px4_sitl_default/src/lib/version/build_git_version.h'
        if not binary.is_file() or not version.is_file():
            raise FileNotFoundError('existing SITL binary and build Git version required')
        head = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain'], text=True)
        if dirty:
            raise ValueError('clean source and matching build version required')
    build_head = re.search(r'PX4_GIT_VERSION_STR "([0-9a-f]{40})"', version.read_text())
    if not build_head or build_head[1] != head:
        raise ValueError('matching build version required')
    dependencies_check = subprocess.run(['/lib64/ld-linux-x86-64.so.2','--list',str(binary)],
        capture_output=True,text=True,timeout=5)
    if dependencies_check.returncode != 0 or 'not found' in dependencies_check.stdout:
        raise RuntimeError('SITL dynamic dependency resolution failed: '+dependencies_check.stderr)
    resolved = {}
    for line in dependencies_check.stdout.splitlines():
        candidate = line.split('=>',1)[-1].strip().split(' ',1)[0]
        if candidate.startswith('/') and Path(candidate).is_file():
            resolved[candidate] = digest(candidate)
    if not resolved:
        raise RuntimeError('resolved SITL dynamic dependencies unavailable')
    if artifact_identity is not None and resolved != artifact_identity['dynamic_dependencies_sha256']:
        raise ValueError('resolved dynamic dependency set differs from artifact lock')
    repository = Path(__file__).resolve().parents[2]
    import pymavlink
    dependencies = Path(pymavlink.__file__).resolve().parents[1]
    local_sources = [Path(__file__).resolve()] + [repository/'rm27/perception/vision/localization'/name
        for name in ('mavlink.py', 'state_output.py', 'schema.py', 'experiments/trajectory.py')]
    input_code_sha256 = {str(path):digest(path) for path in local_sources}
    input_files_sha256 = {str(binary):digest(binary), str(version):digest(version), **resolved}
    if args.artifact_lock is not None:
        input_files_sha256[str(args.artifact_lock.resolve())] = digest(args.artifact_lock)
    if laboratory:
        input_files_sha256[str(args.laboratory_records.resolve())] = digest(args.laboratory_records)
    output.mkdir(parents=True)
    name = 'rm27-vio-sitl-'+uuid.uuid4().hex[:12]
    mounts = [('/usr','/usr','ro'), ('/etc/alternatives','/etc/alternatives','ro'), ('/opt/ros/jazzy/opt','/opt/ros/jazzy/opt','ro'),
              (str(repository),'/repo','ro'), (str(binary),'/artifact/px4','ro'),
              (str(dependencies),'/dependencies','ro'), (str(output),'/work','rw')]
    if laboratory:
        mounts.append((str(args.laboratory_records.resolve()), '/artifact/laboratory-records.json', 'ro'))
    command = ['docker','run','--name',name,'--user',str(os.getuid())+':'+str(os.getgid()),
        '--network','none','--read-only','--cap-drop','ALL',
        '--security-opt','no-new-privileges','--cpus','1','--memory','1g',
        '--tmpfs','/tmp:rw,nosuid,nodev,size=32m','--workdir','/work',
        '--env','PYTHONDONTWRITEBYTECODE=1','--env','PYTHONPATH=/repo:/dependencies']
    for host, container, mode in mounts:
        spec=f'type=bind,src={host},dst={container}'
        command += ['--mount',spec+(',readonly' if mode == 'ro' else '')]
    if args.trace_startup:
        command += ['--env','RM27_SITL_TRACE=1']
    elif args.debug_worker:
        command += ['--env','RM27_SITL_DEBUG_WORKER=1']
    if laboratory:
        command += ['--env', 'RM27_SITL_LABORATORY_RECORDS=/artifact/laboratory-records.json',
                    '--env', 'RM27_SITL_OBSERVER_ONLY_LABORATORY=1']
    command += [args.image,'/usr/bin/python3','/repo/tools/schurvins/verify_sitl.py','--inner']
    report = dict(status='FAIL', evidence_class='EMULATED / MOCK VERIFIED',
        verification_scope=('REAL_VIO_OBSERVER_ONLY_UNQUALIFIED_RECEPTION'
                            if laboratory else 'SYNTHETIC_ODOMETRY_RECEPTION_ONLY'),
        qualification='UNQUALIFIED' if laboratory else 'SYNTHETIC_INTERFACE_ONLY',
        control_use_permitted=False, ekf_accepted='NOT_RUN', ekf_fused='NOT_RUN',
        source=str(source) if source else None, source_head=head,
        source_clean_before=True if source else 'NOT_APPLICABLE_PORTABLE_LOCK',
        artifact_lock=str(args.artifact_lock.resolve()) if args.artifact_lock else None,
        artifact_lock_sha256=digest(args.artifact_lock) if args.artifact_lock else None,
        laboratory_records_sha256=digest(args.laboratory_records) if laboratory else None,
        build_version_sha256=digest(version), binary=str(binary), binary_sha256=digest(binary),
        verifier_sha256=digest(__file__), image_id=args.image, command=command,
        resolved_dynamic_dependencies_sha256=resolved, input_code_sha256=input_code_sha256,
        input_files_sha256=input_files_sha256,
        startup_and_exit='NOT_RUN', odometry_reception='NOT_RUN', vio_fusion='NOT_RUN')
    try:
        run = subprocess.run(command, capture_output=True, text=True, timeout=60 if laboratory else 35)
        report['container_exit_code'] = run.returncode
        report['container_stdout'] = run.stdout[-16_384:]
        report['container_stderr'] = run.stderr[-16_384:]
        inner = output/'sitl-result.json'
        if inner.is_file():
            report['result_sha256'] = digest(inner)
            result = json.loads(inner.read_text())
            stop = result.get('stop', {})
            report['startup_and_exit'] = ('PASS' if result.get('process_exit_code') == 0
                and result.get('mavlink_start_command_exit_code') == 0
                and stop.get('mavlink_exit_code') == 0 and stop.get('shutdown_signal') == 'SIGINT'
                and stop.get('dataman') == 'NATIVE_PROCESS_SHUTDOWN'
                and result.get('owned_native_process_absent') is True else 'FAIL')
            report['odometry_reception'] = result.get('reception_status',result.get('status','FAIL'))
            if result.get('failure'):
                report['failure'] = result['failure']
            report['status'] = 'PASS' if run.returncode == 0 and result.get('status') == 'PASS' else 'FAIL'
    except subprocess.TimeoutExpired:
        report['failure'] = 'owned Docker verification timed out after'+str(60 if laboratory else 35)+'s'
    finally:
        cleanup = subprocess.run(['docker','rm','-f',name], capture_output=True, text=True, timeout=10)
        absent = subprocess.run(['docker','container','ls','--all','--format','{{.Names}}',
                                 '--filter','name=^/'+name+'$'], capture_output=True, text=True, timeout=5)
        report['owned_container_absent'] = absent.returncode == 0 and not absent.stdout.strip()
        report['source_clean_after'] = (not subprocess.check_output(
            ['git','-C',str(source),'status','--porcelain'], text=True)) if source else 'NOT_APPLICABLE_PORTABLE_LOCK'
        report['binary_unchanged'] = digest(binary) == report['binary_sha256']
        report['input_code_unchanged'] = all(Path(path).is_file() and digest(path) == expected
                                             for path,expected in input_code_sha256.items())
        report['input_files_unchanged'] = all(Path(path).is_file() and digest(path) == expected
                                             for path,expected in input_files_sha256.items())
        if (not report['owned_container_absent'] or report['source_clean_after'] is False
                or not report['binary_unchanged'] or not report['input_code_unchanged']
                or not report['input_files_unchanged']):
            report['status'] = 'FAIL'
        (output/'verification.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        receipt = dict(schema_version=1, status=report['status'],
            evidence_class='EMULATED / MOCK VERIFIED',
            verification_scope=report['verification_scope'], qualification=report['qualification'],
            control_use_permitted=False, ekf_accepted='NOT_RUN', ekf_fused='NOT_RUN', vio_fusion='NOT_RUN',
            artifacts_sha256={str(path):digest(path) for path in output.iterdir() if path.is_file()},
            code_sha256=input_code_sha256,
            input_files_sha256=input_files_sha256,
            execution=dict(command=[sys.executable,str(Path(__file__).resolve()),*(sys.argv[1:] if argv is None else argv)],
                           completed=True,exit_code=0 if report['status'] == 'PASS' else 1),
            cleanup=dict(absence_verified=report['owned_container_absent']))
        if report['status'] != 'PASS':
            receipt['reason'] = report.get('failure', 'SITL verification or input-integrity gate failed')
        (output/'receipt.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['status','startup_and_exit','odometry_reception','vio_fusion','owned_container_absent']}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(run_inner() if sys.argv[1:] == ['--inner'] else main())
