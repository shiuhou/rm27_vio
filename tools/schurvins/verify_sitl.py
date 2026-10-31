#!/usr/bin/env python3
"""Verify an existing PX4 SITL artifact's ODOMETRY reception, never VIO fusion.

Uses a minimal startup script (MAVLink only), an owned Docker network namespace
with no external interfaces, read-only executable/libraries and an output-only
writable directory. No estimator, commander, simulator or device driver starts.
The archived PX4 source/build is inspected and mounted read-only, never changed.
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


def parse_received(text, expected):
    """Check actual uORB listener fields; listener exit alone proves nothing."""
    if 'TOPIC: vehicle_visual_odometry' not in text:
        raise ValueError('vehicle_visual_odometry topic not observed')
    for key, values in expected.items():
        if key == 'reset_counter':
            continue
        found = re.search(r'^\s*'+re.escape(key)+r':\s*\[([^\]]+)\]', text, re.M)
        if found is None:
            raise ValueError('missing uORB vector: '+key)
        actual = [float(x.strip()) for x in found.group(1).split(',')]
        if len(actual) != len(values) or any(not math.isfinite(x) or abs(x-y) > 2e-5
                                             for x, y in zip(actual, values)):
            raise ValueError('uORB vector differs: '+key)
    for key, value in [('pose_frame', 1), ('velocity_frame', 3), ('reset_counter', expected['reset_counter'])]:
        found = re.search(r'^\s*'+key+r':\s*(\d+)', text, re.M)
        if found is None or int(found.group(1)) != value:
            raise ValueError('uORB scalar differs: '+key)


def run_inner():
    """Called only in the no-network Docker container by the outer entry."""
    from rm27.perception.vision.localization.mavlink import MavlinkOdometryCodec, OdometryPayload

    work = Path('/work')
    (work/'bin').mkdir()
    for name in ['px4', 'px4-ver', 'px4-mavlink', 'px4-listener', 'px4-shutdown']:
        (work/'bin'/name).symlink_to('/artifact/px4')
    instance = '73'  # Private /tmp and network namespace prevent host collisions.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
    startup = work/'startup.sh'
    startup.write_text('#!/bin/sh\nset -eu\n'
        '/work/bin/px4-ver --instance 73 all\n'
        f'/work/bin/px4-mavlink --instance 73 start -u {port} -o 9 -t 127.0.0.1 -m minimal -r 1000\n')
    command = ['/work/bin/px4', '-d', '-i', instance, '-s', str(startup), '-w', str(work)]
    report = dict(status='FAIL', evidence_class='EMULATED / MOCK VERIFIED',
        verification_scope='SYNTHETIC_ODOMETRY_RECEPTION_ONLY',
        vio_fusion='NOT_RUN', flight_qualification='NOT_RUN', hardware='NOT_RUN',
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
                if status.returncode == 0 and ('UDP' in status.stdout or 'udp' in status.stdout):
                    ready = True; break
                time.sleep(.05)
            if not ready:
                raise RuntimeError('owned MAVLink instance did not become ready')
            report['mavlink_startup_status'] = dict(exit_code=status.returncode, stdout=status.stdout)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.bind(('127.0.0.1', 0))
                for reset in (17, 18):
                    payload = OdometryPayload(time.time_ns()//1000,
                        (1.25, -2.5, .75), (math.sqrt(.5), 0., 0., math.sqrt(.5)),
                        (.1, .2, .3), (.01, .02, .03), reset,
                        dict(source='DECLARED_SYNTHETIC_INTERFACE_INPUT_NOT_VIO', clock='UNIX'))
                    packet = codec.encode(payload)
                    for _ in range(5):
                        sender.sendto(packet, ('127.0.0.1', port)); time.sleep(.02)
                    received = subprocess.run(['/work/bin/px4-listener', '--instance', instance,
                        'vehicle_visual_odometry', '-n', '1'], cwd=work,
                        capture_output=True, text=True, timeout=4)
                    (work/f'odometry-reset-{reset}.txt').write_text(received.stdout+received.stderr)
                    if received.returncode != 0:
                        raise RuntimeError('uORB listener exited unsuccessfully')
                    expected = dict(position=payload.position_ned_m, q=payload.orientation_ned_body_wxyz,
                        velocity=payload.velocity_body_m_s, angular_velocity=payload.angular_velocity_body_rad_s,
                        reset_counter=reset)
                    # reset_counter is scalar, checked separately by parser.
                    parse_received(received.stdout, {k:v for k,v in expected.items() if k != 'reset_counter'}
                                   | {'reset_counter':reset})
                    report['cases'].append(dict(status='PASS', reset_counter=reset,
                        packet_sha256=hashlib.sha256(packet).hexdigest(), payload=payload.to_dict(),
                        listener_exit_code=received.returncode,
                        time_semantic='UNIX wire timestamp; native time-sync conversion not qualified',
                        covariance_semantic='unknown MAVLink sentinel; no uncertainty/fusion claim'))
            stop = subprocess.run(['/work/bin/px4-mavlink', '--instance', instance, 'stop-all'],
                                  cwd=work, capture_output=True, text=True, timeout=4)
            shutdown = subprocess.run(['/work/bin/px4-shutdown', '--instance', instance],
                                      cwd=work, capture_output=True, text=True, timeout=4)
            report['stop'] = dict(mavlink_exit_code=stop.returncode, shutdown_exit_code=shutdown.returncode)
            report['process_exit_code'] = process.wait(timeout=8)
            if stop.returncode != 0 or shutdown.returncode != 0 or report['process_exit_code'] != 0:
                raise RuntimeError('native stop/exit not clean')
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
        report['output_sha256'] = {p.name:digest(p) for p in work.glob('*.txt')}
        if log_path.exists():
            report['output_sha256']['px4.log'] = digest(log_path)
        (work/'sitl-result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-source', type=Path, required=True)
    parser.add_argument('--image', required=True, help='existing immutable Docker image ID, sha256:...')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', args.image):
        raise ValueError('immutable Docker image ID required')
    source, output = args.px4_source.resolve(), args.output.resolve()
    binary = source/'build/px4_sitl_default/bin/px4'
    version = source/'build/px4_sitl_default/src/lib/version/build_git_version.h'
    if not binary.is_file() or not version.is_file():
        raise FileNotFoundError('existing SITL binary and build Git version required')
    if output.exists():
        raise FileExistsError(output)
    head = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain'], text=True)
    build_head = re.search(r'PX4_GIT_VERSION_STR "([0-9a-f]{40})"', version.read_text())
    if dirty or not build_head or build_head[1] != head:
        raise ValueError('clean source and matching build version required')
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
    repository = Path(__file__).resolve().parents[2]
    import pymavlink
    dependencies = Path(pymavlink.__file__).resolve().parents[1]
    output.mkdir(parents=True)
    name = 'rm27-vio-sitl-'+uuid.uuid4().hex[:12]
    mounts = [('/usr','/usr','ro'), ('/opt/ros/jazzy/opt','/opt/ros/jazzy/opt','ro'),
              (str(repository),'/repo','ro'), (str(binary),'/artifact/px4','ro'),
              (str(dependencies),'/dependencies','ro'), (str(output),'/work','rw')]
    command = ['docker','run','--name',name,'--network','none','--read-only','--cap-drop','ALL',
        '--security-opt','no-new-privileges','--cpus','1','--memory','1g',
        '--tmpfs','/tmp:rw,nosuid,nodev,size=32m','--workdir','/work',
        '--env','PYTHONDONTWRITEBYTECODE=1','--env','PYTHONPATH=/repo:/dependencies']
    for host, container, mode in mounts:
        spec=f'type=bind,src={host},dst={container}'
        command += ['--mount',spec+(',readonly' if mode == 'ro' else '')]
    command += [args.image,'/usr/bin/python3','/repo/tools/schurvins/verify_sitl.py','--inner']
    report = dict(status='FAIL', evidence_class='EMULATED / MOCK VERIFIED',
        verification_scope='SYNTHETIC_ODOMETRY_RECEPTION_ONLY',
        source=str(source), source_head=head, source_clean_before=True,
        build_version_sha256=digest(version), binary=str(binary), binary_sha256=digest(binary),
        verifier_sha256=digest(__file__), image_id=args.image, command=command,
        resolved_dynamic_dependencies_sha256=resolved,
        startup_and_exit='NOT_RUN', odometry_reception='NOT_RUN', vio_fusion='NOT_RUN')
    try:
        run = subprocess.run(command, capture_output=True, text=True, timeout=35)
        report['container_exit_code'] = run.returncode
        report['container_stdout'] = run.stdout[-16_384:]
        report['container_stderr'] = run.stderr[-16_384:]
        inner = output/'sitl-result.json'
        if inner.is_file():
            report['result_sha256'] = digest(inner)
            result = json.loads(inner.read_text())
            report['startup_and_exit'] = ('PASS' if result.get('process_exit_code') == 0
                                         and result.get('owned_native_process_absent') is True else 'FAIL')
            report['odometry_reception'] = result.get('status','FAIL')
            report['status'] = 'PASS' if run.returncode == 0 and result.get('status') == 'PASS' else 'FAIL'
    except subprocess.TimeoutExpired:
        report['failure'] = 'owned Docker verification timed out after35s'
    finally:
        cleanup = subprocess.run(['docker','rm','-f',name], capture_output=True, text=True, timeout=10)
        absent = subprocess.run(['docker','container','ls','--all','--format','{{.Names}}',
                                 '--filter','name=^/'+name+'$'], capture_output=True, text=True, timeout=5)
        report['owned_container_absent'] = absent.returncode == 0 and not absent.stdout.strip()
        report['source_clean_after'] = not subprocess.check_output(
            ['git','-C',str(source),'status','--porcelain'], text=True)
        report['binary_unchanged'] = digest(binary) == report['binary_sha256']
        if not report['owned_container_absent'] or not report['source_clean_after'] or not report['binary_unchanged']:
            report['status'] = 'FAIL'
        (output/'verification.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        receipt = dict(schema_version=1, status=report['status'],
            evidence_class='EMULATED / MOCK VERIFIED',
            verification_scope='SYNTHETIC_ODOMETRY_RECEPTION_ONLY', vio_fusion='NOT_RUN',
            artifacts_sha256={str(path):digest(path) for path in output.iterdir() if path.is_file()},
            code_sha256={str(Path(__file__).resolve()):digest(__file__),
                str(repository/'rm27/perception/vision/localization/mavlink.py'):
                    digest(repository/'rm27/perception/vision/localization/mavlink.py')},
            execution=dict(command=[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],
                           completed=True,exit_code=0 if report['status'] == 'PASS' else 1),
            cleanup=dict(absence_verified=report['owned_container_absent']))
        (output/'receipt.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['status','startup_and_exit','odometry_reception','vio_fusion','owned_container_absent']}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(run_inner() if sys.argv[1:] == ['--inner'] else main())
