"""Independent EuRoC mono/stereo comparison; original OpenVINS gate unchanged.

Reuses verified transport, native state reconciliation and fixed-scale SE3
evaluation. This entry does not confer VIO-P, RK3576 or operational validity.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import uuid
import zipfile

from .common import checked_file, cleanup_container, file_hash, read_json, require, write_json
from .openvins import PIN
from .openvins_runtime import StreamOrder, create_output, lines, normalize, state_row
from tools.openvins.comparison_transport import events_from, image_pixels
from tools.openvins.native_accuracy import associate
from tools.schurvins.evaluate import evaluate_rows, read_reference


class ComparisonDataset:
    """Explicit sequence/mode input, without weakening EurocVioDataset."""
    def __init__(self, archive, bag, sequence, mode, source_sha256):
        self.archive, self.bag = Path(archive).resolve(), Path(bag).resolve()
        self.sequence, self.mode = sequence, mode
        require(isinstance(sequence, str) and bool(sequence), 'SEQUENCE_INVALID', 'explicit EuRoC name required')
        v = read_json(self.bag/'source_verification.json')
        require(v.get('purpose') == 'EUROC_PUBLIC_COMPARISON' and v.get('sequence') == sequence
                and v.get('mode') == mode, 'COMPARISON_INPUT_MISMATCH', 'sequence/mode differ')
        require(v.get('readback_exact') is True and v.get('truth_included') is False,
                'COMPARISON_INPUT_UNVERIFIED', 'complete sensor-only bag readback required')
        require(v.get('source_sha256') == source_sha256, 'COMPARISON_SOURCE_MISMATCH', 'explicit source hash')
        checked_file(self.archive, source_sha256)
        self.provenance = v
        self.manifest_sha256 = file_hash(self.bag/'source_verification.json')
        self.verify_bag()
        expected = {'/cam0/image_raw': 'sensor_msgs/msg/Image', '/imu0': 'sensor_msgs/msg/Imu'}
        if mode == 'stereo':
            expected['/cam1/image_raw'] = 'sensor_msgs/msg/Image'
        dbs = list(self.bag.glob('*.db3'))
        require(len(dbs) == 1, 'BAG_FORMAT_UNSUPPORTED', 'single verified sqlite3 bag')
        with sqlite3.connect(dbs[0].as_uri() + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('SELECT name, type FROM topics'))
        require(topics == expected, 'TRUTH_ISOLATION_FAILED', 'exact camera/IMU topic set required')
        with zipfile.ZipFile(self.archive) as z:
            streams, _ = events_from(z, mode)
            def original_rows(sensor):
                return list(csv.reader(line for line in z.read('mav0/' + sensor + '/data.csv').decode().splitlines()
                                       if line.strip() and not line.lstrip().startswith('#')))
            self.camera, self.imu = original_rows('cam0'), original_rows('imu0')
        for sensor, rows in streams.items():
            require(len(rows) == v['verified_counts'][sensor], 'COMPARISON_COUNT_MISMATCH', sensor)
        self.streams = streams
        self.camera_ns = [r[0] for r in streams['cam0']]
        self.imu_ns = [r[0] for r in streams['imu0']]

    def verify_bag(self):
        """Every mounted file must belong to the sensor-only frozen bag."""
        artifacts = self.provenance.get('artifacts_sha256', {})
        db_names = [name for name in artifacts if name.endswith('.db3')]
        require(len(db_names) == 1 and set(artifacts) == {'metadata.yaml', db_names[0]},
                'BAG_ARTIFACT_INVALID', 'one frozen database and metadata required')
        for name in artifacts:
            require(Path(name).name == name, 'BAG_ARTIFACT_INVALID', 'bag-local file names')
        entries = list(self.bag.iterdir())
        require({p.name for p in entries} == set(artifacts) | {'source_verification.json'}
                and all(p.is_file() and not p.is_symlink() for p in entries),
                'TRUTH_ISOLATION_FAILED', 'only frozen sensor bag files may be mounted')
        checked_file(self.bag/'source_verification.json', self.manifest_sha256)
        for name, digest in artifacts.items():
            checked_file(self.bag/name, digest)


def set_sequence_clock(record, sequence):
    """Name this explicit dataset clock while preserving raw integer values."""
    clock = 'EuRoC:' + sequence + ':sensor_time'
    for key in ['source_timestamp', 'backend_timestamp']:
        record[key]['clock_domain'] = clock
    record['localization_fields']['source_time']['clock_domain'] = clock
    return record


def verify_second_camera(raw, dataset):
    if dataset.mode != 'stereo':
        return None
    observed = lines(raw/'cam1.jsonl')
    expected = dataset.streams['cam1']
    order = StreamOrder()
    for r in observed:
        order.push('cam1', r['state_ns'])
    require([r['state_ns'] for r in observed] == [ns for ns, _ in expected],
            'CAM1_DELIVERY_MISMATCH', 'complete exactly synchronized right-camera observer stream')
    import hashlib
    with zipfile.ZipFile(dataset.archive) as z:
        for row, (_, filename) in zip(observed, expected):
            pixels = image_pixels(z, 'cam1', filename)
            require((row['height'], row['width']) == pixels.shape and row['encoding'] == 'mono8'
                    and row['step'] == row['width'] and row['is_bigendian'] == 0
                    and row['frame_id'] == 'cam1'
                    and row['pixels_sha256'] == hashlib.sha256(pixels.tobytes()).hexdigest(),
                    'CAM1_PIXELS_MISMATCH', 'observer right image versus source')
    return dict(count=len(observed), first_ns=observed[0]['state_ns'], last_ns=observed[-1]['state_ns'],
                scope='observer receipt only; estimator image callback coverage not instrumented')


def normalize_comparison(root, dataset, install, run_id):
    # The preserved adapter validates left pixels/IMU values and timestamps.
    # This independent entry also checks the input frame envelopes it records.
    for sensor in ['cam0', 'imu0']:
        observed = lines(root/'raw'/(sensor+'.jsonl'))
        require(all(r.get('frame_id') == sensor for r in observed),
                'INPUT_FRAME_INVALID', sensor+' observer frame identity')
        if sensor == 'cam0':
            require(all(r.get('is_bigendian') == 0 for r in observed),
                    'CAMERA_ENCODING_INVALID', 'little-endian mono8 envelope required')
    right = verify_second_camera(root/'raw', dataset)
    records, evidence = normalize(root, dataset, install, run_id)
    for record in records:
        set_sequence_clock(record, dataset.sequence)
    trajectory = read_json(root/'online/trajectory.json')
    trajectory['records'] = records
    write_json(root/'online/trajectory.json', trajectory)
    propagated = read_json(root/'online/propagated.json')
    for record in propagated['records']:
        record['localization_fields']['source_time']['clock_domain'] = 'EuRoC:' + dataset.sequence + ':sensor_time'
    write_json(root/'online/propagated.json', propagated)
    evidence.update(sequence=dataset.sequence, mode=dataset.mode, right_camera=right,
                    historical_v101_mono_acceptance='not evaluated by this independent comparison',
                    native_reset_count=None, estimator_camera_ingestion_count=None)
    write_json(root/'online/evidence.json', evidence)
    write_json(root/'observations.json', dict(
        initialization_proxy=dict(status='CODE_SUPPORTED_PROXY',
            first_visual_pose_camera_index=records[0]['source_frame_id'],
            first_visual_pose_image_ns=records[0]['source_timestamp']['raw_value'],
            sensor_time_from_first_image_s=(records[0]['source_timestamp']['raw_value']-dataset.camera_ns[0])/1e9,
            scope='first published visual pose after native initialized gate; no instrumented transition timestamp'),
        native_reset_count=None, tracking_loss_observations=[], tracking_loss_detection='NOT_RUN',
        native_tail_seen=None, camera_lag_at_info_receipt=None))
    return records, evidence


def evaluate_comparison(root, dataset, records):
    reference = read_reference(dataset.archive)
    poses = lines(root/'raw/pose.jsonl')
    states = [state_row(line) for line in (root/'raw/state_estimate.txt').read_text().splitlines()
              if line and not line.startswith('#')]
    camera_ids = associate(dataset.camera_ns, [r['image_ns'] for r in states], 6000)
    require(len(poses) == len(states), 'RAW_STATE_COUNT_MISMATCH', 'raw poses versus native text states')
    # The existing adapter preserves the exact source camera stamp after
    # validating native-text association. Report its rounding residual separately.
    raw = [dict(state_ns=p['state_ns'], image_ns=dataset.camera_ns[index], position=p['position'],
                quaternion_xyzw=p['quaternion_xyzw'], source_frame_id=index, epoch=0)
           for p, index in zip(poses, camera_ids)]
    raw_score = evaluate_rows(raw, reference)
    score = evaluate_rows(records, reference)
    require(score == raw_score, 'ADAPTER_METRICS_DIFFER', 'same-run raw ROS pose versus OnlinePoseRecord')
    score.update(same_run_metrics_exact=True, raw_pose_source='raw/pose.jsonl',
                 projection_source='online/trajectory.json', source_sha256=file_hash(root/'online/trajectory.json'),
                 camera_association=dict(tolerance_ns=6000,
                     max_native_text_residual_ns=max(abs(dataset.camera_ns[i]-s['image_ns'])
                                                    for i, s in zip(camera_ids, states)),
                     policy='original camera ID and source timestamp; native-text residual retained separately'))
    write_json(root/'evaluation.json', score)
    write_json(root/'native-evaluation.json', raw_score)
    require(score['scored_epoch_count'] > 0, 'TRAJECTORY_NOT_EVALUABLE', 'no scored visual trajectory')
    return score


def command_for(root, dataset, install, config, container):
    repo = Path(__file__).resolve().parents[5]
    return ['docker', 'run', '--rm', '--name', container, '--network', 'none', '--cpus', '4', '--memory', '12g',
            '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'VIO_MODE=' + dataset.mode,
            '-e', 'VIO_RATE=' + str(float(config['rate'])),
            '-v', f'{repo}:/repo:ro', '-v', f'{install["source"]}:/upstream:ro',
            '-v', f'{install["workspace"]}:/ws:ro', '-v', f'{dataset.bag}:/data:ro',
            '-v', f'{root/"raw"}:/out', install['image'], 'bash', '/repo/tools/openvins/run_comparison_ros2.sh']


def run(config):
    result = dict(status='REFUSED', execution_started=False, stage_result='PARTIAL', config=config)
    root = None
    try:
        keys = {'schema_version', 'purpose', 'run_id', 'backend', 'archive', 'archive_sha256', 'bag',
                'installation', 'output', 'sequence', 'mode', 'rate'}
        require(set(config) == keys and config['schema_version'] == 1 and config['backend'] == 'OpenVINS'
                and config['purpose'] == 'EUROC_PUBLIC_COMPARISON', 'COMPARISON_CONFIG_INVALID', 'explicit comparison v1')
        require(config['mode'] in ('mono', 'stereo') and type(config['rate']) in (int, float)
                and config['rate'] in (.5, 1.), 'COMPARISON_CONFIG_INVALID', 'mono/stereo and rate .5/1 required')
        dataset = ComparisonDataset(config['archive'], config['bag'], config['sequence'], config['mode'], config['archive_sha256'])
        install = read_json(config['installation'])
        require(install['commit'] == PIN, 'BACKEND_VERSION_UNSUPPORTED', PIN)
        require(install['image'].startswith('sha256:') and len(install['image']) == 71,
                'IMAGE_UNPINNED', 'immutable image required')
        for path, digest in install['files_sha256'].items():
            checked_file(path, digest)
        binary = Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
        checked_file(binary, install['binary_sha256'])
        output = Path(config['output']).resolve()
        for protected in [dataset.bag, Path(install['workspace']).resolve(), Path(install['source']).resolve()]:
            require(output != protected and protected not in output.parents,
                    'OUTPUT_INVALID', 'output must be outside frozen bag/source/workspace')
        root = create_output(config['output'])
        container = 'rm27-openvins-comparison-' + uuid.uuid4().hex
        command = command_for(root, dataset, install, config, container)
        repo = Path(__file__).resolve().parents[5]
        sources = [Path(__file__), repo/'tools/openvins/run_comparison_ros2.sh',
                   repo/'tools/openvins/observe_comparison_ros2.py', repo/'tools/openvins/comparison_transport.py',
                   repo/'tools/openvins/comparison_qos.yaml',
                   repo/'tools/openvins/sample_comparison_resources.py',
                   repo/'tools/openvins/asl_transport.py', repo/'tools/schurvins/ros_transport.py',
                   repo/'tools/schurvins/evaluate.py', repo/'tools/openvins/native_accuracy.py',
                   Path(__file__).with_name('openvins.py'), Path(__file__).with_name('online.py'),
                   Path(__file__).with_name('trajectory.py'), Path(__file__).parent.parent/'schema.py',
                   Path(__file__).with_name('openvins_runtime.py'), Path(__file__).with_name('common.py')]
        hashes = {str(p): file_hash(p) for p in sources}
        result.update(status='EXECUTING', execution_started=True, command=command, installation=install,
                      dataset=dataset.provenance, runtime_sources_sha256=hashes, final_optimized_trajectory=None,
                      transport_protocol='explicit-reliable-publisher-and-observer-paused-match-v1',
                      qualification='independent public-data comparison; original VIO-P acceptance unchanged')
        write_json(root/'run.json', result)
        with (root/'docker.log').open('w') as log:
            try:
                completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=900)
            except BaseException as exc:
                result.update(status='FAILED', failure_code='EXECUTION_INTERRUPTED',
                              failure_reason=type(exc).__name__)
                raise
            finally:
                result['container_cleanup'] = cleanup_container(container, log)
        result['exit_code'] = completed.returncode
        require(completed.returncode == 0, 'BACKEND_EXECUTION_FAILED', 'retained docker/raw logs')
        require(result['container_cleanup']['absence_verified'], 'CONTAINER_CLEANUP_UNVERIFIED',
                'owned replay container absence not verified; outputs may still change')
        records, evidence = normalize_comparison(root, dataset, install, config['run_id'])
        evaluate_comparison(root, dataset, records)
        for path, digest in {**install['files_sha256'], **hashes}.items():
            checked_file(path, digest)
        checked_file(binary, install['binary_sha256'])
        checked_file(dataset.archive, config['archive_sha256'])
        dataset.verify_bag()
        result.update(status='EXECUTION_COMPLETED', stage_result='COMPARISON_RUNTIME_AND_METRIC_PASS',
                      online_trajectory='online/trajectory.json', evaluation='evaluation.json', evidence=evidence)
    except (ValueError, KeyError, OSError, sqlite3.Error, subprocess.SubprocessError) as exc:
        if 'failure_code' not in result:
            result.update(status='FAILED' if result['execution_started'] else 'REFUSED',
                          failure_code=getattr(exc, 'code', 'COMPARISON_RUNTIME_ERROR'), failure_reason=str(exc))
    finally:
        if root is not None:
            if result.get('container_cleanup', {}).get('absence_verified') is not True:
                result['artifact_finalization'] = 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'
            else:
                result['output_sha256'] = {str(p.relative_to(root)): file_hash(p) for p in root.rglob('*')
                                          if p.is_file() and p.name != 'run.json'}
            write_json(root/'run.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    report = run(read_json(args.config))
    print(json.dumps({k: report.get(k) for k in ['status', 'stage_result', 'failure_code', 'failure_reason']}, indent=2))
    raise SystemExit(0 if report['status'] == 'EXECUTION_COMPLETED' else 1)
