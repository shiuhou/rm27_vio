"""Independent pinned SchurVINS public replay; never dispatches OpenVINS.

The estimator container sees sensor-only inputs. Truth is read on the host after
execution. A numerical replay PASS is not tracking, hardware or flight approval.
"""
import argparse
import bisect
from decimal import Decimal
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import uuid
import zipfile

import numpy as np

from .common import checked_file, cleanup_container, file_hash, read_json, require, write_json
from .schurvins import PIN, pose_observation, project_native_corrections
from .native_events import analyze_native_events
from tools.openvins.native_accuracy import associate, ordered
from tools.schurvins.dataset import reconcile
from tools.schurvins.build import symlinks
from tools.schurvins.evaluate import evaluate_rows, read_reference, read_uzh_reference


CONFIG_KEYS = {'schema_version', 'backend', 'purpose', 'run_id', 'sequence', 'mode',
               'rate', 'input', 'installation', 'reference_archive', 'output', 'timeout_s'}
OPTIONAL_CONFIG_KEYS = {'start_offset_s', 'native_parameters', 'state_output_policy', 'memory_limit_gib'}
INIT_PARAMETERS = {'rm27_init_min_frames', 'rm27_init_max_frames', 'rm27_init_min_span_s',
    'rm27_init_max_imu_gap_s', 'rm27_init_min_excitation_mps2', 'rm27_init_gravity_mps2',
    'rm27_init_gravity_tolerance_mps2', 'rm27_init_max_gyro_bias_rps', 'rm27_init_max_accel_bias_mps2',
    'rm27_init_max_residual_mps', 'rm27_init_min_singular_ratio', 'rm27_init_max_rotation_residual_rad',
    'rm27_init_min_imu_translation_m'}


def checked_tree(root, entries):
    """Check every frozen file, refusing paths outside its declared root."""
    root = Path(root).resolve()
    require(isinstance(entries, dict) and entries, 'INSTALLATION_INVALID', 'empty file manifest')
    for name, expected in entries.items():
        path = root / name
        # Symlink artifacts may point to a sibling build directory in catkin.
        # The lexical manifest key must still stay inside the declared root.
        require(not Path(name).is_absolute() and '..' not in Path(name).parts,
                'MANIFEST_PATH_INVALID', name)
        checked_file(path, expected)


def checked_installation(workspace, install):
    for tree, prefix in [('src', 'source'), ('devel', 'devel')]:
        root = workspace/tree
        expected = install.get(prefix+'_symlinks')
        require(isinstance(expected, dict) and symlinks(root) == expected,
                'INSTALLATION_SYMLINK_MISMATCH', tree+' link names/targets changed or unfrozen')
        entries = install[prefix+'_files_sha256']
        names = {str(p.relative_to(root)) for p in root.rglob('*')
                 if p.is_file() and '.git' not in p.parts and '__pycache__' not in p.parts}
        require(names == set(entries), 'INSTALLATION_TREE_CHANGED', tree+' file membership')
        checked_tree(root, entries)


def preflight(config):
    require(CONFIG_KEYS <= set(config) <= CONFIG_KEYS | OPTIONAL_CONFIG_KEYS,
            'RUN_CONFIG_UNSUPPORTED', 'dedicated SchurVINS keys')
    require(type(config['schema_version']) is int and config['schema_version'] == 1
            and config['backend'] == 'SchurVINS', 'RUN_SCHEMA_UNSUPPORTED', 'SchurVINS v1')
    uzh = config['purpose'] == 'UZH_FPV_PUBLIC_BASELINE'
    require(config['purpose'] in ('EUROC_PUBLIC_BASELINE', 'UZH_FPV_PUBLIC_BASELINE'),
            'PUBLIC_REPLAY_REQUIRED', 'explicit public dataset experiment')
    require(isinstance(config['run_id'], str) and bool(config['run_id']), 'RUN_ID_INVALID', 'run ID')
    require(config['sequence'] in ({'indoor_forward_3', 'indoor_forward_6'} if uzh else
            {'V1_01_easy', 'V1_03_difficult', 'MH_04_difficult', 'V2_02_medium'}),
            'SEQUENCE_UNSUPPORTED', 'explicit initial comparison matrix')
    require(config['mode'] in ('mono', 'stereo'), 'MODE_INVALID', 'mono or stereo')
    require(type(config['rate']) in (int, float) and config['rate'] in (.5, 1.),
            'RATE_INVALID', '0.5 accuracy and 1.0 performance groups')
    require(type(config['timeout_s']) in (int, float) and np.isfinite(config['timeout_s'])
            and config['timeout_s'] > 0, 'TIMEOUT_INVALID', 'positive finite seconds')
    offset = config.get('start_offset_s', 0)
    require(type(offset) in (int, float) and np.isfinite(offset) and offset >= 0,
            'START_OFFSET_INVALID', 'explicit nonnegative source-time offset')
    params = config.get('native_parameters', {})
    require(isinstance(params, dict) and set(params) <= INIT_PARAMETERS | {'automatic_reinitialization'},
            'NATIVE_PARAMETERS_INVALID', 'only explicit initialization/recovery experiment parameters')
    for key, value in params.items():
        require((type(value) is bool if key == 'automatic_reinitialization' else
                 type(value) in (int, float) and np.isfinite(value) and value > 0),
                'NATIVE_PARAMETERS_INVALID', key)
    require(config.get('memory_limit_gib', 12) in (4, 12) and type(config.get('memory_limit_gib', 12)) is int,
            'RESOURCE_LIMIT_INVALID', 'distinct 4/12 GiB experiment groups')
    if 'state_output_policy' in config:
        from ..state_output import StateOutputPolicy
        policy = config['state_output_policy']
        require(isinstance(policy, dict) and set(policy) == set(StateOutputPolicy.__dataclass_fields__),
                'STATE_OUTPUT_POLICY_INVALID', 'all explicit offline output policy fields required')
        StateOutputPolicy(**policy)
    output = Path(config['output']).resolve()
    require(not output.exists(), 'OUTPUT_EXISTS', str(output))
    inputs = Path(config['input']).resolve()
    data = read_json(inputs/'input.json')
    artifacts = ({'sensors.zip', 'expected.jsonl', 'native_calibration.yaml', 'calibration.json',
                  'source-events.jsonl', 'source-config.json'} if uzh else {'sensors.zip', 'expected.jsonl'})
    if 'derivation' in data:
        require(not uzh and offset == 0 and data['derivation'].get('experiment_kind') == 'ARTIFICIAL_DISTURBANCE',
                'DERIVATION_INVALID', 'explicit EuRoC disturbance, separate from start-offset group')
        require(data['derivation'].get('validation', {}).get('status') == 'PASS',
                'FAULT_INPUT_REJECTED', 'invalid derived input must not reach the estimator')
        artifacts |= {'source-expected.jsonl', 'sensor-config.json', 'sensor-events.jsonl', 'derivation.json'}
        if 'delivery-schedule.json' in data.get('artifacts_sha256', {}):
            artifacts.add('delivery-schedule.json')
    require({p.name for p in inputs.iterdir()} == {'input.json'} | artifacts
            and all(p.is_file() for p in inputs.iterdir()),
            'INPUT_TRUTH_OR_TOPIC_FORBIDDEN', 'only explicitly prepared sensor artifacts may be mounted')
    require(data.get('schema_version') == 1 and data.get('purpose') ==
            ('UZH_FPV_PUBLIC_REPLAY' if uzh else 'EUROC_PUBLIC_REPLAY')
            and data.get('truth_included') is False and data.get('mode') == config['mode'],
            'INPUT_UNVERIFIED', 'sensor-only preparation for this mode required')
    require(set(data.get('artifacts_sha256', {})) == artifacts,
            'INPUT_UNVERIFIED', 'complete prepared artifacts required')
    if uzh:
        require(data.get('dataset') == 'UZH_FPV' and config['mode'] == 'mono'
                and data.get('native_imu_config_complete') is True
                and data.get('source_camera_id') in ('cam0', 'cam1'),
                'INPUT_UNVERIFIED', 'independent UZH mono with complete explicit calibration')
    checked_tree(inputs, data['artifacts_sha256'])
    expected_sensors = {'cam0', 'imu0'} | ({'cam1'} if config['mode'] == 'stereo' else set())
    require(set(data['counts']) == expected_sensors
            and all(type(n) is int and n > 0 for n in data['counts'].values()),
            'INPUT_UNVERIFIED', 'all selected sensor streams required')
    # A manifest truth_included flag alone is not isolation evidence.
    with zipfile.ZipFile(inputs/'sensors.zip') as z:
        names = z.namelist()
        require(len(names) == len(set(names)), 'INPUT_INVALID', 'duplicate ZIP members')
        for name in names:
            uzh_index = 'left_images.txt' if data.get('source_camera_id') == 'cam0' else 'right_images.txt'
            permitted = ((name in ('imu.txt', uzh_index) or
                          re.fullmatch(r'img/image_[01]_[0-9]+\.png', name) is not None) if uzh else
                          any(name in ('mav0/'+s+'/data.csv', 'mav0/'+s+'/sensor.yaml')
                        or re.fullmatch(r'mav0/'+s+r'/data/[0-9]+\.png', name)
                        for s in expected_sensors))
            require(permitted, 'INPUT_TRUTH_OR_TOPIC_FORBIDDEN', name)
    archive, _ = checked_file(config['reference_archive'], data['original_archive_sha256'])
    install = read_json(config['installation'])
    require(install.get('schema_version') == 1 and install.get('backend') == 'SchurVINS'
            and install.get('commit') == PIN, 'BACKEND_VERSION_UNSUPPORTED', PIN)
    require(re.fullmatch(r'sha256:[a-f0-9]{64}', install.get('image_id', '')) is not None,
            'IMAGE_UNPINNED', 'immutable image ID required')
    workspace = Path(install['workspace']).resolve()
    checked_installation(workspace, install)
    require(install.get('binary') in ('devel/lib/svo_ros/svo_node',
                                    'devel/.private/svo_ros/lib/svo_ros/svo_node'),
            'INSTALLATION_INVALID', 'expected native executable')
    checked_file(workspace/install['binary'], install['binary_sha256'])
    if install.get('architecture') == 'arm64':
        from tools.schurvins.arm64 import checked_runtime
        checked_runtime(install)
    else:
        require(install.get('architecture', 'amd64') in ('amd64', 'x86_64'),
                'ARCHITECTURE_UNSUPPORTED', 'explicit PC or ARM64 reference installation')
    require('#define USE_SCHUR_VINS' in (workspace/'src/SchurVINS/svo/include/svo/global.h').read_text(),
            'SCHUR_PATH_UNVERIFIED', 'compiled Schur selection source required')
    cmake = (workspace/'src/SchurVINS/svo_cmake/cmake/Modules/SvoSetup.cmake').read_text()
    require('SET(USE_LOOP_CLOSING FALSE)' in cmake and 'SET(USE_GLOBAL_MAP FALSE)' in cmake,
            'VIO_SCOPE_INVALID', 'loop/global mapping must be disabled in source')
    repo = Path(__file__).resolve().parents[5]
    for path in [output, inputs, workspace, repo/'tools/schurvins']:
        require(':' not in str(path) and '\n' not in str(path), 'MOUNT_PATH_INVALID', str(path))
    require(output != inputs and inputs not in output.parents
            and workspace not in output.parents, 'OUTPUT_INVALID', 'independent run directory')
    return inputs, data, install, workspace, archive


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def watchdog_evidence(root, execution):
    """A complete timer stream needs its joined-thread marker and clean exits."""
    marker = root/'raw/watchdog-complete.json'
    ticks = root/'raw/watchdog.jsonl'
    complete = (marker.is_file() and read_json(marker) == dict(
        schema_version=1, trigger='INDEPENDENT_SCHEDULE', thread_joined=True,
        clock_source='OBSERVER_CLOCK_MONOTONIC')
        and execution.get('failure') is None and not execution.get('cleanup_errors')
        and all(execution.get('exits', {}).get(s) == 0
                for s in ['native', 'player', 'observer', 'master']))
    return (jsonl(ticks) if ticks.is_file() else []), complete


def freeze_transport(source, output):
    """Snapshot the two executed ROS scripts before mounting them read-only.

    Unrelated build-tool development cannot change a running experiment. Host
    scoring/input modules are still checked against their original identities.
    """
    source, output = Path(source), Path(output)
    expected = {name: file_hash(source/name)
                for name in ('run_inner.py', 'ros_transport.py')}
    output.mkdir(exist_ok=False)
    for name, digest in expected.items():
        require(not (source/name).is_symlink(), 'TOOL_INVALID', name)
        shutil.copyfile(source/name, output/name)
        checked_file(output/name, digest)
    return expected


def check_native_text(path, poses):
    rows = [line.split() for line in Path(path).read_text().splitlines() if line.strip()]
    require(len(rows) == len(poses) > 0, 'NATIVE_TEXT_MISMATCH', 'pose count')
    max_stamp, max_value = 0, 0.
    for text, pose in zip(rows, poses):
        require(len(text) == 8, 'NATIVE_TEXT_MISMATCH', 'TUM columns')
        text_ns = int(Decimal(text[0])*10**9)
        stamp_error = abs(text_ns-pose['state_ns'])
        actual = np.asarray([float(x) for x in text[1:]], float)
        expected = np.asarray(pose['position']+pose['quaternion_xyzw'], float)
        value_error = float(np.max(np.abs(actual-expected)))
        require(np.isfinite(actual).all() and stamp_error <= 1000 and value_error <= 1e-9,
                'NATIVE_TEXT_MISMATCH', 'raw ROS vs double/decimal native export')
        max_stamp, max_value = max(max_stamp, stamp_error), max(max_value, value_error)
    return dict(count=len(rows), max_stamp_residual_ns=max_stamp,
                max_value_residual=max_value, timestamp_tolerance_ns=1000,
                value_tolerance=1e-9, authority='integer ROS header; native text rounding retained')


def normalize(root, data, install, config, native_observations=None):
    root = Path(root)
    raw = root/'raw'
    # Index against the frozen source, even when a failed run only delivered a
    # prefix (or lost an observer message). Receipt loss must not renumber poses.
    source_ledger = Path(config['input'])/('source-expected.jsonl' if 'derivation' in data else 'expected.jsonl')
    cameras = [r['state_ns'] for r in jsonl(source_ledger)
               if r['stream'] == 'cam0']
    ordered(cameras)
    if 'derivation' not in data:
        require(len(cameras) == data['counts']['cam0'], 'INPUT_UNVERIFIED', 'source camera count')
    poses, info = jsonl(raw/'pose.jsonl'), jsonl(raw/'info.jsonl')
    if native_observations is None:
        native_observations = analyze_native_events(root/'native-events.jsonl', cameras,
            first_pose_sensor_ns=poses[0]['state_ns'] if poses else None)
    segments = {row['sensor_ns']:row['observation_segment'] for row in
        native_observations.get('coverage', {}).get('camera_observation_segments', [])}
    first_segment = min(segments.values()) if segments else 0
    require(poses and info, 'NATIVE_OUTPUT_EMPTY', 'pose and native Info required')
    ordered([r['state_ns'] for r in info])
    camera_ids = associate(cameras, [r['state_ns'] for r in poses], 0)
    info_camera_ids = associate(cameras, [r['state_ns'] for r in info], 0)
    by_time = {r['state_ns']:r for r in info}
    tracking_seen, losses, native_info_gaps = False, [], []
    for previous, row in zip([None]+info[:-1], info):
        require(type(row['stage']) is int and type(row['tracking_quality']) is int,
                'INFO_INVALID', 'raw enum fields required')
        require(type(row['native_sequence']) is int and row['native_sequence'] >= 0,
                'INFO_INVALID', 'ROS topic header sequence required')
        if row['stage'] == 2:
            tracking_seen = True
        elif tracking_seen:
            losses.append(dict(state_ns=row['state_ns'], stage=row['stage'],
                               quality=row['tracking_quality']))
        if previous is not None and row['native_sequence'] != previous['native_sequence']+1:
            native_info_gaps.append([previous['native_sequence'], row['native_sequence']])
    associated, projections = [], []
    for index, (pose, camera_id) in enumerate(zip(poses, camera_ids)):
        require(pose.get('frame_id') == 'world', 'FRAME_INVALID', 'native parent must be world')
        observation = by_time.get(pose['state_ns'])
        # roscpp overwrites Header.seq with a per-topic publication counter.
        # Info includes initialization frames while pose_imu does not, so their
        # sequences are not a shared frame identifier. The exact image stamp is.
        require(observation is not None and observation['stage'] == 2,
                'POSE_INFO_MISMATCH', 'same exact image time and native tracking stage required')
        require(type(pose['native_sequence']) is int and pose['native_sequence'] >= 0,
                'POSE_INVALID', 'ROS topic header sequence required')
        if segments:
            require(pose['state_ns'] in segments, 'NATIVE_EPOCH_UNOBSERVED',
                    'pose missing completed native frontend segment')
        epoch = segments.get(pose['state_ns'], first_segment)-first_segment
        reset_evidence = (dict(source='native-events.jsonl', observation_segment=segments[pose['state_ns']],
                               meaning='explicit native reset-call segment; not inferred from pose') if segments else None)
        associated.append(dict(pose, image_ns=cameras[camera_id], source_frame_id=camera_id, epoch=epoch))
        projections.append(pose_observation(source_frame_id=camera_id, image_ns=cameras[camera_id],
            state_ns=pose['state_ns'], observed_monotonic_ns=pose['observed_monotonic_ns'],
            position=pose['position'], quaternion_xyzw=pose['quaternion_xyzw'],
            run_id=config['run_id'], epoch=epoch, binary_sha256=install['binary_sha256'],
            sequence=config['sequence'], raw_reference=dict(file='raw/pose.jsonl',
                topic='/svo/pose_imu', ordinal=index), stage=observation['stage'],
            quality=observation['tracking_quality'], processing_time_s=observation['processing_time_s'],
            reset_evidence=reset_evidence, dataset='UZH_FPV' if data.get('dataset') == 'UZH_FPV' else 'EuRoC'))
    text_parity = check_native_text(root/'logs/imu_traj.txt', poses)
    processing = np.asarray([r['processing_time_s'] for r in info], float)
    require(np.isfinite(processing).all() and np.all(processing >= 0),
            'INFO_INVALID', 'finite nonnegative frontend processing times')
    tracking_processing = [r['processing_time_s'] for r in info if r['stage'] == 2]
    published = [r for r in jsonl(raw/'published.jsonl') if r['stream'] == 'cam0']
    calls = [r['publish_call_monotonic_ns'] for r in published]
    ordered(calls)
    published_ids = associate(cameras, [r['state_ns'] for r in published], 0)
    lags = []
    for row, camera_id in zip(info, info_camera_ids):
        last = bisect.bisect_right(calls, row['observed_monotonic_ns'])-1
        lags.append(max(0, published_ids[last]-camera_id) if last >= 0 else 0)
    pose_sequence_gaps = [[previous['native_sequence'], row['native_sequence']]
                         for previous, row in zip(poses, poses[1:])
                         if row['native_sequence'] != previous['native_sequence']+1]
    report = dict(pose_count=len(poses), native_info_count=len(info),
        native_camera_callbacks=dict(observed_count=len(info_camera_ids),
            input_count=len(cameras), missing_count=len(cameras)-len(info_camera_ids),
            first_camera_index=info_camera_ids[0], last_camera_index=info_camera_ids[-1],
            meaning='Info after native image callback; paused callbacks are not visual updates'),
        native_info_sequence_gaps=native_info_gaps, tracking_loss_observations=losses,
        pose_topic_sequence_gaps=pose_sequence_gaps,
        header_sequence_semantics='ROS per-topic publication counter; not native trace ID or cross-topic frame ID',
        pose_info_association='exact integer image header timestamp and native stage',
        native_tail_seen=info[-1]['state_ns'] == data['last_ns']['cam0'],
        initialization_proxy=dict(first_pose_camera_index=camera_ids[0],
            first_pose_sensor_elapsed_s=(poses[0]['state_ns']-cameras[0])/1e9,
            exact_initialization_completion_time=None, source='first native kTracking pose; proxy only'),
        frontend_processing_s=dict(p50=float(np.percentile(tracking_processing,50)),
            p95=float(np.percentile(tracking_processing,95)), max=float(max(tracking_processing)), samples=len(tracking_processing),
            scope='native stage=2 only; nontracking timer may be stale'),
        camera_lag_at_info_receipt=dict(p50=float(np.percentile(lags,50)),
                                       p95=float(np.percentile(lags,95)), max=int(max(lags))),
        native_text_parity=text_parity, native_reset_count=None,
        adapter_epoch_count=1, automatic_reinitialization=False,
        estimator_imu_ingestion='UNOBSERVED', publication_latency_s=None,
        successful_visual_update_count=None,
        pose_semantics='camera-triggered state output; per-frame successful Schur correction unobserved',
        imu_propagated_stream='NOT_EXPOSED_BY_REVIEWED_WRAPPER')
    if native_observations['status'] != 'NOT_RUN':
        report.update(native_observations='native-observations.json',
            native_reset_count=native_observations['lifecycle']['post_frontend_reset_call_count'],
            adapter_epoch_count=len({row['epoch'] for row in associated}),
            successful_visual_update_count=native_observations['visual_corrections']['count'],
            pose_semantics='camera-triggered numerical state; explicit corrections separately audited')
    (root/'online').mkdir()
    write_json(root/'online/trajectory.json', projections)
    write_json(root/'online/raw-associated.json', associated)
    write_json(root/'observations.json', report)
    return associated, projections, report


def run(config):
    result = dict(status='REFUSED', execution_started=False,
                  qualification='PC_PUBLIC_NUMERICAL_REPLAY_ONLY', stage_result='PARTIAL')
    root = None
    try:
        inputs, data, install, workspace, archive = preflight(config)
        output = Path(config['output']).resolve()
        output.mkdir(parents=True, exist_ok=False)
        # Ownership starts only after exclusive creation succeeds. A competing
        # run may create this path after preflight; never finalize its evidence.
        root = output
        repo = Path(__file__).resolve().parents[5]
        transport = root/'runtime-tools'
        transport_hashes = freeze_transport(repo/'tools/schurvins', transport)
        container = 'rm27-schur-'+uuid.uuid4().hex
        command = ['docker','run','--rm','--name',container,'--network','none','--cpus','4','--memory',
            str(config.get('memory_limit_gib', 12))+'g',
            '--user',str(os.getuid())+':'+str(os.getgid()),
            '-v',str(workspace)+':/ws:ro','-v',str(transport)+':/tools:ro',
            '-v',str(inputs)+':/input:ro','-v',str(root)+':/out',install['image_id'],'bash','-c',
            'source /opt/ros/melodic/setup.bash && source /ws/devel/setup.bash && '
            'export ROS_HOME=/out/ros ROS_LOG_DIR=/out/ros/log && '
            'exec python /tools/run_inner.py "$@"', 'rm27-schur-inner',
            '--mode',config['mode'],'--rate',str(config['rate']),'--timeout',str(config['timeout_s'])]
        command += ['--start-offset', str(config.get('start_offset_s', 0))]
        if install.get('architecture') == 'arm64':
            from tools.schurvins.arm64 import replay_command
            (root/'runtime-tmp').mkdir()
            command = replay_command(install,transport,inputs,root,config['mode'],config['rate'],
                config['timeout_s'],config.get('start_offset_s', 0),container_name=container,
                memory_gib=config.get('memory_limit_gib', 12))
        command += ['--parameters-json', json.dumps(config.get('native_parameters', {}), allow_nan=False)]
        runtime_sources = [Path(__file__), Path(__file__).with_name('schurvins.py'),
            Path(__file__).with_name('common.py'), Path(__file__).with_name('online.py'),
            Path(__file__).with_name('trajectory.py'), Path(__file__).parent.parent/'schema.py',
            Path(__file__).with_name('native_events.py'),
            Path(__file__).parent.parent/'state_output.py',
            repo/'tools/openvins/native_accuracy.py',
            *(repo/'tools/schurvins'/name for name in
              ('dataset.py', 'evaluate.py', 'build.py', 'prepare.py')),
            *(transport/name for name in transport_hashes)]
        if install.get('architecture') == 'arm64':
            runtime_sources.append(repo/'tools/schurvins/arm64.py')
        result.update(status='EXECUTING', execution_started=True, config=config, command=command,
                      installation_sha256=file_hash(config['installation']), input=data,
                      runtime_sha256=file_hash(__file__),
                      adapter_sha256=file_hash(Path(__file__).with_name('schurvins.py')),
                      tool_sha256=transport_hashes,
                      runtime_sources_sha256={str(p):file_hash(p) for p in runtime_sources},
                      final_optimized_trajectory=None)
        write_json(root/'run.json',result)
        with (root/'docker.log').open('x') as log:
            try:
                completed = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,
                                           timeout=config['timeout_s']+90)
            except BaseException as exc:
                result.update(status='FAILED', failure_code='EXECUTION_INTERRUPTED',
                              failure_reason=type(exc).__name__)
                raise
            finally:
                result['container_cleanup'] = cleanup_container(container, log)
        result['exit_code'] = completed.returncode
        if completed.returncode != 0:
            result.update(status='FAILED', failure_code='BACKEND_EXECUTION_FAILED',
                          failure_reason='see retained docker.log/native.log')
        require(result['container_cleanup']['absence_verified'], 'CONTAINER_CLEANUP_UNVERIFIED',
                'owned replay container absence not verified; outputs may still change')
        execution = read_json(root/'execution.json')
        result['execution'] = execution
        if completed.returncode != 0 or execution.get('failure') is not None:
            result.update(status='FAILED', failure_code='BACKEND_EXECUTION_FAILED',
                          failure_reason='see retained execution.json/native.log')
        elif not all(execution.get('exits',{}).get(s) == 0 for s in ['native','player','observer','master']):
            result.update(status='FAILED', failure_code='PROCESS_EXIT_FAILED',
                          failure_reason='native/player/observer/master clean exits required')
        # Failed execution/transport remains a failure. Valid retained poses may
        # still be scored diagnostically; no missing observations are fabricated.
        try:
            result['input_reconciliation'] = reconcile(inputs,root/'raw',
                start_offset_ns=int(Decimal(str(config.get('start_offset_s', 0)))*10**9), rate=config['rate'])
        except (ValueError, KeyError, OSError) as exc:
            result['input_reconciliation'] = dict(status='FAILED', reason=str(exc))
            if 'failure_code' not in result:
                result.update(status='FAILED', failure_code='INPUT_RECONCILIATION_FAILED',
                              failure_reason=str(exc))
        # Partial-run scores require exactly the same frozen source and binary
        # provenance as successful runs, and are never an execution PASS.
        checked_tree(inputs,data['artifacts_sha256'])
        checked_installation(workspace,install)
        if install.get('architecture') == 'arm64':
            from tools.schurvins.arm64 import checked_runtime
            checked_runtime(install)
        checked_file(archive,data['original_archive_sha256'])
        ledger = inputs/('source-expected.jsonl' if 'derivation' in data else 'expected.jsonl')
        native = analyze_native_events(root/'native-events.jsonl', jsonl(ledger))
        write_json(root/'native-observations.json',native)
        result['native_observations'] = 'native-observations.json'
        require(native['status'] != 'FAIL', 'NATIVE_OBSERVATION_INVALID', 'see native-observations.json')
        if 'state_output_policy' in config:
            from ..state_output import StateOutputPolicy
            ticks, watchdog_complete = watchdog_evidence(root, execution)
            states = project_native_corrections(native, jsonl(root/'raw/info.jsonl'),
                policy=StateOutputPolicy(**config['state_output_policy']), run_id=config['run_id'],
                sequence=config['sequence'], binary_sha256=install['binary_sha256'],
                dataset='UZH_FPV' if data.get('dataset') == 'UZH_FPV' else 'EuRoC',
                watchdog_events=ticks,
                watchdog_complete=watchdog_complete)
            write_json(root/'native-state-output.json', states)
            result['native_state_output'] = 'native-state-output.json'
        associated, projections, observations = normalize(root,data,install,config,native)
        reference = (read_uzh_reference(archive, camera_id=data['source_camera_id'])
                     if data.get('dataset') == 'UZH_FPV' else read_reference(archive))
        numerical = evaluate_rows(associated,reference,tolerance_ns=0)
        projected = evaluate_rows(projections,reference,tolerance_ns=0)
        require(numerical == projected, 'ADAPTER_METRICS_DIFFER', 'same-run exact metric parity')
        write_json(root/'evaluation.json',numerical)
        for path, digest in result['runtime_sources_sha256'].items():
            checked_file(path, digest)
        result.update(observations=observations, evaluation='evaluation.json', same_run_metrics_exact=True,
                      online_trajectory='online/trajectory.json')
        if 'failure_code' in result:
            result['stage_result'] = 'FAILED_RUN_WITH_AUDITED_NUMERICAL_OUTPUT'
        require(numerical['scored_epoch_count'] == 1, 'TRAJECTORY_NOT_EVALUABLE', 'one scored epoch required')
        require(not observations['tracking_loss_observations'], 'TRACKING_LOST', 'native left kTracking')
        require(not observations['native_info_sequence_gaps'], 'NATIVE_INFO_GAP', 'native Info loss observed')
        require(not observations['pose_topic_sequence_gaps'], 'POSE_TOPIC_GAP', 'pose topic publication loss observed')
        require(observations['native_tail_seen'], 'NATIVE_TAIL_MISSING', 'last camera not observed in native Info')
        require(data.get('dataset') != 'UZH_FPV' or data.get('timing_application_verified') is True,
                'REFERENCE_TIMING_UNQUALIFIED', 'UZH camera time precompensation unknown; retained scores diagnostic only')
        if 'failure_code' not in result:
            emulated = install.get('architecture') == 'arm64'
            result.update(status='PASS',stage_result=(
                'EMULATED_ARM64_REFERENCE_REPLAY' if emulated else 'PC_REPLAY_AND_SAME_RUN_NUMERICAL_PARITY'),
                qualification=('EMULATED / MOCK VERIFIED' if emulated else 'PC OFFLINE VERIFIED'))
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as exc:
        failure = dict(failure_code=getattr(exc,'code','SCHUR_RUNTIME_ERROR'), failure_reason=str(exc))
        if 'failure_code' in result:
            result['analysis_failure'] = failure
        else:
            result.update(status='FAILED' if result['execution_started'] else 'REFUSED', **failure)
    finally:
        if root is not None:
            if result.get('container_cleanup', {}).get('absence_verified') is not True:
                # A failed daemon/remove operation may leave the producer alive.
                # Hashing its files here would falsely finalize mutable evidence.
                result['artifact_finalization'] = 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'
            else:
                result['output_sha256'] = {str(p.relative_to(root)):file_hash(p)
                    for p in root.rglob('*') if p.is_file() and p.name != 'run.json'}
            write_json(root/'run.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args=parser.parse_args(argv)
    result=run(read_json(args.config))
    print(json.dumps({k:result[k] for k in ['status','stage_result','failure_reason'] if k in result},indent=2))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
