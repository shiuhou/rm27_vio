"""Freeze a sensor-only EuRoC ZIP and decoded input ledger; never export truth."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import zipfile
import tempfile
import shutil
import os

import yaml

import cv2
import numpy as np

from tools.schurvins.ros_transport import inputs
from rm27.perception.vision.localization.experiments.common import canonical, file_hash, number, read_json, require, write_json
from rm27.perception.vision.localization.experiments import sensors


def prepare(archive, output, mode):
    if mode not in ('mono', 'stereo'):
        raise ValueError('mono/stereo mode required')
    archive, output = Path(archive).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    original_hash = file_hash(archive)
    with zipfile.ZipFile(archive) as source:
        if len(source.namelist()) != len(set(source.namelist())):
            raise ValueError('duplicate ZIP members')
        streams = inputs(source, mode)
        output.mkdir(parents=True)
        calibration = {}
        with zipfile.ZipFile(output/'sensors.zip', 'w', compression=zipfile.ZIP_STORED) as target, \
                (output/'expected.jsonl').open('x') as ledger:
            for sensor, rows in streams.items():
                prefix = 'mav0/'+sensor+'/'
                for suffix in ['data.csv', 'sensor.yaml']:
                    payload = source.read(prefix+suffix)
                    target.writestr(prefix+suffix, payload)
                    if suffix == 'sensor.yaml':
                        calibration[sensor] = hashlib.sha256(payload).hexdigest()
                for ns, data in rows:
                    row = dict(stream=sensor, state_ns=ns)
                    if sensor == 'imu0':
                        row.update(gyro=data[:3], accel=data[3:])
                    else:
                        member = prefix+'data/'+data
                        encoded = source.read(member)
                        pixels = cv2.imdecode(np.frombuffer(encoded,np.uint8), cv2.IMREAD_UNCHANGED)
                        if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
                            raise ValueError('mono8 image required')
                        target.writestr(member, encoded)
                        row.update(pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
                                   height=pixels.shape[0], width=pixels.shape[1],
                                   encoding='mono8', step=pixels.shape[1])
                    ledger.write(json.dumps(row, allow_nan=False)+'\n')
        if file_hash(archive) != original_hash:
            raise ValueError('source archive changed')
        report = dict(schema_version=1, purpose='EUROC_PUBLIC_REPLAY', mode=mode,
                      original_archive_sha256=original_hash, truth_included=False,
                      counts={s:len(rows) for s,rows in streams.items()},
                      first_ns={s:rows[0][0] for s,rows in streams.items()},
                      last_ns={s:rows[-1][0] for s,rows in streams.items()},
                      calibration_sha256=calibration,
                      artifacts_sha256={p.name:file_hash(p) for p in output.iterdir()})
        write_json(output/'input.json', report)
    return report


def _uzh_calibration(calibration_archive, camera_id, template, time_shift_policy):
    """Convert sourced Kalibr camera-to-IMU geometry without assuming timing."""
    require(time_shift_policy in ('source_calibration', 'not_applied'),
            'SENSOR_CALIBRATION_INVALID', 'explicit source_calibration/not_applied time policy')
    calibration_archive = Path(calibration_archive).resolve()
    source_hash = file_hash(calibration_archive)
    with zipfile.ZipFile(calibration_archive) as archive:
        require(len(archive.namelist()) == len(set(archive.namelist())),
                'SENSOR_CALIBRATION_INVALID', 'duplicate calibration ZIP members')
        camera_member = next(n for n in archive.namelist() if '/camchain-imucam-' in n and n.endswith('.yaml'))
        imu_member = next(n for n in archive.namelist() if '/imu-' in n and n.endswith('.yaml'))
        report_member = next(n for n in archive.namelist() if '/results-imucam-' in n and n.endswith('.txt'))
        camera_bytes, imu_bytes, report_bytes = (archive.read(n) for n in (camera_member, imu_member, report_member))
    camera = yaml.safe_load(camera_bytes)[camera_id]
    imu = yaml.safe_load(imu_bytes)['imu0']
    require(camera['camera_model'] == 'pinhole' and camera['distortion_model'] == 'equidistant',
            'SENSOR_CALIBRATION_INVALID', 'supported UZH pinhole/equidistant model required')
    sensors._rigid(camera['T_cam_imu'])
    T_imu_camera = np.linalg.inv(np.array(camera['T_cam_imu'], dtype=float))
    sensors._rigid(T_imu_camera.tolist())
    for value in camera['intrinsics']+camera['distortion_coeffs']:
        number(value, 'source camera parameter')
    require(len(camera['intrinsics']) == 4 and len(camera['distortion_coeffs']) == 4
            and len(camera['resolution']) == 2
            and all(type(v) is int and v > 0 for v in camera['resolution']),
            'SENSOR_CALIBRATION_INVALID', 'source camera dimensions/parameters')
    report_text = report_bytes.decode()
    require('(t_imu = t_cam + shift)' in report_text and 'T_ci:' in report_text,
            'SENSOR_CALIBRATION_INVALID', 'source report must state time and transform direction')
    shift = number(camera['timeshift_cam_imu'], 'source camera-to-IMU time shift')
    camera_hash = hashlib.sha256(camera_bytes).hexdigest()
    def matrix(values):
        array = np.asarray(values)
        return dict(rows=array.shape[0], cols=1 if array.ndim == 1 else array.shape[1],
                    data=array.flatten().tolist())
    native = dict(label='UZH_FPV:'+camera_id, cameras=[dict(camera=dict(label=camera_id,
        type='pinhole', image_width=camera['resolution'][0], image_height=camera['resolution'][1],
        intrinsics=matrix(camera['intrinsics']), distortion=dict(type='equidistant',
        parameters=matrix(camera['distortion_coeffs']))), T_B_C=matrix(T_imu_camera))])
    # Timing and calibration values come from the official source. Values with
    # no sensor calibration source remain explicit estimator experiment tuning.
    native['imu_params'] = {}
    template_identity = None
    if template is not None:
        template = Path(template).resolve()
        baseline = yaml.safe_load(template.read_text())
        native['imu_params'] = copy.deepcopy(baseline['imu_params'])
        native['imu_initialization'] = copy.deepcopy(baseline['imu_initialization'])
        template_identity = dict(path=str(template), sha256=file_hash(template), qualification='EXPERIMENT_CONFIG',
            inherited_imu_fields=sorted(set(baseline['imu_params'])-{
                'sigma_omega_c', 'sigma_acc_c', 'sigma_omega_bias_c', 'sigma_acc_bias_c', 'imu_rate', 'delay_imu_cam'}),
            inherited_initialization=copy.deepcopy(baseline['imu_initialization']))
    for native_key, source_key in [('sigma_omega_c', 'gyroscope_noise_density'),
        ('sigma_acc_c', 'accelerometer_noise_density'), ('sigma_omega_bias_c', 'gyroscope_random_walk'),
        ('sigma_acc_bias_c', 'accelerometer_random_walk'), ('imu_rate', 'update_rate')]:
        native['imu_params'][native_key] = number(imu[source_key], source_key, positive=True)
    native['imu_params']['delay_imu_cam'] = -shift if time_shift_policy == 'source_calibration' else 0.
    provenance = dict(schema_version=1, source_archive=str(calibration_archive), source_sha256=source_hash,
        source_member_sha256={camera_member:camera_hash, imu_member:hashlib.sha256(imu_bytes).hexdigest(),
                              report_member:hashlib.sha256(report_bytes).hexdigest()},
        camera_id=camera_id, source_T_cam_imu=camera['T_cam_imu'], native_T_B_C=T_imu_camera.tolist(),
        T_B_C_convention='camera_to_imu; rigid inverse of source T_cam_imu',
        source_timeshift_cam_imu_s=shift, source_time_convention='t_imu = t_cam + shift',
        native_time_convention='t_corrected_camera = t_camera - delay_imu_cam',
        time_shift_policy=time_shift_policy, source_sensor_timestamps_modified=False,
        timing_application_verified=False, shutter_model='UNKNOWN', exposure_semantics='UNKNOWN',
        line_delay_ns=None, native_line_delay_policy='OMITTED_UNKNOWN', experiment_template=template_identity,
        native_imu_config_complete=template is not None, covariance_qualification='SOURCE_PARAMETERS_ONLY')
    return native, provenance


def prepare_uzh(archive, calibration_archive, output, camera_id='cam0', *,
                recording_directory=None, native_template=None, time_shift_policy='source_calibration'):
    """Freeze UZH mono input in its own layout; truth is never exported.

    The existing ROS transport can consume this only through its explicit UZH
    support. It is deliberately not relabeled as a EuRoC/ASL recording. The
    expected ledger covers actual publisher payload fields; source-events.jsonl
    separately preserves original decimal text/ps and the ns representation map.
    """
    require(camera_id in ('cam0', 'cam1'), 'SENSOR_IDENTITY_INVALID', 'UZH camera ID')
    archive, output = Path(archive).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    original_hash = file_hash(archive)
    native, calibration = _uzh_calibration(calibration_archive, camera_id, native_template, time_shift_policy)
    # Reuse an audited recording when available. Otherwise only temporary raw
    # images are created, removed on success/failure after the final sensor ZIP.
    temporary = tempfile.TemporaryDirectory(prefix='rm27-uzh-intake-') if recording_directory is None else None
    output_owned = False
    try:
        if recording_directory is None:
            recording_directory = Path(temporary.name)/'recording'
            sensors.import_dataset(archive, recording_directory, 'UZH_FPV', camera_id=camera_id)
        config, events, recording = sensors.load_recording(recording_directory)
        require(config['provenance']['source_sha256'] == original_hash
                and config['provenance'].get('dataset') == 'UZH_FPV'
                and config['selected_camera_ids'] == [camera_id] and config['mode'] == 'mono',
                'SENSOR_DATASET_INVALID', 'recording source/mode differs')
        output.mkdir(parents=True, exist_ok=False)
        output_owned = True
        with zipfile.ZipFile(archive) as source, zipfile.ZipFile(output/'sensors.zip', 'w',
                compression=zipfile.ZIP_STORED) as target, (output/'expected.jsonl').open('x') as ledger:
            image_index = 'left_images.txt' if camera_id == 'cam0' else 'right_images.txt'
            image_members = {int(line.split()[0]):line.split()[2]
                for line in source.read(image_index).decode().splitlines()
                if line.strip() and not line.lstrip().startswith('#')}
            members = set(source.namelist())
            for member in ('imu.txt', image_index):
                target.writestr(member, source.read(member))
            image_names = set()
            for event in events:
                # Native mono uses cam0 topic regardless of the source camera;
                # this remap and its calibration are explicit in input.json.
                stream = 'imu0' if event['kind'] == 'IMU' else 'cam0'
                row = dict(stream=stream, state_ns=sensors.event_time(event, config)[1])
                if stream == 'imu0':
                    row.update(gyro=event['payload']['gyro_xyz'], accel=event['payload']['accel_xyz'])
                else:
                    row.update(pixels_sha256=event['payload']['pixels_sha256'], width=event['payload']['width'],
                        height=event['payload']['height'], step=event['payload']['width'], encoding='mono8')
                    # Preserve actual ZIP member names, avoiding any timestamp
                    # filename invention or synthetic EuRoC path conversion.
                    sequence = event['sequence']
                    member = image_members[sequence]
                    require(member in members and not Path(member).is_absolute() and '..' not in Path(member).parts,
                            'SENSOR_IMAGE_INVALID', 'source image member')
                    if member not in image_names:
                        target.writestr(member, source.read(member))
                        image_names.add(member)
                ledger.write(canonical(row)+'\n')
        (output/'source-events.jsonl').write_text(''.join(canonical(e)+'\n' for e in events))
        write_json(output/'source-config.json', config)
        (output/'native_calibration.yaml').write_text(yaml.safe_dump(native, sort_keys=False))
        write_json(output/'calibration.json', calibration)
        require(file_hash(archive) == original_hash, 'HASH_MISMATCH', 'source archive changed')
        counts = dict(imu0=sum(e['kind'] == 'IMU' for e in events), cam0=sum(e['kind'] == 'CAMERA' for e in events))
        report = dict(schema_version=1, purpose='UZH_FPV_PUBLIC_REPLAY', dataset='UZH_FPV', mode='mono',
            source_camera_id=camera_id, native_camera_id='cam0', camera_remap_explicit=True,
            original_archive_sha256=original_hash, truth_included=False, counts=counts,
            first_ns={s:min(sensors.event_time(e,config)[1] for e in events if (e['kind']=='IMU') == (s=='imu0')) for s in counts},
            last_ns={s:max(sensors.event_time(e,config)[1] for e in events if (e['kind']=='IMU') == (s=='imu0')) for s in counts},
            calibration_sha256={'source_archive':calibration['source_sha256'], 'native_calibration.yaml':file_hash(output/'native_calibration.yaml')},
            native_calibration_file='native_calibration.yaml', native_imu_config_complete=calibration['native_imu_config_complete'],
            time_shift_policy=time_shift_policy, timing_application_verified=False,
            source_recording_sha256=file_hash(Path(recording_directory)/'recording.json'),
            timestamp_representation=dict(raw_unit='ps', preserved_decimal_text=True,
                replay_ns_quantization='ROUND_HALF_EVEN', maximum_representation_error_ns=.5,
                physical_clock_resolution='UNKNOWN'),
            artifacts_sha256={p.name:file_hash(p) for p in output.iterdir()})
        write_json(output/'input.json', report)
        return report
    except Exception:
        if output_owned:
            import shutil
            shutil.rmtree(output)
        raise
    finally:
        if temporary is not None:
            temporary.cleanup()


def _payload_row(event, config):
    """Existing native transport payload ledger, with no state/truth content."""
    row = dict(stream=event['sensor_id'],state_ns=sensors.event_time(event,config)[1])
    if event['kind'] == 'IMU':
        row.update(gyro=event['payload']['gyro_xyz'],accel=event['payload']['accel_xyz'])
    else:
        row.update(pixels_sha256=event['payload']['pixels_sha256'],width=event['payload']['width'],
            height=event['payload']['height'],step=event['payload']['width'],encoding='mono8')
    return row


def derive_prepared(source_directory, output_directory, profile, *, recording_directory=None):
    """Apply the common deterministic disturbance to native EuRoC sensor inputs.

    Original source stamps/sequence IDs and the full original scoring denominator
    remain separate from the delivered payload ledger. Delays have an explicit
    delivery-only schedule; no acquisition time is rewritten. Invalid injected
    inputs retain a FAIL manifest for preflight rejection, never a repaired VIO
    experiment. This function does not execute an estimator.
    """
    source,output=Path(source_directory).resolve(),Path(output_directory).resolve()
    if output.exists():
        raise FileExistsError(output)
    require(source not in output.parents,'SENSOR_OUTPUT_INVALID','derived native output must be outside source')
    parent=read_json(source/'input.json')
    require(parent.get('purpose') == 'EUROC_PUBLIC_REPLAY' and parent.get('truth_included') is False
            and parent.get('mode') in ('mono','stereo'), 'SENSOR_DATASET_INVALID','native EuRoC sensor-only input required')
    for name,digest in parent['artifacts_sha256'].items():
        require(Path(name).name == name,'SENSOR_DATASET_INVALID','prepared-local artifacts required')
        require(file_hash(source/name) == digest,'HASH_MISMATCH',name)
    parent_hash=file_hash(source/'input.json')
    temporary=tempfile.TemporaryDirectory(prefix='rm27-native-fault-')
    owned=False
    try:
        temp=Path(temporary.name)
        if recording_directory is None:
            recording_directory=temp/'parent'
            sensors.import_dataset(source/'sensors.zip',recording_directory,'EUROC',mode=parent['mode'])
        config,events,recording=sensors.load_recording(recording_directory)
        expected=[json.loads(line) for line in (source/'expected.jsonl').read_text().splitlines() if line.strip()]
        event_payloads=[_payload_row(event,config) for event in events]
        streams=set(parent['counts'])
        require(config['mode'] == parent['mode'] and set(config['selected_camera_ids']) == streams-{'imu0'}
                and all([row for row in expected if row['stream']==s] ==
                        [row for row in event_payloads if row['stream']==s] for s in streams),
                'SENSOR_DATASET_INVALID','common recording does not match full prepared source payloads')
        disturbance=temp/'disturbance'
        derived_manifest=sensors.derive_recording(recording_directory,disturbance,profile)
        derived_config,derived,_=sensors.load_recording(disturbance,allow_invalid=True)
        # The storage check precedes persistent output creation; originals and
        # unrelated build/data products are never cleaned to satisfy it.
        ancestor=output.parent
        while not ancestor.exists():
            ancestor=ancestor.parent
        estimate=(0 if profile['kind']=='delivery_delay' else (source/'sensors.zip').stat().st_size)+len(derived)*3000
        require(shutil.disk_usage(ancestor).free-estimate >= 20*1024**3,
                'SENSOR_STORAGE_BLOCKED','native derivation would breach 20GiB reserve')
        output.mkdir(parents=True,exist_ok=False)
        owned=True
        zip_storage='DERIVED_PAYLOAD_ARCHIVE'
        if profile['kind']=='delivery_delay':
            # Delivery delay never changes an acquisition value, timestamp or
            # pixel. Preserve the entire frozen archive byte-for-byte. Shared
            # hardlinks are safe only under the same immutable hash audit used
            # before and after every replay; no source is opened for writing.
            try:
                os.link(source/'sensors.zip',output/'sensors.zip')
                zip_storage='IMMUTABLE_PARENT_HARDLINK'
            except OSError:
                require(shutil.disk_usage(ancestor).free-estimate-(source/'sensors.zip').stat().st_size >= 20*1024**3,
                        'SENSOR_STORAGE_BLOCKED','delay archive copy would breach reserve')
                shutil.copyfile(source/'sensors.zip',output/'sensors.zip')
                zip_storage='BYTE_IDENTICAL_PARENT_COPY'
        else:
            with zipfile.ZipFile(source/'sensors.zip') as original,zipfile.ZipFile(output/'sensors.zip','w',
                    compression=zipfile.ZIP_STORED) as target:
                for stream in sorted(streams):
                    prefix='mav0/'+stream+'/'
                    target.writestr(prefix+'sensor.yaml',original.read(prefix+'sensor.yaml'))
                    stream_events=[event for event in derived if event['sensor_id']==stream]
                    csv_rows=[]
                    for event in stream_events:
                        ns=sensors.event_time(event,derived_config)[1]
                        if stream == 'imu0':
                            csv_rows.append(','.join([str(ns),*[str(x) for x in
                                event['payload']['gyro_xyz']+event['payload']['accel_xyz']]])+'\n')
                        else:
                            csv_rows.append(str(ns)+','+str(ns)+'.png\n')
                            target.writestr(prefix+'data/'+str(ns)+'.png',
                                            (disturbance/event['payload']['path']).read_bytes())
                    target.writestr(prefix+'data.csv',''.join(csv_rows))
        payloads=[_payload_row(event,derived_config) for event in derived]
        (output/'expected.jsonl').write_text(''.join(canonical(row)+'\n' for row in payloads))
        original_expected=source/('source-expected.jsonl' if (source/'source-expected.jsonl').is_file() else 'expected.jsonl')
        shutil.copyfile(original_expected,output/'source-expected.jsonl')
        shutil.copyfile(disturbance/'events.jsonl',output/'sensor-events.jsonl')
        shutil.copyfile(disturbance/'config.json',output/'sensor-config.json')
        provenance=dict(derived_manifest['derivation'],parent_input_sha256=parent_hash,
            parent_input_path=str(source),parent_artifacts_sha256=parent['artifacts_sha256'],
            validation=derived_manifest['validation'],source_event_count=len(events),derived_event_count=len(derived),
            native_archive_storage=zip_storage)
        write_json(output/'derivation.json',provenance)
        delayed=[dict(stream=e['sensor_id'],state_ns=sensors.event_time(e,derived_config)[1],
            source_sequence=e['sequence'],delivery_delay_ns=e['delivery_delay_ns'])
            for e in derived if e.get('delivery_delay_ns',0)>0]
        if delayed:
            write_json(output/'delivery-schedule.json',dict(schema_version=1,
                time_semantics='DELIVERY_DELAY_ONLY_SOURCE_STAMP_UNCHANGED',entries=delayed))
        counts={s:sum(event['sensor_id']==s for event in derived) for s in streams}
        report=dict(schema_version=1,purpose=parent['purpose'],dataset='EUROC',mode=parent['mode'],
            original_archive_sha256=parent['original_archive_sha256'],truth_included=False,counts=counts,
            source_counts=parent.get('source_counts',parent['counts']),
            source_first_ns=parent.get('source_first_ns',parent['first_ns']),
            source_last_ns=parent.get('source_last_ns',parent['last_ns']),
            first_ns={s:min((row['state_ns'] for row in payloads if row['stream']==s),default=None) for s in streams},
            last_ns={s:max((row['state_ns'] for row in payloads if row['stream']==s),default=None) for s in streams},
            calibration_sha256=parent['calibration_sha256'],derivation=provenance,
            artifacts_sha256={p.name:file_hash(p) for p in output.iterdir()})
        require(file_hash(source/'input.json')==parent_hash,'HASH_MISMATCH','parent changed during derivation')
        for name,digest in parent['artifacts_sha256'].items():
            require(file_hash(source/name)==digest,'HASH_MISMATCH','parent artifact changed during derivation')
        write_json(output/'input.json',report)
        return report
    except Exception:
        if owned:
            shutil.rmtree(output)
        raise
    finally:
        temporary.cleanup()


def reconcile(directory, raw, *, start_offset_ns=0, rate=.5):
    """Full equality at publisher and observer; native ingestion is a separate fact."""
    directory, raw = Path(directory), Path(raw)
    report = json.loads((directory/'input.json').read_text())
    require(type(start_offset_ns) is int and start_offset_ns>=0,
            'SENSOR_INTEGER_INVALID','nonnegative integer replay start offset')
    require(type(rate) in (int,float) and rate in (.5,1.),'SENSOR_RATE_INVALID','explicit .5/1 playback rate')
    origin=min(report.get('source_first_ns',report['first_ns']).values())
    cutoff=origin+start_offset_ns
    expected = {s:[] for s in report['counts']}
    for line in (directory/'expected.jsonl').read_text().splitlines():
        row = json.loads(line)
        expected[row['stream']].append(row)
    full_expected={s:list(rows) for s,rows in expected.items()}
    expected={s:[row for row in rows if row['state_ns']>=cutoff] for s,rows in expected.items()}
    source_order={s:[row['state_ns'] for row in rows] for s,rows in expected.items()}
    schedule_path=directory/'delivery-schedule.json'
    if schedule_path.is_file():
        schedule=json.loads(schedule_path.read_text())
        require(schedule.get('schema_version') == 1 and schedule.get('time_semantics') ==
                'DELIVERY_DELAY_ONLY_SOURCE_STAMP_UNCHANGED','SENSOR_FAULT_INVALID','delivery schedule semantics')
        delays={}
        for entry in schedule['entries']:
            key=(entry['stream'],entry['state_ns'])
            require(key not in delays and entry['stream'] in expected
                    and type(entry['delivery_delay_ns']) is int and entry['delivery_delay_ns']>=0
                    and any(r['state_ns']==entry['state_ns'] for r in full_expected[entry['stream']]),
                    'SENSOR_FAULT_INVALID','duplicate/unknown/invalid delivery schedule entry')
            delays[key]=entry['delivery_delay_ns']
        for stream,rows in expected.items():
            order_index={stamp:i for i,stamp in enumerate(source_order[stream])}
            rows.sort(key=lambda row: ((row['state_ns']-origin)*(2 if rate==.5 else 1)
                                      +delays.get((stream,row['state_ns']),0),order_index[row['state_ns']]))
    published = {s:[] for s in expected}
    for line in (raw/'published.jsonl').read_text().splitlines():
        row = json.loads(line)
        published[row['stream']].append(row)
    result = {}
    for sensor, wanted in expected.items():
        if len(full_expected[sensor]) != report['counts'][sensor]:
            raise ValueError(sensor+' manifest/ledger count mismatch')
        seen = [json.loads(line) for line in (raw/(sensor+'.jsonl')).read_text().splitlines()]
        for label, rows in [('published',published[sensor]),('observed',seen)]:
            if len(rows) != len(wanted):
                raise ValueError(sensor+' '+label+' count mismatch')
            for actual, source in zip(rows,wanted):
                if any(actual.get(k) != v for k,v in source.items() if k != 'stream'):
                    raise ValueError(sensor+' '+label+' content/order mismatch')
        result[sensor] = dict(published=len(published[sensor]), observed=len(seen),
                              content_exact=True, native_imu_ingestion_verified=False)
        if start_offset_ns:
            result[sensor].update(omitted_prefix_count=len(full_expected[sensor])-len(wanted),
                full_delivered_stream_count=len(full_expected[sensor]),replay_cutoff_ns=cutoff)
        if schedule_path.is_file():
            result[sensor].update(delivery_order='DECLARED_SCHEDULE',
                source_order_changed=[r['state_ns'] for r in wanted] != source_order[sensor],
                adjacent_source_time_inversions=sum(b['state_ns']<a['state_ns'] for a,b in zip(wanted,wanted[1:])))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mode', choices=['mono','stereo'], required=True)
    parser.add_argument('--dataset', choices=['EUROC','UZH_FPV'], default='EUROC')
    parser.add_argument('--calibration-archive')
    parser.add_argument('--native-template')
    parser.add_argument('--source-recording')
    parser.add_argument('--camera', choices=['cam0','cam1'], default='cam0')
    parser.add_argument('--time-shift-policy', choices=['source_calibration','not_applied'], default='source_calibration')
    args = parser.parse_args()
    if args.dataset == 'UZH_FPV':
        if args.mode != 'mono' or args.calibration_archive is None:
            parser.error('UZH_FPV requires mono and --calibration-archive')
        result = prepare_uzh(args.archive,args.calibration_archive,args.output,args.camera,
            recording_directory=args.source_recording,native_template=args.native_template,time_shift_policy=args.time_shift_policy)
    else:
        result = prepare(args.archive,args.output,args.mode)
    print(json.dumps(result,indent=2))
