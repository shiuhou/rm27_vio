"""Exercise the actual compiled C++ boundary when an execution command is supplied."""
import json
import os
import signal
from pathlib import Path
import subprocess

import pytest


@pytest.fixture
def command():
    raw = os.environ.get('SCHURVINS_HEADLESS_TEST_COMMAND')
    if not raw:
        pytest.skip('set SCHURVINS_HEADLESS_TEST_COMMAND to a JSON argv for the built runtime')
    value = json.loads(raw)
    assert isinstance(value, list) and value and all(isinstance(x, str) for x in value)
    return value


def dataset(path, camera='10,10.png\n20,20.png\n', imu='9,0,0,0,0,0,9.8\n11,0,0,0,0,0,9.8\n21,0,0,0,0,0,9.8\n'):
    for name, contents in [('cam0', camera), ('imu0', imu)]:
        folder = path / name
        folder.mkdir(parents=True)
        (folder / 'data.csv').write_text(contents)
    return path


def run(command, tmp_path, *, preexec_fn=None, **options):
    parameters = tmp_path / 'params.yaml'
    parameters.write_text('use_imu: true\n')
    args = dict(dataset=tmp_path / 'mav0', mode='mono', calib=tmp_path / 'missing-calibration.yaml',
                params=parameters, output=tmp_path / 'out', rate='0')
    args.update(options)
    argv = command + [str(x) for key, value in args.items() for x in ('--' + key, value)]
    return subprocess.run(argv, capture_output=True, text=True, timeout=30, preexec_fn=preexec_fn)


def test_help(command):
    result = subprocess.run(command + ['--help'], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0
    assert '--dataset' in result.stdout


@pytest.mark.parametrize('camera,imu,message', [
    ('10,10.png\n10,10.png\n', '9,0,0,0,0,0,9.8\n', 'strictly increasing'),
    ('10,10.png\n', '9,0,nan,0,0,0,9.8\n', 'finite measurement'),
    ('10,../10.png\n', '9,0,0,0,0,0,9.8\n', 'invalid image row'),
])
def test_invalid_input_rejected_before_output(command, tmp_path, camera, imu, message):
    dataset(tmp_path / 'mav0', camera, imu)
    result = run(command, tmp_path)
    assert result.returncode == 2
    assert message in result.stderr
    assert not (tmp_path / 'out').exists()


def test_stereo_requires_exact_pairs(command, tmp_path):
    data = dataset(tmp_path / 'mav0')
    (data / 'cam1').mkdir()
    (data / 'cam1/data.csv').write_text('11,11.png\n21,21.png\n')
    result = run(command, tmp_path, mode='stereo')
    assert result.returncode == 2
    assert 'exact pairing required' in result.stderr
    assert not (tmp_path / 'out').exists()


def test_existing_output_is_immutable(command, tmp_path):
    dataset(tmp_path / 'mav0')
    output = tmp_path / 'out'
    output.mkdir()
    sentinel = output / 'summary.json'
    sentinel.write_text('original evidence\n')
    result = run(command, tmp_path)
    assert result.returncode == 2
    assert 'output must not exist' in result.stderr
    assert sentinel.read_text() == 'original evidence\n'


def test_output_write_failure_is_caught_before_estimator_start(command, tmp_path):
    resource = pytest.importorskip('resource')
    dataset(tmp_path / 'mav0')

    def force_write_error():
        # Ignoring SIGXFSZ exposes EFBIG to ofstream instead of terminating it.
        signal.signal(signal.SIGXFSZ, signal.SIG_IGN)
        resource.setrlimit(resource.RLIMIT_FSIZE, (1, 1))

    result = run(command, tmp_path, preexec_fn=force_write_error)
    assert result.returncode == 2
    assert 'iostream error' in result.stderr
    assert not (tmp_path / 'out/poses.csv').exists()
