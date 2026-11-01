"""VIO-only entry point and absence of executable legacy SLAM backends."""
from pathlib import Path

import pytest

from rm27.perception.vision.localization.experiments.common import ExperimentError
from rm27.perception.vision.localization.experiments.online import normalized_state
from rm27.perception.vision.localization.experiments.runner import run

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('backend', ['stella_vslam', 'ORB-SLAM3', 'SchurVINS', 'unknown', None])
def test_runner_rejects_non_vio_before_creating_output(tmp_path, backend):
    output = tmp_path/'must-not-exist'
    with pytest.raises(ExperimentError, match='BACKEND_UNSUPPORTED'):
        run(dict(schema_version=1, run_id='scope', backend=backend,
                 output=str(output), dataset=str(tmp_path/'not-a-dataset')))
    assert not output.exists()


@pytest.mark.parametrize('backend', ['stella_vslam', 'ORB-SLAM3'])
def test_online_contract_has_no_legacy_tracking_mapper(backend):
    with pytest.raises(ExperimentError, match='BACKEND_UNSUPPORTED'):
        normalized_state(backend, 'Tracking')


def test_active_tree_excludes_slam_launchers_and_roadmaps():
    base = ROOT/'rm27/perception/vision'
    forbidden = ['localization/experiments/backends.py',
                 'localization/experiments/public_tum.py',
                 'localization/experiments/observers/build.py',
                 'localization/experiments/upstream_contracts.json',
                 'docs/SLAM_RECOVERY_AND_AUDIT.md']
    assert not [name for name in forbidden if (base/name).exists()]
    assert not list((ROOT/'tests').glob('test_slam_*.py'))
    assert not [p for p in base.glob('M3C_OS04A10_SLAM*/**/*') if p.is_file()]


def test_openvins_tracking_state_stays_unknown():
    assert normalized_state('OpenVINS', 'Tracking') == 'UNKNOWN'
