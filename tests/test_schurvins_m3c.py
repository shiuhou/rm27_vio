"""Audit production failure/identity boundaries without physical device access."""
import json

import pytest

from tools.schurvins import m3c


def inventory():
    return dict(schema_version=1, scope='READ_ONLY_DEVICE_INVENTORY', hostname='MOCK_BOARD',
                architecture='aarch64', os_release='MOCK_OS', meminfo='MOCK_MEMORY',
                physical_device_opened=False, camera_capture_started=False,
                uart_settings_read_or_changed=False)


def test_closed_ssh_retains_blocked_evidence_and_safe_options(tmp_path, monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        if command[:2] == ['ssh', '-G']:
            return dict(exit_code=0, stdout='user root\nhostname 10.18.198.1\nlocalcommand do-not-run\n', stderr='')
        if command[:2] == ['ssh', '-v']:
            return dict(exit_code=255, stdout='', stderr='kex_exchange_identification: Connection closed by remote host')
        return dict(exit_code=0, stdout='[]', stderr='')
    monkeypatch.setattr(m3c, '_run', run)
    destination = tmp_path/'closed'
    report = m3c.audit('root@10.18.198.1', destination)
    assert report['status'] == 'BLOCKED' and report['inventory'] is None
    assert report['ssh_phase'] == 'BEFORE_AUTHENTICATION'
    assert report['board_runtime'] == report['camera_capture'] == report['uart_test'] == 'NOT_RUN'
    assert json.loads((destination/'audit.json').read_text()) == report
    ssh, kwargs = calls[-1]
    assert 'StrictHostKeyChecking=yes' in ssh and 'BatchMode=yes' in ssh
    assert kwargs['input'] == m3c.REMOTE_AUDIT
    assert 'localcommand' not in report['local_network']['ssh_effective']['stdout']
    with pytest.raises(FileExistsError):
        m3c.audit('root@10.18.198.1', destination)
    assert len(calls) == 4


def test_inventory_pass_is_not_board_runtime_qualification(tmp_path, monkeypatch):
    def run(command, **kwargs):
        return dict(exit_code=0, stdout=json.dumps(inventory()) if command[:2] == ['ssh', '-v'] else '', stderr='')
    monkeypatch.setattr(m3c, '_run', run)
    report = m3c.audit('root@10.18.198.1', tmp_path/'identity')
    assert report['status'] == 'PASS'
    assert report['identity_status'] == 'OBSERVED_REQUIRES_M3C_IDENTITY_REVIEW'
    assert report['qualification'] == 'DEVICE_INVENTORY_ONLY'
    assert report['board_runtime'] == 'NOT_RUN'


@pytest.mark.parametrize('value', [True, '1', 2])
def test_invalid_schema_rejected(value):
    data = inventory(); data['schema_version'] = value
    with pytest.raises(ValueError):
        m3c.validate_inventory(data)


@pytest.mark.parametrize('host', ['-F/tmp/config', 'root@host; reboot', 'root@host $(id)', 'root@host\nreboot', ''])
def test_host_option_or_shell_injection_rejected_before_output(host, tmp_path):
    with pytest.raises(ValueError):
        m3c.audit(host, tmp_path/'invalid')
    assert not (tmp_path/'invalid').exists()


def test_remote_inventory_syntax_and_local_metadata_path():
    compile(m3c.REMOTE_AUDIT, '<remote_inventory>', 'exec')
    assert m3c.validate_inventory(inventory())['architecture'] == 'aarch64'
    with pytest.raises(ValueError):
        m3c.validate_inventory(dict(inventory(), physical_device_opened=True))
