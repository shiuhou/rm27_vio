"""Read-only SSH inventory for an M3C candidate; never opens sensor devices.

The audit is not a deployment, a camera capture, or a UART preflight approval.
It retains SSH failures as BLOCKED instead of substituting historical hardware.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess


# Send this fixed script on stdin. No remote files, packages or settings change.
REMOTE_AUDIT = r'''
import glob,json,os,pathlib,platform,struct,subprocess
P=pathlib.Path
def read(name,limit=65536):
    try:
        with open(name,'rb') as f: return f.read(limit).decode('utf-8','replace').replace('\x00',' ').strip()
    except OSError as e: return {'unavailable':type(e).__name__}
def command(args):
    try:
        p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=5)
        return {'exit_code':p.returncode,'stdout':p.stdout[:65536].decode('utf-8','replace'),
                'stderr':p.stderr[:4096].decode('utf-8','replace'),'truncated':len(p.stdout)>65536}
    except (OSError,subprocess.SubprocessError) as e: return {'unavailable':type(e).__name__}
def elf(name):
    try:
        with open(name,'rb') as f: h=f.read(64)
        if h[:4]!=b'\x7fELF': return {'error':'not ELF'}
        return {'class_bits':{1:32,2:64}.get(h[4]),'endianness':{1:'little',2:'big'}.get(h[5]),
                'machine_id':struct.unpack('<H' if h[5]==1 else '>H',h[18:20])[0]}
    except (OSError,IndexError,struct.error) as e: return {'unavailable':type(e).__name__}
devices=[]
for pattern in ('/dev/video*','/dev/ttyS*','/dev/i2c-*','/dev/spidev*','/dev/ax_*'):
    for name in sorted(glob.glob(pattern))[:128]:
        try:
            s=os.stat(name);devices.append({'path':name,'mode':oct(s.st_mode & 0o777),
                'uid':s.st_uid,'gid':s.st_gid,'major':os.major(s.st_rdev),'minor':os.minor(s.st_rdev)})
        except OSError: pass
owners=[]
processes=[]
for proc in sorted(P('/proc').iterdir(),key=lambda p:p.name):
    if not proc.name.isdigit(): continue
    comm=read(str(proc/'comm'),256)
    processes.append({'pid':int(proc.name),'comm':comm})
    try:
        for fd in (proc/'fd').iterdir():
            try: target=os.readlink(str(fd))
            except OSError: continue
            if target.startswith(('/dev/video','/dev/ttyS','/dev/i2c-','/dev/spidev','/dev/ax_')):
                owners.append({'pid':int(proc.name),'comm':comm,'fd':fd.name,'target':target})
    except OSError: pass
    if len(processes)>=2048: break
sdk=[]
for base in ('/opt','/usr/local','/root','/maixapp','/usr/lib'):
    try:
        for p in sorted(P(base).iterdir()):
            if base!='/usr/lib' or any(x in p.name.lower() for x in ('maix','axera','opencv','libstdc++')):
                sdk.append(str(p))
    except OSError: pass
camera_sysfs={str(p):read(str(p/'name'),1024) for p in P('/sys/class/video4linux').glob('*')}
thermal={str(p):read(str(p),1024) for p in P('/sys/class/thermal').glob('thermal_zone*/temp')}
cpu_freq={str(p):read(str(p),1024) for p in P('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')}
result={'schema_version':1,'scope':'READ_ONLY_DEVICE_INVENTORY','hostname':platform.node(),
 'uname':list(platform.uname()),'architecture':platform.machine(),'python_version':platform.python_version(),
 'model':read('/proc/device-tree/model'),'compatible':read('/proc/device-tree/compatible'),
 'soc_id':read('/sys/devices/soc0/soc_id'),'soc_family':read('/sys/devices/soc0/family'),
 'cpuinfo':read('/proc/cpuinfo'),'meminfo':read('/proc/meminfo'),'os_release':read('/etc/os-release'),
 'process_elf':elf('/proc/self/exe'),'libc':command(['getconf','GNU_LIBC_VERSION']),
 'loader':command(['readelf','-l','/proc/self/exe']),
 'filesystem':command(['df','-h']),'compiler':command(['gcc','--version']),
 'cxx_compiler':command(['g++','--version']),'cmake':command(['cmake','--version']),
 'pkg_config':command(['pkg-config','--modversion','opencv4','eigen3','yaml-cpp']),
 'library_cache':command(['ldconfig','-p']),'sdk_candidate_paths':sdk[:512],
 'devices':devices,'camera_sysfs':camera_sysfs,'device_owners':owners[:256],
 'processes':processes,'thermal_millicelsius':thermal,'cpu_frequency_khz':cpu_freq,
 'physical_device_opened':False,'camera_capture_started':False,'uart_settings_read_or_changed':False,
 'deployment_qualification':'NOT_RUN'}
print(json.dumps(result,sort_keys=True,separators=(',',':')))
'''


def _run(args, **kwargs):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=kwargs.pop('timeout', 30), **kwargs)
        return dict(exit_code=result.returncode, stdout=result.stdout, stderr=result.stderr)
    except subprocess.TimeoutExpired as exc:
        def decode(value):
            return value.decode('utf-8', 'replace') if isinstance(value, bytes) else value or ''
        return dict(exit_code=None, stdout=decode(exc.stdout), stderr=decode(exc.stderr), error='TIMEOUT')
    except OSError as exc:
        return dict(exit_code=None, stdout='', stderr=str(exc), error=type(exc).__name__)


def validate_inventory(value):
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1:
        raise ValueError('unsupported inventory schema')
    if value.get('scope') != 'READ_ONLY_DEVICE_INVENTORY':
        raise ValueError('unexpected audit scope')
    for key in ('hostname', 'architecture', 'os_release', 'meminfo'):
        if not isinstance(value.get(key), str) or not value[key]:
            raise ValueError('missing inventory field: ' + key)
    for key in ('physical_device_opened', 'camera_capture_started', 'uart_settings_read_or_changed'):
        if value.get(key) is not False:
            raise ValueError('audit must not open or configure physical sensor devices')
    return value


def audit(host, output, *, timeout_s=45):
    if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:\-]*', host):
        raise ValueError('host must be a single SSH target without options or shell syntax')
    if type(timeout_s) not in (int, float) or not 1 <= timeout_s <= 120:
        raise ValueError('timeout must be between 1 and 120 seconds')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    effective = _run(['ssh', '-G', host], timeout=10)
    selected = ('hostname ', 'user ', 'port ', 'proxycommand ', 'proxyjump ', 'identityfile ',
                'identitiesonly ', 'stricthostkeychecking ', 'userknownhostsfile ')
    effective['stdout'] = '\n'.join(line for line in effective['stdout'].splitlines() if line.startswith(selected))
    local = dict(ssh_effective=effective,
                 route=_run(['ip', '-j', 'route', 'get', host.rsplit('@', 1)[-1]], timeout=10),
                 interfaces=_run(['ip', '-j', 'address'], timeout=10))
    cmd = ['ssh', '-v', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
           '-o', 'ConnectTimeout=8', '-o', 'ConnectionAttempts=1', host,
           'env PYTHONDONTWRITEBYTECODE=1 python3 -']
    result = _run(cmd, input=REMOTE_AUDIT, timeout=timeout_s)
    report = dict(schema_version=1, host=host, observed_at=datetime.now(timezone.utc).isoformat(),
                  command=cmd, remote_script_sha256=hashlib.sha256(REMOTE_AUDIT.encode()).hexdigest(),
                  local_network=local, execution=result, status='BLOCKED', inventory=None,
                  qualification='DEVICE_INVENTORY_ONLY', physical_device_opened=False,
                  board_runtime='NOT_RUN', camera_capture='NOT_RUN', uart_test='NOT_RUN')
    if result['exit_code'] == 0:
        try:
            report['inventory'] = validate_inventory(json.loads(result['stdout']))
            report['status'] = 'PASS'
            report['identity_status'] = 'OBSERVED_REQUIRES_M3C_IDENTITY_REVIEW'
        except (ValueError, TypeError) as exc:
            report.update(status='FAIL', failure_code='INVALID_REMOTE_INVENTORY', failure_reason=str(exc))
    else:
        lines = result['stderr'].strip().splitlines()
        report.update(failure_code='SSH_AUDIT_UNAVAILABLE',
                      failure_reason=result.get('error') or (lines[-1] if lines else 'SSH exited nonzero'))
        authenticated = 'Authenticated to ' in result['stderr']
        report['ssh_phase'] = 'REMOTE_EXECUTION' if authenticated else 'BEFORE_AUTHENTICATION'
    (output/'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='root@10.18.198.1')
    parser.add_argument('--output', required=True, type=Path, help='fresh local evidence directory')
    parser.add_argument('--timeout-s', type=float, default=45)
    args = parser.parse_args(argv)
    try:
        result = audit(args.host, args.output, timeout_s=args.timeout_s)
    except (OSError, ValueError) as exc:
        parser.exit(2, str(exc)+'\n')
    print(json.dumps({key: result.get(key) for key in ('status', 'host', 'failure_code', 'ssh_phase')}, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
