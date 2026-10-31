"""Sample cgroup v2 CPU/memory during playback; these values are not RSS."""
import json
from pathlib import Path
import signal
import time


def main():
    stopping = False
    def stop(*unused):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    sources = {'cgroup_cpu': Path('/sys/fs/cgroup/cpu.stat'),
               'cgroup_memory_bytes': Path('/sys/fs/cgroup/memory.current')}
    if not all(path.is_file() for path in sources.values()):
        Path('/out/resource-sampling.json').write_text(json.dumps(dict(
            status='NOT_RUN', reason='cgroup v2 counters unavailable')) + '\n')
        return
    with Path('/out/resources.jsonl').open('x', buffering=1) as stream:
        while not stopping:
            row = dict(observed_monotonic_ns=time.monotonic_ns(),
                       **{name: path.read_text().strip() for name, path in sources.items()})
            stream.write(json.dumps(row, allow_nan=False) + '\n')
            time.sleep(.5)


if __name__ == '__main__':
    main()
