# RM27 VIO research

This branch includes an **unfinished SchurVINS mono WIP checkpoint**, published
at the user's request to preserve the work before a possible change of direction.
The latest joint-window engine changes are not fully integrated or qualified;
the last complete mono replays failed. Earlier candidate test passes do not
qualify this snapshot. No replacement ARM64/M3C deployment package is issued.
See the current [deployment evidence](.agents/docs/M3C_DEPLOYMENT.md) for the
remaining runtime integration work and validation boundaries.

`main` is exclusively for Camera + IMU visual-inertial odometry research.
The current estimator line is OpenVINS. Legacy monocular SLAM launchers,
stella_vslam / ORB-SLAM3 experiments, VSL roadmaps and their tests are archived
on `legacy/slam` at `74d44b09c65e2a839b74d834756f0d178207d679`.
They are not supported by the main-branch runner.

This worktree is the independent `research/m3c-vio` deployment branch. The
[M3C deployment report](.agents/docs/M3C_DEPLOYMENT.md) contains measured
PC/ARM64 results and actual M3C acceptance; the
[integration record](.agents/docs/SCHURVINS_INTEGRATION.md) preserves the shared baseline. The
[ROS-free C++ runtime](tools/schurvins/headless/README.md), read-only M3C audit
and native benchmark are reusable on other Linux ARM64 targets. Original RK3576
work remains in its separate worktree. Publication is authorized only to
`origin/research/m3c-vio`; no merge or PR is included.

## Start here

- [M3C laptop 測試與結果回傳操作手冊](.agents/docs/M3C_LAPTOP_HANDOFF.md)
- [Current handoff](HANDOFF.md)
- [VIO scope and plan](new_plan.md)
- [Resume guide](rm27/perception/vision/docs/CODEX_RESUME.md)
- [Camera + IMU data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
- [Sensor evidence](rm27/perception/vision/docs/IMU_SOURCE_AUDIT.md)
- [PX4–M3C connection](rm27/perception/vision/docs/PX4_M3C_CONNECTION.md)
- [OpenVINS tools](tools/openvins/README.md)
- [M3C SchurVINS deployment evidence](.agents/docs/M3C_DEPLOYMENT.md)
- [Bench/offline evidence index](.maixpy/README.md)

VIO-P and VIO-S0 remain PARTIAL. Stage A/B validates transport and synthetic
ODOMETRY only; real VIO, Camera–IMU synchronization/calibration, M3C estimator
deployment and flight are not qualified. The latest physical logger diagnosis
does not constitute a loss fix. No acceptance gate was relaxed by this cleanup.

## Retained shared infrastructure

Camera capture/calibration and image-evidence preparation, timestamp/pose
contracts, rigid transforms and evaluation primitives remain because VIO uses
them. The Python package stays `rm27.perception.vision.localization`.
`experiments.runner` accepts only `backend=OpenVINS`; unknown or legacy
backends are refused before output creation. No alternate SLAM launcher remains.

With an existing compatible Python/NumPy/OpenCV/pytest environment:

```powershell
py -3 -m pytest tests -q --ignore=tests/camera_capture.py
```

The excluded file is an unrelated, untracked Maix hardware program, not a test.
ROS2-specific integration needs its existing sourced environment. Offline tests
do not open a device or authorize a dated capture script.

Raw recordings, public datasets, third-party binaries/builds, environments,
credentials and most dated diagnostic evidence remain outside Git. Published
historical manifests describe their original artifacts, not this refactored
tree. A fresh clone cannot claim a saved-data or hardware replay without them.
Repository cleanup does not update the main drone repository or the Vault.


## Shared real-camera test videos

[Shared video catalog](data/shared_vision/README.md) provides four user-supplied
real drone-camera MP4 fixtures, hashes, reported capture settings, decoding checks
and replay instructions. `data/shared_vision/` is a relative link to the shared
RM27 dataset, with one physical copy; videos are local data, not included in a
standalone clone. Keep derived test outputs separate from these inputs. See the
catalog for exposure-mapping and synchronized-IMU/timestamp limitations.

Host validation completed on 2026-11-02 UTC for dataset
`rm27-real-camera-exposure-01`, using source HEAD `704c40a`, Python 3.12.3,
OpenCV 5.0.0/FFMPEG and one observed decoder thread. All four source sizes and
SHA-256 values matched the catalog before and after processing. The existing
`rm27.perception.vision.localization.analyze_video` entry point decoded every
frame at native 1344x760 and converted it to grayscale, without resizing or
undistortion. Quality diagnostics sampled every 30 frames, with ORB500 and LK;
the encoded-PTS large-gap factor remained 1.5.

| Video ID | Complete decoded frames | Diagnostic samples |
|---|---:|---:|
| clip01 | 5,505 | 184 |
| clip02 | 17,987 | 600 |
| clip03 | 10,079 | 336 |
| clip04 | 11,891 | 397 |
| Total | 45,462 | 1,517 |

Decode/grayscale checks passed. An independent sequential pass also verified
every frame's dimensions and all 12 catalog BGR pixel hashes. The four existing
analyzer tests passed. ORB/LK results are a diagnostic baseline without a feature
quality acceptance threshold; actual SchurVINS frontend, full VIO and hardware
validation were **NOT_RUN**. Individual exposure values remain unknown; the
batch multiset is 1500/2500/2500/3000 us, LCG, 1x. Missing synchronized IMU and
matched calibration prevent qualifying these clips as a VIO dataset.

Evidence identities:

- Catalog SHA-256: `00ba541032febbfd203396c043d5e20d8aced6a5e6e49811a95857d6c1792208`.
- Analyzer SHA-256: `df02adf1865d30979be12eaa43b5036efd9229933676462a2d605dd2bdd0770e`.
- Local raw results/configuration: `artifacts/shared-real-camera-regression/result.json`
  and per-clip provenance/CSV files; ignored outputs are not included in Git.

With the shared data present and a compatible Python/OpenCV environment, rerun
the existing analyzer from the repository root into fresh output directories:

```bash
python - <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
catalog = Path('data/shared_vision/real_camera_exposure/manifest.json')
manifest = json.loads(catalog.read_text())
for clip in manifest['clips']:
    source = catalog.parent / clip['path']
    with source.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    assert digest == clip['sha256'] and source.stat().st_size == clip['size_bytes']
    subprocess.run([sys.executable, '-m',
                    'rm27.perception.vision.localization.analyze_video', str(source),
                    '--out', f"artifacts/shared-video-{clip['id']}",
                    '--sample-step', '30', '--large-gap-factor', '1.5'], check=True)
PY
```

This command reruns the analyzer; it does not perform the additional catalog
pixel-hash comparison. Container PTS and image-motion proxies are not exposure
timestamps, physical attitude or capture-drop measurements.
