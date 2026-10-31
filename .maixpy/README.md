# Published RM27 bench/offline research

2026-10-31 documentation publication: the four historical RAM diagnostic RESULT
reports below are included; raw data, firmware/parameters, approvals, source
overlays and test helpers remain local. See
`../rm27/perception/vision/docs/RAM_DIAGNOSTIC_PUBLICATION.md` for exact scope.
Older "NOT pushed" notices below describe their original dated snapshots.

Local latest2026-10-12: `imu-ram-run-20261012/RESULT.md` / analysis-reviewed.json /
verification.json. ONE approved physical RAM observation consumed,58s host bound,
two identical/checksum-valid frozen dumps. Dynamically located logger Wait/depth-one
overwrite, not a fix/global hardware PASS. Original raw prefix/wire/trace preserved;
13 board helpers/8 local tests and evidence verification pass. Runtime restored,
diagnostic firmware remains installed, no stock rollback/flight. Whole capture
still has two gyro holes, one startup TX rejection and196.32949Hz below200 gate.
S0/P PARTIAL, Stage C closed. Ignored local artifact NOT pushed. Do not rerun.
Below no-trace/no-flash/build-only notices are historical snapshots.

Local latest2026-10-12: `imu-ram-build-20261012/RESULT.md` / FLASH_APPROVAL.
Actual OFF/ON board compile/link VERIFIED; candidate04 NuttX native atomic port,
no flash/runtime/logger-cause PASS. Original candidate03 failure and trace snapshot
preserved. Official firmware copied locally; fresh backups/rollback choice/final
approval still required. This new ignored artifact folder has NOT been pushed.

Local follow-up2026-10-12: `imu-ram-trace-20261012/RESULT.md` / DESIGN / APPROVAL.
Bounded RAM collector and pinned patch implemented/tested OFFLINE, Cortex-M7
objects compiled; no full board image, flash or physical trial. TSAN unavailable;
skip trigger/hardware health UNKNOWN, S0/P PARTIAL, Stage C CLOSED. Original image
backup/full build/current params/approval remain gates. No push/publication of
this new locally ignored folder is claimed. Earlier DESIGN ONLY notices below
refer to historical snapshots; do not rerun high-load coordinators.

2026-10-12 publication index:
`../rm27/perception/vision/docs/PUBLICATION_20261012.md` and
`publication-20261012/VALIDATION.md`. Pending controller/diagnostic source and
measured-result reports are curated, not promoted to production. Raw recordings,
environments and original full manifest dependencies remain local; a clone cannot
claim hardware replay/qualification without them. Stage A/B unchanged, S0/P
PARTIAL, Stage C CLOSED. Dated coordinators must not be rerun as permission.

Latest ONE upstream-gyro observation: `imu-upstream-gyro-20261011/RESULT.md`.
Startup logging286→288/global19→21 FAILED; ONE PX4 TX-buffer rejection verified.
No60s source qualification; original subscription-skip trigger still UNKNOWN.
Prior clean vehicle_imu-only transport is unchanged, observer profile not adopted.
Exact original communication/logger settings restored; Disarmed/EV0, Stage A/B
preserved, VIO-S0/P PARTIAL, Stage C CLOSED. Windows34 PASS/3 POSIX skips and
M3C27 PASS/zero skips. Next RAM trace is DESIGN ONLY, not firmware authority.
No retry/commit/push/Vault. Older dated next actions below are historical.

Only an explicit curated subset of this normally ignored directory is versioned.
Paths are retained so the already tested imports keep working; no research module
has been silently promoted into the production estimator or flight stack.

Start with `../HANDOFF.md` and
`../rm27/perception/vision/docs/PX4_M3C_CONNECTION.md`.
Newest constraints supersede old next-action paragraphs: no spare UART, Stage A/B
retained, Stage C closed. All dated validation reports remain historical snapshots.

Latest ONE physical reader control: `imu-reader-control-20261011/RESULT.md`.
Fixed-r2500 did NOT eliminate subscription skips:197.104634Hz/two holes4460/4507us.
UART clean in measured window; source200Hz/continuity FAILED, S0/P PARTIAL/StageC
CLOSED. Settings restored; no blind retry, firmware/parameters/sensor changes.
Newprofile/comparison15 plus71 Windows regressions/M3C48 tests PASS. Next requires
scoped source execution/generation observability; Stage A/B preserved.

Latest OFFLINE residual diagnosis: `imu-source-diagnosis-20261011/DIAGNOSIS.md`.
4554us hole is logger vehicle_imu queue1 generation skip, not UART; trigger UNKNOWN.
197.215Hz still FAIL original200Hz,298 longer accel windows/source callback timing
explain most cadence deficit;271 long endpoints are not271 losses.56offline tests
PASS/zero skips;80own/36external old hashes and14source pins verified. No hardware/
settings/Git/Vault action. VIO-S0/P PARTIAL, StageC CLOSED; next comparison planned only.

Latest physical comparison: `imu-uart-reduced-trial-20261011/RESULT.md`.
ONE approved reduced-telemetry run completed60.016s normal data:5001 framesCRC
verified/no sequence gaps,64LINKintervals successful4980=received4980/no rejection.
Settings fully restored to original115200/streams/logger/SDLOG/M3Ctermios.
But11843 BMI088integrals197.215Hz and one4554us gyro coverage hole/logger
generation skip remain: inputPARTIAL, not200Hz/loss-free/VIO, StageC CLOSED.
New13board testsPASS/zero skips;17Windows14PASS/3POSIX skips;verifierexit0.
Runtime18 files/old evidence/StageA/B unchanged. No retry/push/Vault.
Next OFFLINE source/logger subscription skip analysis; no new physical action.
Earlier proposed reduced-telemetry wording below is now a historical snapshot.

Latest OFFLINE attribution: `imu-gap-diagnosis-20261011/DIAGNOSIS.md`.
Pinned CRC verifies10 LINK_NODE_STATUS plus23 other unknown frames; all3697
retained frames accounted without changing raw rows. Nine intervals: six TX
send-buffer rejections match six missing global positions. Terminal logging435:
rejection4→5/global34→36,398 successful writes=398 received frames. VERIFIED
sender rejection layer, exact burst/scheduling/drain trigger UNKNOWN; no fix.
Complete packet437 perf metadata says ulog_stream missed0; no repaired/admitted
dataset.15 new+9 existing offline tests PASS/zero skips, verifier exit0; source18/
old manifests/StageA/B unchanged. No device/settings/runtime change, commit/push.
Next proposed ONE reduced-telemetry60s trial in NEXT_TRIAL_PLAN.md, not executed
or authorized here. Source~196.43Hz/coverage/clocks/model remain unqualified,
VIO-S0/P PARTIAL, StageC CLOSED. Prior dated reports remain historical snapshots.

### Previous physical snapshot (attribution now narrowed above)

Latest physical result: `imu-uart-negotiated-trial-20261011/RESULT.md`.
Current M3C120-test precheck PASS/zero skips/18-source parity. Same-owner guarded
heartbeat negotiation physically reachedv2; logging START/STOP accepted. Missing
logging435 after434 stops the ONE trial;60s NOT_COMPLETED, no retry or complete
ULog. Failed-prefix BMI088513 records~196.43Hz/gaps are diagnostics, NOT≥200Hz,
VIO or sync PASS. Exact PX4/M3C settings restored, Disarmed/EV0/flow-ToF retained.
Fresh9 offline tests / independent provenance-restoration verifier pass. Next
OFFLINE attribution/counter/burst-budget plan; don't rerun frozen coordinators or
unchanged board tests. StageA/B unchanged; VIO-S0/P PARTIAL, StageC CLOSED.
No commit/push. Raw trial evidence is local/ignored, not automatically published.

### Previous offline snapshot (next-step text is historical)

Current working-tree software revision: `imu-negotiation-fix-20261011/VALIDATION.md`.
Explicit finite heartbeat-onlyv1 bootstrap on SAME UART owner, verified PX4v2 gate
before START; logging/ACK/post-upgrade safety remain strictlyv2.120 research-Linux
tests PASS (7 PTY)/zero skips,113 Windows PASS/7 platform skips,54 related tests and
226 root PASS/1 existing skip. No M3C/PX4 access/deployment/capture/settings this turn.
Latest board precheck `controller-m3c-startup-pty-20261011/VALIDATION.md` covers OLD
87-test source, not this revision. Next isolated current120-test M3C precheck, then
ONE separately approved physical capture with a new reviewed coordinator. Never
rerun frozen dated one-off helpers or require outgoingv2 before launching the peer.
Earlier Bad prefix and `imu-uart-retry-20261011/RESULT.md` preparation abort remain
failed historical evidence, exact restoration verified. StageA/B and391/306 dropout
bytes unchanged. No high-rate/VIO PASS or relaxed200Hz/source/tail/clock/model gate;
VIO-S0/P PARTIAL, StageC closed. Not yet committed/pushed. Raw local test/fixture
evidence in the new task folder is not automatically part of the curated Git subset.

## Included

- Stage A/B report, heartbeat and synthetic ODOMETRY helpers + unit tests.
- Passive IMU decoder/receiver, timestamp boundary and rate experiment reports.
- Integral diagnostics, fixed PX4 message definitions, observation-model tests.
- Strict finite ULog-over-MAVLink reassembly, synthetic tests, saved replay script,
  validation report and not-yet-executed trial proposal.
- Single-owner start/ACK/stop session controller, bounded Linux UART runner and
  capture audit: revised startup path offline/virtual-PTY validated; prior physical
  startup failure retained, no completed high-rate capture.
- Selected offline VIO timestamp/FIFO source, OpenVINS diagnostic patches/reports.
- Windows handoff/proposed-Vault-update archives. No Vault write is claimed.

## Offline tests (no hardware)

Existing research environment used pymavlink 2.4.49, pyulog 1.2.4 and NumPy 2.5.3.
These are research requirements, not changes to the root production requirements.
Use an already compatible Python environment; don't install onto a device by
running this guide. `python` below denotes that environment.

```sh
python -m unittest discover -s .maixpy/m3c-ping-20261009 -p test_heartbeat.py -v
python -m unittest discover -s .maixpy/mavlink2-20261010 -p test_odometry.py -v
python -m unittest discover -s .maixpy/imu-input-20261010 -p test_imu_input.py -v
python -m unittest discover -s .maixpy/imu-design-20261010 -p test_offline.py -v
python -m unittest discover -s .maixpy/imu-design-20261010/followup -p test_model_review.py -v
python -m unittest discover -s .maixpy/imu-design-20261010/single-uart/reassembly -p test_ulog_mavlink.py -v
python -m unittest discover -s .maixpy/imu-design-20261010/single-uart/controller -p 'test_*.py' -v
```

The `.msg` files are small definitions from the audited PX4 revision, required by
the layout tests. Synthetic BMI088 IDs are fixture labels, not a real capture.
Tests do not start the hardware scripts, and passing them does not qualify VIO.

The finite reassembler returns ACK *intents*: no serial IO. The new controller and
runner are offline/PTY validated; physical launch remains approval-gated. See
`imu-design-20261010/single-uart/controller/VALIDATION.md` and `RUNBOOK.md`.
Hardware helpers open/configure UARTs only when explicitly launched;
their presence is not permission to execute them or rerun completed Stage A/B.

## Not included / local-only evidence

Raw ULogs, UART captures, full console/JUnit logs, public camera/IMU datasets,
trajectories, PDFs, credentials, virtualenvs, wheels, vendor trees and builds stay
outside Git. Reports identify them and their checksums where available.
`replay_saved.py` requires the exact two local ULogs at its documented paths;
it cannot reproduce that saved-file result from a fresh clone alone.

Older reports link to such local-only artifacts. Their manifests describe the
original experiment directories, **not this published subset**; no claim that a
clone contains every manifest entry is made. Use the connection/resume documents
as the portable current summary, rather than running old capture/restore scripts.
