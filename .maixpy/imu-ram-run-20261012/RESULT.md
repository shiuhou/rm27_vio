# One physical PX4 RAM-trigger diagnosis — 2026-10-12

## Result

**One logger subscription skip is dynamically located; no loss fix is claimed.**
The retained RAM timeline shows two consecutive `vehicle_imu0` publications
during a logger Wait phase. Its depth-one subscription subsequently reads the
newest generation and misses the intermediate publication. That exact
publication is absent from the received ULog; the corresponding integrated
gyro coverage hole is 4507 us. The containing MAVLink counter interval has no
rejection, sequence gap or unaccounted successful send. This event is therefore
upstream of UART delivery, not attributed to a broken wire or SD write.

The exact reason for the long Wait is **not isolated**: that instrumentation
span encloses poll, possible scheduling delay and notification-clearing
`orb_copy`. No scheduler/individual poll-return trace was collected. This is
not proof of global hardware health or the cause of every historical loss.
VIO-S0/P remain **PARTIAL**, Stage C closed; all original gates remain unchanged.

## Authority, baseline and provenance

The user's `ok do it` approved ONE bounded RAM observation after the diagnostic
flash, not a new firmware repair, repeated capture, calibration or fusion task.
Props removed/disarmed was freshly confirmed for the preceding flash and remains
the agreed bench condition. Props are user-confirmed, not remotely inspected.

Repository: `main`, HEAD `74d44b09c65e2a839b74d834756f0d178207d679`, dirty research
worktree retained. No Git stage/commit/push or Vault action. Exact firmware is
candidate04 **BASE d6f12ad1 + diagnostic patch**, NOT stock despite its v1.17.0
version label; ON image SHA256:
`21f82789fb089c99f3e097c361d1fd1b7e6ffe83f55cea46505709ac512591d9`.
Fresh `ver all` reports MICOAIR_H743_V2, Oct 12 2026 14:02:00/GCC13.3.1, PX4
d6f12ad1 and NuttX fb2fadf6. Selected BMI088 instance0 is gyro6684690/accel6946834;
do not substitute the historical BMI270 instance1 evidence.

New M3C directory: `/root/rm27-ram-trial-20261012.1LpzrU`, source hashes matched
the frozen physically-tested reduced-telemetry controller archive. It is
intentionally **not** an assertion that the current dirty local uart_session.py
is identical to that deployed revision. Existing Python/NumPy/pymavlink/pyulog
were reused; 13 fresh board helper tests passed, zero skips. SSH ran only scoped
shell helpers; no MaixPy dev-mode/app-stop operation or dependency installation.
USB COM19 is the independent control link, not the physical UART baud.

## Bounded operation and preserved data

Fresh preflight verified Disarmed, EV_CTRL0, idle logger, trace unused, no UART
owner, current configuration and absence of a custom SD logger path. Backups
and actual commands/outputs are in `coordinator.jsonl`, board preflight and the
post-flash typed parameter snapshot. Temporarily used:

```text
mavlink start -d /dev/ttyS2 -b 921600 -r 46080 -m onboard -Z -x
logger start -b 64 -t -m mavlink -p vehicle_imu
```

Custom eight-topic/166-byte file, verified by download:

```text
vehicle_imu 0 0
vehicle_imu_status 100 0
sensor_selection 100 0
timesync_status 100 0
logger_status 1000 0
cpuload 1000 0
vehicle_status 1000 0
actuator_armed 1000 0
```

Reduced finite telemetry kept HEARTBEAT1, SYSTEM_TIME1, TIMESYNC10,
LINK_NODE_STATUS1, SYS_STATUS5, STATUSTEXT20; existing request-driven streams
remained. Single-owner receiver performed guarded heartbeat-only MAVLink2
bootstrap, START/ACK/STOP. No high-rate sensor observer, sensor rate/integrator/
queue, parameter, EKF, wiring, camera or control change was made.

RAM was armed once after normal packets had streamed >=1 s on the M3C
monotonic clock. Stop was requested 58.0 s after arm-return (Windows monotonic),
within the approved 60 s. This is the measured host control bound, **not** an
exact FC-time recording window. Receiver exited0 without supervisor timeout.
STOP returned both writer acknowledgements:
`active=1 logger_frozen=1 imu_frozen=1 reason=1 dump_ready=1`.
The active latch remains1 by design; both writers are frozen. Logger was then
stopped before two dumps. Both 26,889-byte raw dumps are byte-identical, and the
26,862-byte canonical dump passes its checksum. Finite ring overwrites
41086/11067 describe intentional pretrigger retention, not sensor loss counts.

Raw prefix and `framing-valid.ulg`: 939,013 bytes, byte-identical, SHA256
`0fec9bb13c529e9ab245f9151c7a693b31280dc67e7cdd9f46b832112e1e5844`.
A fresh M3C `sha256sum` matches the local copy. No raw repair, insertion,
interpolation or retiming. Raw wire bytes/metadata and original dumps retained.
3772 logging packets, zero reassembly retransmissions/errors; START/STOP accepted.
Finite framing and parsing are verified; startup/pre-START and final tail
completeness remain unqualified. Receipt duration is not a camera/IMU clock map.

## Trigger timeline: direct source and consumer evidence

All following times are microseconds in the same FC HRT domain. Row fields are
from `trace-canonical.txt`, independently decoded in `analysis-reviewed.json`.

| Event | FC HRT us | Evidence |
|---|---:|---|
| Previous logger copy ends | 4145334796 | generation815904, publication4145334684 |
| Wait begins | 4145334805 | logger loop8267 |
| Intermediate publication | 4145343628 | source ordinal8268, gyro dt4507us |
| Newest publication | 4145345126 | source ordinal8269; separation1498us |
| Wait ends | 4145345248 | duration10443us |
| Next copy begins/ends | 4145345254/5255 | 815904 ->815906, missing815905 |

Both intermediate and newest publications occur inside Wait. The next copy is
1626us after the intermediate publication,128us after the newest. Source ordinals
and gyro integral coverage are continuous in the retained trigger span; source
gyro/accel generation-gap flags are zero and identities/calibration counters are
unchanged. Per-Run sample counts are not total sample counts of an accumulated
integral; zero gyro reads in one publishing Run do not establish a sensor failure.
Maximum retained logger lock wait is1us, VehicleIMU Run wall time191us. These
bounded observations do not establish whole-run CPU/heap/stack margins.

The absent publication timestamp4145343628 is not in the received ULog. Its
next retained record is publication4145345126, with a 4507us gyro integral
coverage hole (row8804). Logger status brackets that hole with message_gaps2->3.
The MAVLink bracket4144992865..4145993708 has76 successful sends and76 received
frames, zero rejection and no sequence gaps. Thus the queue-one overwrite is
dynamically associated with the specific missing log record, independently of
transport delivery in that interval.

### Source-code interpretation and limits

Pinned base `../px4-v117-source-audit-20261009/source/new/src__modules__logger__logger.cpp`
lines900 onward calls `px4_poll(...,20)` and then notification `orb_copy` before
the next loop. Actual candidate04 patch:
`../imu-ram-build-20261012/generated-candidate-04/diagnostic.patch` places Wait
timestamps around this entire branch and records the independent logger copy
generation cursor. The 20ms timeout is not a hard50Hz rate, and this10.443ms
span does not prove a full timeout. Poll wake, ready-to-run scheduling and
notification-copy subphases remain unresolved. No invented hardware/SD or
unordered scheduling root cause is asserted. Diagnostic hooks may perturb
timing; this is a result for this build/run, not an identical-stock timing proof.

## Whole-capture qualifications: do not hide the second hole

11788 retained vehicle_imu0 records over60.036829s of sample-time span give
196.329490Hz. The original >=200Hz gate **does not pass**. This is integrated
publication rate, not raw physical gyro/accel ODR. Records contain delta-angle
(rad), delta-velocity (m/s), separate integration durations and the gyro endpoint;
dividing by dt gives diagnostic window means, not point-sampled raw IMU or an
approved OpenVINS adapter. Accel coverage outside the RAM span lacks a separate
endpoint; camera/FC clock mapping remains UNKNOWN.

Two positive gyro coverage holes remain: startup45069us and trigger4507us,
sum49576us. Logger message_gaps1->3 gives two discontinuity events, **not** an
exact missing-sample count. Both holes have logger-counter brackets; only the
later event has this RAM trace and exact one-missing-generation attribution.
Zero ULog writer dropout events is therefore **not** evidence of loss-free input.

64 LINK_NODE_STATUS intervals contain matching successful send/receive totals.
One startup bracket4099982543..4100980023 has103 successful/received frames and
one TX-buffer rejection/sequence position11->13. Its rejected message type is
not identified; it is not a proved missing LOGGING packet, and no reassembly
packet gap is observed. This separate startup event must not be silently removed
or generalized to the later RAM trigger. The intervals before first/after last
counter snapshots are not qualified. M3C parity/frame/overrun/break/buffer-overrun
counter deltas are zero; that alone is not hardware acceptance.

M3C receiver observed CPU mean21.743% of one core over66.504s, peak sampled RSS
43,626,496B; this is sample-window accounting, not peak core use or proof against
scheduling stalls. Zero timestamp duplicate/backwards, clipping or identity/
calibration change in retained integrals; no broader sensor-health claim.

## Safety, restoration and current physical state

No arming command was sent;65 saved FC heartbeats have base_mode29/armed clear,
ULog actuator_armed onlyfalse and arming_state only1, plus pre/post commander
Disarmed. Independent USB guard checked disarmed freshness during setup/run.
These observations and user props confirmation are the actual safety evidence.

Restored exact baseline logger `logger start -b 64 -t -m all`, idle state, UART
startup command `mavlink start -d /dev/ttyS2 -b p:SER_GPS1_BAUD -m p:MAV_1_MODE -r p:MAV_1_RATE -s -x`
(115200), original stream map, other UART commands and all8SDLOG values. Removed
only our temporary166-byte logger file/two new empty dirs; its content/readback
is preserved and recoverable. Original SD logs and other files remain. SD /etc
absent again; coordinator errors/acquisition_errors empty, restoration verified.
Fresh bytewise-typed961 parameters match the post-flash backup; zero ULog changed
parameters. M3C receiver stopped, no UART owner, exact original termios restored.
Its saved B9600 speed flags are an idle OS snapshot, **not** the115200 FC setting.

**Diagnostic firmware remains installed.** No rollback/reflash/reboot was done.
The reviewed vendor recovery download is not proven byte-identical to the
preflash app (see prior flash report), so firmware recovery remains an explicit
reviewed operation. Runtime restoration is not a stock-firmware restoration.
Keep disarmed/do not fly this diagnostic build.

## Validation and retained helper failures

```text
.maixpy/imu-offline-20261008/venv/Scripts/python.exe -m unittest discover -s .maixpy/imu-ram-run-20261012 -p test_*.py -v
# 8 tests PASS; includes saved actual ULog normalization regression
.maixpy/imu-offline-20261008/venv/Scripts/python.exe .maixpy/imu-ram-run-20261012/analysis.py
# exit0; analysis-reviewed.json; no hardware access
.maixpy/imu-offline-20261008/venv/Scripts/python.exe .maixpy/imu-ram-run-20261012/verify_evidence.py
# saved evidence, hashes, bounds and restoration verification; see verification.json
```

These tests are bounded helper/replay tests, not main RM27/ROS or an IMU fix test.
Main production code is unchanged. Initial offline PyULog field conversion
failed because indexed vectors were not normalized. A RED regression was added,
then the existing inspect_imu original-vector schema was reused (no raw rewrite).
A proposed new decoder provenance label was correctly refused; its conservative
allowlist was retained, with reviewed attribution in a separate story field.
Failed console/RED records and the earlier derived analysis.json are retained;
analysis-reviewed.json adds counter brackets. See FAILURES.md.

## Next actions — proposed, not executed

Design the smallest loss-accounted **queued logger acquisition** path offline,
preserving publications before deferred write/transport. A faster loop alone is
not a loss-free guarantee for a depth-one source. Any global uORB queue change,
callback/workqueue design, firmware implementation/reflash or new bench capture
needs a separately reviewed scope and tests; none occurred here. Do not reduce
sensor/integration rates, weaken200Hz/clock/data-model gates or fill missing data.
No new physical assembly/action is needed to review the software design now.
Do not rerun coordinator.py or trace_arm; this one experiment is consumed.
