# Approved PX4 RAM-diagnostic flash — 2026-10-12

## Result and authority

**Application flash/bootloader CRC/post-boot config preservation VERIFIED.**
**IMU loss fix NOT_VERIFIED; VIO-S0/P PARTIAL.**
User explicitly said `你刷吧`, after rollback package discussion, and freshly
confirmed all propellers still removed and disarmed. Scope performed: diagnostic
application upload and read-only verification only. No logger on/trace_arm,
60s trial, synthetic odometry, parameter set/save/import/reset, sensor/queue/
math/EKF changes, calibration, M3C operation, wiring, commit/push or Vault action.
Current physical FC STILL HAS THE DIAGNOSTIC IMAGE, not restored stock. Do not
arm or fly; later bounded trace/rollback procedure remains a separate operation.

## Exact images and uploader

- Uploaded diagnostic ON candidate04:
  `../imu-ram-build-20261012/download/images-02/ON/micoair_h743-v2_default.px4`.
  SHA256 `21f82789fb089c99f3e097c361d1fd1b7e6ffe83f55cea46505709ac512591d9`,
  board1179/rev0,1909796 application bytes/max1966080.
- Vendor DFU recovery CANDIDATE retained twice,2MiB, SHA256
  `d2ebcd2a9d5af86bb3d22d2924c5f8d37bf70d48d518c4369bbf92772ca89c15`.
  It is Bootloader+Firmware.bin, NOT the uploaded diagnostic.px4 or a parameter
  backup; appropriate recovery format/selection must remain explicit.
- Pinned official uploader source:
  https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/Tools/px_uploader.py
  SHA256 `5cae28e8ebb0cdb1c25035034afdb44b2fc09f3893c32d54f2a31e96e0ac144c`.
  Imported its protocol class, NOT its broad CLI scan/reboot/GCS-UDP main routine.
  force=false/full-erase=false/boot-delay=None; application CHIP_ERASE/PROG/GET_CRC
  only, no DFU/full-chip erase or bootloader upload. Existing research Python and
  pyserial/pymavlink used; no installs.

## Fresh backup and identity

Before application overwrite: independent COM19 USB CDC, VID1B8C/PID0036,
location1-1.4/serial0, MICOAIR_H743_V2/PX4d6f12ad1/NuttXfb2fadf6, commander
Disarmed, heartbeat armed-bit clear, EV_CTRL0, logger idle/all. BMI0880 selected
gyro6684690/accel6946834. Props removed is USER-CONFIRMED, not remotely observed.

Preserved before-02-console.jsonl, complete MAVLink raw parameter frames and
typed bytewise snapshot961/961 advertised used parameters; verified capability
PARAM_ENCODE_BYTEWISE. INT32 decoded from raw payload bits, not float rounding.
Also complete `param show -a`1780 definitions, including unused-index-1 entries.
Console floats have four-decimal precision; this is NOT bytewise verification
of all unused defaults. Both SD persistent params/BSON copies1445bytes downloaded:
`sd-params.bson`, `sd-parameters_backup.bson`, SHA256
`dd1ffa9e4c4459ae3c6b47beb59c2421422456c641a602189e3813b869fb3aaf`.
SD root successful FTP/listing confirms no etc/custom startup/logger topics;
original SD logs retained. Runtime ps/MAVLink stream map/8SDLOG/CPU/free saved.
No old default/historical backup imported.

## Bootloader transition and actual write

Exactly one MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,param1=3 bootloader request after
fresh identity/disarmed/EV0/logger-idle checks. Original scanner allowed known
application USB vendors only; bootloader re-enumerated COM18 VID3162/PID004B and
was refused by that filter.30s timeout occurred BEFORE any erase/write, retained
in flash-events.jsonl/upload-console.txt. No repeated reboot/failed erase hidden.

Read-only identification of COM18 on SAME physical USB location1-1.4 showed
board1179/rev0/protocol5/max1966080, bootloaderPX4BLv1.15.4g. MCU UID words
00240024/35335116/34343432 match original PX4GUID suffix. Rechecked before write.
New exclusive evidence path continued the SAME authorized flash through this
identified endpoint; actual-upload-events.jsonl contains ONE upload_once_begin.
No force or blind port match. Official uploader completed28.156s with
program100%, verification100%, expected/reported diagnostic CRC0xeec363e0 and
normal reboot. Exit0; before-erase application CRC was0xc65b75f2.

## Post-boot verification

Returned COM19 VID1B8C/PID0036. after-01-console.jsonl verifies unchanged MCU,
PX4d6f12ad1 and NuttXfb2fadf6 hashes; build Oct12 2026 14:02:00/GCC13.3.1.
PX4 version string still1.17.0; this is BASE+PATCH, NOT stock! Reported NuttX
release metadata0.0.0 differs from original11.0.0 despite matching source hash;
do not claim identical build metadata or treat ver alone as firmware identity.

Fresh Disarmed, EV_CTRL0, logger idle/all. `logger trace_status` responds:
`RM27_TRACE active=0 logger_frozen=0 imu_frozen=0 reason=0 dump_ready=0`.
No trace record/arm; diagnostic command availability verified, not timing/overhead
qualification. BMI0880 and exact physical UART115200/stream configured rates
preserved. Fresh8SDLOG outputs same; custom SD path absent. Post BMI088 status
errors/overflows/DRDY0, reset1; counters restarted on reboot and are NOT subtractable
from the old long-runtime counters or proof of a sensor-level loss fix.

961 named typed used parameters: names/types/raw4bytes identical. All1780 displayed
definition values identical. Four before/after BSON files byte-identical1445bytes.
No automatic import/reset/restore needed. Free-memory snapshot after: UMem618032
free/largest446624; CPU snapshot0.49568. Neither certifies stack, sustained
performance, flight safety or instrumented trace timing.

## Preserved failures and recovery limitation

First read-only backup rejected unsupported UINT32 protocol hash reply; it stopped
before reboot. Added tested UINT32 handling and excluded ONLY `_HASH_CHECK` special
65535-index replies from user-parameter inventory; all raw replies preserved in
the second read. New output paths, no erased failures.6 initial/7 updated backup
tests and4 flash gate tests now11 PASS; tests are synthetic, not upload proof.
Verifier initially ignored unused negative indices and counted only961; corrected
its regex to include-1, then1780 actual entries compare equal; raw files unchanged.

Vendor current-download .bin region0x20000..end yields application CRC0x96ebbe30,
different from preflash bootloader CRC0xc65b75f2. This exploratory region comparison
assumes0x20000 application layout; it is not a whole-flash readback or source-cause
proof. Do NOT claim current downloadable vendor package equals the exact old
flashed bytes, or silently use it as an exact-clone rollback. Two candidate copies
remain recoverable; current configuration was preserved separately. Any rollback
must use explicitly reviewed format/artifact and verify post-state.

## Verification command and next step

```
.maixpy/imu-offline-20261008/venv/Scripts/python.exe .maixpy/imu-ram-flash-20261012/verify_result.py
```

Exit0; verification.json records exact claims and limits. Root main74d44b0;
unrelated existing research/report/camera_capture.py/tmp changes preserved.
Next is ONE separately scoped bounded RAM trace under existing approved-design
guard and post-STOP dump/rollback review, NOT already executed by this flash.
Original skip cause, hardware health,>=200Hz, camera clock map and observation
model remain unresolved. No acceptance threshold changed.
