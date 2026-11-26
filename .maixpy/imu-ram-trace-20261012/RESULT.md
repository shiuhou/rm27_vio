# VIO-S0 trigger-frozen RAM diagnostics — 2026-10-12

## Outcome

**VERIFIED offline:** bounded C++ collector, exact-source diagnostic patch,
STOP-only dump validator, host/sanitizer tests and Cortex-M7 object compilation.
**PARTIAL:** firmware integration preparation. **BLOCKED:** full board build,
link-map/stack review, identified original image backup and fresh session parameter
backup. **UNKNOWN:** actual intermittent logger scheduling/skip trigger and FC
hardware health. VIO-S0/P PARTIAL; Stage A/B preserved; Stage C CLOSED.

No physical-device access/capture, flashing, parameters, baud/wiring/rate changes,
ARM, fusion, camera/calibration, dependency install, Git stage/commit/push or Vault
access. Only Windows and isolated existing Linux RESEARCH HOST SSH software tests.
Rootmain74d44b09c65e2a839b74d834756f0d178207d679; unrelated PLAN_HISTORY.md,
camera_capture.py and tmp edits preserved. New research folder is locally ignored;
no publication is implied. Engineering/debug/TDD/verification/handoff skills kept
RED/GREEN evidence and approval boundaries; no Vault templates read.

## Existing evidence reused (not a new run)

Saved selected BMI0880 vehicle_imu-only reduced run:11843 records197.215284Hz,
one4554us gyro coverage residual, logger counter1→2, observed transport clean.
Fixed-reader control remained197.104634Hz/two skips, not a fix. Later gyro observer
failed at startup with one verified sender TX rejection. Neither exposes the
missing Run/copy timing; queue1 overwrite mechanism is confirmed, its trigger is
UNKNOWN. No historical BMI270/v1.15.2 sample used as selected BMI088 evidence.

Exact pinned source and old diagnostics reused; `prepare_patch.py` checks stock
SHA256 and unique anchors. Three missing CMake files + four build/interface files
were retrieved from official immutable d6f12ad1 URLs, not moving branches. Sources
and hashes retained in source/. Official [logger build definition](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/CMakeLists.txt)
and [uORB interval subscription](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/SubscriptionInterval.hpp)
confirm target linking/available generation and instance methods.

## Actual implementation

DESIGN.md specifies two single-writer arrays, first-copy baseline exclusion,
unsigned32 generation trigger, release/acquire initialization and owner-only
freeze/STOP handshake, per-writer FOUR-record tail, one arm per boot, source
identity/calibration/lifecycle invalidation. 256×48B logger +80×128B IMU records.
No printing/locks/allocation in hooks, no live UART diagnostic topic. Logs actual
gyro/accel internal endpoints and own integral dt, not retimed5ms samples.

`generated-candidate-03/diagnostic.patch` and manifest contain nine affected files;
all stock code outside diagnostic spans byte-identical. `decode_trace.py` checks
finite output counts/checksum/schema/chronology and refuses incomplete evidence.
It never emits an automatic hardware-root-cause or OpenVINS PASS. Approval scope
and build/backup/rollback prerequisites are in [APPROVAL.md](APPROVAL.md).

## Fresh tests / compile evidence

```powershell
py -3 -B -m unittest discover -s .maixpy/imu-ram-trace-20261012 -v
.maixpy/imu-offline-20261008/venv/Scripts/python.exe -B -m unittest discover -s .maixpy/imu-source-diagnosis-20261011 -p test_diagnosis.py -v
```

New7 unittest methods PASS/zero skips, including actual C++8 core cases, wrapper/
guard/dump roundtrip with LABELED SYNTHETIC HAL bindings, checksum/truncation
rejection, exact patch application/stock recovery, hash tamper rejection,
fail-closed missing-source build preflight and Cortex-M7 object compilation.
Saved-source19 regressions PASS/zero skips, including ONE real pinned ULog replay
test; this rereads existing data, not new capture. Broader production/controller/
Stage A/B tests were not rerun or claimed as new results.

Linux research host ceopc isolated copy:
`/home/shiuhou/Projects/rm27-vio-20261007/ram-trace-20261012.omfedw/midarm/`.
`bash run_host.sh` exit0: actual8 core cases and wrapper dump roundtrip PASS;
AddressSanitizer+UndefinedBehaviorSanitizer core/wrapper exit0. ThreadSanitizer
**UNAVAILABLE**, exit66 unexpected memory mapping (also -no-pie attempted,
same failure). Overall script0 does NOT make TSAN pass. No board test/deployment.

Cortex-M7 ARM GNU13.3.Rel1 standalone objects compiled -Wall/-Wextra/-Werror,
no exceptions/rtti. Core `.bss`22648B; stub-bound wrapper `.bss`172B; total22820B,
below24KiB. No allocation/lock/libatomic core undefined symbol; memcpy only.
Standalone stack record: core logger64/imu56B, wrapper copy96/phase<=112B,
STOP-only dump696B before library internals. This is NOT whole task stack margin
or full firmware linkage. Host-only append benchmark ~6–14ns per append (native
observations), NOT FC timing/WCET or a performance guarantee. HRT hook samples
would measure only a limited portion of hook work, not total instrumentation.

## Build / backup status and failures

`build_candidate.py` read-only preflight exit2 on Windows and Linux, saved JSON:
flattened source is not full Git/submodule checkout. Windows has ARM toolchain,
but lacks em/kconfiglib in system Python; Linux has build Python dependencies but
no ARM toolchain. Full `.px4/.elf/.map` OFF/ON build NOT completed, no install or
source checkout downloaded. Exact build helper/flags/provenance path prepared,
but neither reproducible firmware image nor original binary backup is claimed.

Historical user parameter files remain unchanged and hash-inventoried; not used
as current restore targets. A fresh parameter backup must precede approved flash.
No original board firmware image found in inspected locations; source hash and
a future rebuild cannot substitute for the file actually used for rollback.

Retained failed helpers/tests: initial RED missing collector/integration/validator/
build helper/lifecycle; two nonunique source anchors rejected before generation;
formatter field count and nested span chronology caught by roundtrip; first patch
fixture hit global Windows CRLF conversion, fixed test invocation to per-command
core.autocrlf=false (original research bytes untouched); ARM compile caught
uint32_t format ABI mismatch, fixed explicit formatting casts. Original failed
logs and generated snapshots are retained, never silently replaced. TSAN failure
and filesystem lookup misses preserved/reported; no hardware retry.

Final review added a RED/GREEN regression for arm landing mid-Run/loop: partial
producer and unmatched phase spans are now omitted until a captured begin. A failed
Windows RED executable was held open (termination access denied); the test now
uses isolated per-run output rather than overwriting that artifact. Actual wrapper
and core still pass in Windows and Linux; tests-midarm-green-03.txt and
linux-host-midarm.txt supersede earlier candidate test snapshots. No device-side
timing or original logger-skip cause is inferred from this software-only fix.

`seal.py` validates current candidate against pinned inputs, all own-file entries
of the four reused experiments' manifests, original raw/Stage A/B hashes, exact
before-report bodies, historical parameter file hashes and transferred research-
host test bytes. `preservation-check.json` and `manifest.json` record this task's
snapshot. Old external manifest entries describe their historical report revision;
they are not falsely claimed to match newly supplemented root reports. No prior
manifest or old experiment file is rewritten. New native host executables are
excluded from the evidence inventory; compile/test logs and ARM objects retained.

## Next / approval boundary

No verified new root cause. Current finite logs cannot reconstruct the omitted
publisher integral or exact task execution; don't infer them or hardware failure.
Finish full pinned board build/memory review and identify rollback image/current
backup, then present image hashes for approval. Exactly ONE conditional next
hardware experiment is specified in APPROVAL.md: original clean vehicle_imu-only
UART profile plus RAM trigger capture, maximum60s, retrieval AFTER STOP and rollback.
Not authorized/executed here; failed high-load coordinator must not be rerun.
