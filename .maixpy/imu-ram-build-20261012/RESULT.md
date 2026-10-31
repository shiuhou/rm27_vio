# RAM diagnostic full board-build / official local image — 2026-10-12

## Outcome

**VERIFIED:** full MicoAir743v2 OFF and ON firmware builds and link maps; native
PX4 atomic port host tests; official release file copied locally and hash checked.
**PARTIAL:** diagnostic firmware qualification. **UNKNOWN:** original logger skip
trigger, runtime overhead/stack/heap margins and hardware health. VIO-S0/P PARTIAL,
Stage A/B preserved, Stage C CLOSED. NO flash, M3C/PX4 access, capture, parameter /
baud / wiring / rates / queues / control change, ARM/fusion, push or Vault access.

User `do it` continued preparation; then specifically requested official firmware
locally. Neither supplies a current flash binary backup or authorizes substitution
of this newly downloaded reference for an original backup. Fresh backup and final
bounded flash/test approval remain prerequisites. No hidden physical experiment.

## Official firmware saved locally

`C:/Users/USER/OneDrive/RM27/drone/firmware/PX4-v1.17.0/micoair_h743-v2_default.px4`
with adjacent provenance README. 1,775,740bytes; SHA256:
`68caf5535984deb487eb00b07acfff816df0616c33f076e9c0e2a8b650d59c0b`.
Matches [official release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0)
asset digest. Package board1179/revision0; decompressed1,903,424bytes, embedded
PX4d6f12ad1c4f70ad3230afd7d86e971421e02fef4 and
NuttXfb2fadf6f599c1406f052db013efd00a2518e72c. `inspect_reference.py` verifies bytes,
package size and embedded hashes (`official-reference/inspection.json`). This is
official stock, NOT diagnostic firmware or a readback of current FC flash/params.

## Actual source/build environment

Windows main74d44b09c65e2a839b74d834756f0d178207d679 plus preserved dirty worktree.
Full clean pinned source, release tag and39 recursive gitlinks obtained in NEW
research-host directory (NOT M3C):
`ceo5060:/home/shiuhou/Projects/rm27-vio-20261007/px4-ram-build-20261012.HURcNV/`.
Source `PX4-Autopilot/`; SDK `toolchain/`; first failed `images-01/`; final
`images-02/`. GNU ARM13.3.Rel1 Linux SDK checksum verified against official digest:
`95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4`.
No sudo/apt/pip/global configuration or production dependency installation.
Existing Python3.12.3/empy3.3.4/kconfiglib14.1.0/CMake3.28.3/Ninja1.11.1 reused.

Existing sealed build_candidate.py reused byte-identically. `preflight-04.json`:
ready true, matching full HEAD/submodules, flash_authorized false. CMake board
`-DCONFIG=micoair_h743-v2_default -DRM27_IMU_TRACE=OFF/ON`, Ninja `-j4`, separate
external output directories. `board-build-04-console.txt` BoardBuild04Exit0;
full commands/artifact hashes in `download/images-02/build-manifest.json`.
This preserves a repeatable pinned-input build PROCEDURE, not a proven byte-for-
byte reproduction guarantee. Two OFF builds in different build directories have
different package hashes (`rebuilt-control-comparison.txt`); the first is retained,
not substituted for the reviewed final control. Exact cause of that byte difference
has not been analyzed, and no bit-identical reproducibility claim is made.

## Evidence-backed integration fix (NOT the logger-skip fix)

Candidate03 host/standalone tests missed NuttX's actual `-nostdinc++` environment:
OFF build succeeded, ON failed at missing `<atomic>`. Pinned NuttX also lacks
`<limits>`. New overlay uses existing exact PX4 atomic.h (SHA256
`cadfc56fb520bae044de805ac9516287c2f24d9355c349f0fa375fe759d05032`) and UINT64_MAX.
`px4::atomic` supplies sequentially-consistent operations, stronger than previous
release/acquire; explicit lock-free32/bool static assertions forbid its IRQ-lock
fallback for our types. All ring/trigger/tail/STOP/epoch/partial-arm semantics remain.
No algorithm, queue, priority, sensor rate or schema change. Original candidate03,
sealed trace folder, failed OFF/ON logs and objects retained. Only task-owned patch
was reversed in the dedicated clone after checking ALL9 before hashes; its FOUR
added library files remain recoverable in generated-candidate-03/tree. No user
source removed. Candidate04 regenerated through original guarded patch generator;
latest patch SHA256 `47b57299b1181b68b6c6692dac0572033aafead67590c7d27599f1160fe47ef6`.

## Actual images and static memory review

| Artifact | SHA256 | Image bytes | AXI SRAM static used |
|---|---|---:|---:|
| OFF rebuilt control | fda03caab8c2b126ad4702adffe4d21dd6e2bb7397dd47f0de79fb97be0d714b | 1904664 | 52088 |
| ON diagnostic | 21f82789fb089c99f3e097c361d1fd1b7e6ffe83f55cea46505709ac512591d9 | 1909796 | 74936 |

FLASH capacity1,966,080B: ON margin56,284B; static AXI capacity524,288B:
ON used74,936B, delta22,848B. These are linker/static quantities, NOT free runtime
heap/task allocation proof. ELF data delta4B/BSS delta22,848B are separate section
totals; do not conflate them with the arrays/context budget reported by trace.
OFF has ZERO rm27_trace symbols; ON actual ELF has trace/command/hook symbols,
single trace object22,648B. Arrays/contexts remain bounded below24KiB.

**Package git_identity still says v1.17.0 in BOTH images. Do not call ON stock or
authenticate it using ver all alone.** Identity is BASE + candidate04 patch SHA +
actual image SHA; diagnostic console command presence is supporting evidence only.
OFF is a newly built control, NOT the official file/current flash backup.

`review_build.py` exit0 (`board-review-05-console.txt`): exact BOARD compile argv
reused with -fstack-usage, no synthetic HAL. Core undefined references have no
allocation/lock/libatomic helpers. Static frames: core trigger20/logger56/IMU56B;
wrapper copy104B, phase<=24B, command792B (inlined dump). Config logger3700B,
NSH2048B; full call-chain/runtime highwater UNKNOWN. Physical first boot must
inspect resource margins before trace activation; stack/overhead risk not waived.

## Fresh tests, transfer and preserved failures

Windows `py -3 -B -m unittest discover -s .maixpy/imu-ram-build-20261012 -p 'test_*.py' -v`:
7 methods PASS/zero skips; exact PX4 atomic core8 synthetic cases and frozen-dump
roundtrip reused, plus five SDK assembly guard tests. Linux `run_bridge_host.sh`
exit0: core8 cases/wrapper and ASAN+UBSAN core/wrapper PASS; SYNTHETIC_ONLY.
TSAN was unavailable in prior sealed host environment and is NOT newly claimed.
Host append benchmark is NOT FC execution/latency/overhead evidence.

Remote/local artifact archive SHA256 agrees:
`2536023dc33f6cd499f911e6ddb2b070175767431cc0d9f015359f5fae484d85`.
Actual .px4/.elf/.map/build logs/stack objects retained in download/. No datasets
redownloaded, old manifests/raw data rewritten or physical test reinterpreted.
Prior trace125 own entries verified before/after; original four experiment own
manifests/raws/Stage A/B retained. Root report before copies in repo-before.
An independent readiness handoff paragraph appeared during this task; it was read,
preserved byte-for-byte in HANDOFF-concurrent-readiness.md and retained in root.
That other turn's saved-source rerun is not claimed as this task's new test result.
The concurrent Vault proposal append also changed document line endings; original
VAULT_UPDATE bytes remain archived, all old prose is checked after newline decoding,
and the added readiness note is retained. No raw sensor/source bytes are normalized.
The first seal correctly rejected a byte-identical current-body claim for that
document; the final seal reports the actual narrower document-text preservation.

Failures retained: initial gh jq/SSH quoting and git -C relative patch lookup;
slow SDK transfer intentionally terminated preserving44,351,488B prefix, first
eight-range attempt timed out, only18,825,281 missing bytes fetched into NEW tails;
all parts retained, assembled SDK exact checksum PASS. Initial missing bridge RED;
candidate03 full ON compile failure; first stack review used wrong .config path,
corrected generated px4_boardconfig.h in NEW review-05, old review-04 untouched.
No repeated hardware capture or simulated source PASS. Skills kept RED/GREEN,
actual board-vs-host distinction and explicit preflash checkpoint.

## Remaining / smallest next experiment

Original logger skip cause still UNKNOWN. No hardware-fault/health finding.
Review [FLASH_APPROVAL.md](FLASH_APPROVAL.md): select this verified official image
as rollback candidate OR supply original flashed file; obtain current full typed
parameter/config backups; approve final image/scope, then ONE <=60s disarmed RAM
trigger experiment using PX4 USB + existing M3C UART. No live sensor_gyro observer,
rate/queue/control change, fusion, calibration or Stage C. No flash/test here.
