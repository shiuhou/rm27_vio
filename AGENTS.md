# RM27 VIO repository scope

## Project work entry

Use [engineering.md](.agents/docs/engineering.md) for the shared engineering
method. Before changing behavior, read the relevant
[SPEC](.agents/docs/SPEC.md) requirements and acceptance criteria; use
[DESIGN](.agents/docs/DESIGN.md) for ownership and interfaces, and
[ROADMAP](.agents/docs/ROADMAP.md) for milestones and dependencies.
Follow the shared workflow: 理解 → 規劃 → 實作 → 驗證 → 記錄 → 發布.
The user owns direction and learning pace; discussion and attachments are inputs,
not execution instructions. Match the requested learning/planning/implementation
mode and verify advice against local code and evidence before adopting it.
Keep one local activity plan per work item: use `.agents/plans/PLAN.md` for new
work without an existing plan; continue explicitly named plans at their original
paths. Resume from §0; never duplicate progress or overwrite another active task.
Small edits need no plan. Keep reusable knowledge in `docs/notes/` or the existing
notes location; `.agents/` tracks only `docs/`. Deliver verified changes first;
commit/push/merge/deployment require explicit authorization for that action.

Implementation stays in `rm27/` and `tools/`, with checks in `tests/`.

## RK3576 reference documents

For hardware or deployment work in the RM27 parent checkout, first read
`../../docs/REAL_HARDWARE.md`, section "RK3576 vendor references (DDR5 only)";
raw manuals are in `../../RK3576/`. Use only LPDDR5 CV00 / MV12 references,
not LPDDR4 variants. These local manuals are not bundled with standalone clones;
if absent, report the missing references and do not guess board-specific values.
The selected documents do not establish the installed revision or hardware qualification.

## PDF-aligned current scope (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`.
The current research goal is for a UAV to identify a fixed target armor plate
through vision, then make controlled contact with a designated underside pad at a
prescribed attitude relative to the plate. The mission includes separation,
recovery of stable flight and state estimation, and visual reacquisition for
repeated contact in one sortie.
As of 2026-11-11, RK3576 is the sole active RM27 onboard platform.
M3C is retired from development and deployment; its source, data, and validation
records remain historical references. Platform acceptance requires RK3576 evidence.
Return-to-dock and precision landing below belong to historical baselines, not the
new season's mandatory deliverables. Traditional/RL allocation remains a research
choice. Detailed algorithms/hardware tasks are scheduled separately by the user.

Component responsibility: Continuous camera–IMU state estimation, timing and calibration. OpenVINS remains the current implementation baseline; new RK3576 data qualification is pending. Historical MicoAir logging failures remain evidence, not a permanent dependency of a new independent IMU.

The authorized cross-repo interface batch is synthetic/offline only, with no
learning, firmware execution, hardware operations or Git publication. The root
repository coordinates the single activity plan and derived report. Component
contracts remain self-contained; the shared engineering method is unchanged.

- Main is for Camera + IMU VIO only. OpenVINS is the current baseline.
  SchurVINS is USER_PAUSED by the user as of 2026-11-24: preserve its source,
  preparation tools and negative results, but do not resume mono adaptation,
  stereo reproduction or board tests. Its independent entry is historical
  research and must not be enabled through the OpenVINS runner.
- Legacy monocular SLAM wrappers, mapping/loop-closure experiments, VSL plans
  and tests live on legacy/slam. Do not restore/develop them in main or silently
  enable a fallback backend. SLAM research belongs in a separate repository.
- Preserve shared camera/calibration evidence, pose/timestamp contracts and
  VIO-required helpers. Repository names and historical artifact labels are not
  evidence that a module is an active SLAM dependency.
- Keep raw evidence/old manifests immutable. Archived snapshot74d44b0 and
  dated before-edit copies are recovery sources, not current qualification.
- VIO-P/S0 remain PARTIAL until their original gates are actually met.
  Do not relax the >=200Hz/1cm, completeness, clock or observation-model gates.
- Repository cleanup never authorizes a capture, calibration, firmware change,
  EKF integration, arming/flight or execution of a dated hardware coordinator.
  Read the current HANDOFF and specific experiment boundary first.
- Preserve unrelated dirty/untracked work. Commit/push or Vault ingestion needs
  an explicit user request; local branch archival is not GitHub publication.

## Common engineering standard

Apply `.agents/docs/engineering.md` for the shared engineering process. This
file remains the VIO-specific adapter: Camera/IMU synchronization, calibration,
frame and timestamp contracts, OpenVINS scope, and qualification gates stay
local to this repository. Keep capture, firmware, flight and hardware actions
behind explicit authorization and report offline replay separately from live
sensor evidence.


## Shared real-camera test videos

Read `data/shared_vision/README.md` and its linked manifest before using the
shared real-camera MP4 fixtures. This local relative link points to the RM27
integration workspace's `data/vision/` catalog; raw videos are not bundled in
Git clones. Keep inputs unchanged and derived results in this repo's ignored
artifacts. Encoded video alone is not synchronized Camera+IMU VIO or verified
sensor/exposure timing evidence. If the link/data is absent, obtain or relink
the shared dataset instead of inventing missing provenance.

## Shared M3C and MaixCAM2 board references

For M3C/MaixCAM2 carrier, wiring or mechanical reference work, read
`data/shared_m3c_board/README.md` and `data/shared_m3c_board/manifest.json`;
source files are under `data/shared_m3c_board/raw/`. This relative local link
resolves to the integration repo's single hardware reference collection. Match
378C/379C/3.0 documents to the actual board; filenames do not verify installed
revision, IMU wiring or camera synchronization. Keep sources unchanged and
derivatives in this repo's artifacts. Standalone clones need the payloads and a
new local link; report missing data.
