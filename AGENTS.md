# RM27 VIO repository scope

## RK3576 reference documents

For hardware or deployment work in the RM27 parent checkout, first read
`../../docs/REAL_HARDWARE.md`, section "RK3576 vendor references (DDR5 only)";
raw manuals are in `../../RK3576/`. Use only LPDDR5 CV00 / MV12 references,
not LPDDR4 variants. These local manuals are not bundled with standalone clones;
if absent, report the missing references and do not guess board-specific values.
The selected documents do not establish the installed revision or hardware qualification.

## PDF-aligned current scope (2026-10-30)

Research source: `RM27 无人机组开发文档 (2).pdf`, SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`. Current goal: repeated fixed-base underside contact,
separation, attitude/estimation recovery and target reacquisition in one sortie.
RK3576 is the new platform direction; its full hardware/data chain is not qualified.
Return-to-dock and precision landing below belong to historical baselines, not the
new season's mandatory deliverables. Traditional/RL allocation remains a research
choice. Detailed algorithms/hardware tasks are scheduled separately by the user.

Component responsibility: Continuous camera–IMU state estimation, timing and calibration. OpenVINS remains the current implementation baseline; new RK3576 data qualification is pending. Historical MicoAir logging failures remain evidence, not a permanent dependency of a new independent IMU.

The authorized cross-repo interface batch is synthetic/offline only, with no
learning, firmware execution, hardware operations or Git publication. The root
repository coordinates the single activity plan and derived report. Component
contracts remain self-contained; the shared engineering method is unchanged.

- Main is for Camera + IMU VIO only. OpenVINS is the preserved baseline;
  SchurVINS is the authorized first RK3576 candidate through an independent
  experimental entry point. Do not merge it into the OpenVINS runner implicitly.
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
