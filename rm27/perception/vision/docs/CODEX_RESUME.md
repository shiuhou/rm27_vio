# RM27 VIO resume — 2026-10-13

## Current candidate work

SchurVINS is now authorized as the first RK3576 VIO candidate; OpenVINS
remains the preserved comparison baseline. Use the independent SchurVINS
entry and [integration record](../../../../.agents/docs/SCHURVINS_INTEGRATION.md).
Historical PX4/M3C logging failures do not block the new independent IMU.
The older scope and evidence below remain specific to those experiments.

## Scope

Work only on VIO in main. Historical monocular SLAM code/reports are archived
on `legacy/slam` at74d44b0, not an enabled fallback backend.
Do not implement mapping, loop closure or another SLAM experiment here.

## Read first

1. Root [HANDOFF](../../../../HANDOFF.md) and [VIO plan](../../../../new_plan.md).
2. [Data contract](RM27_VIO_DATA_CONTRACT.md), [sensor audit](IMU_SOURCE_AUDIT.md).
3. [Connection](PX4_M3C_CONNECTION.md), [workspace locations](WORKSPACE_LOCATIONS.md).
4. [Bench index](../../../../.maixpy/README.md) and the specific dated result,
   not an old coordinator command.

## Current boundary

VIO-P PARTIAL: finite reliable-input candidate replays were consistent, without
claiming every default/asynchronous configuration is deterministic.
VIO-S0 PARTIAL: high-rate completeness, >=200Hz, observation model and camera/
FC clock mapping remain unqualified. Stage A/B is transport/synthetic-only;
Stage C/calibration/M3C estimator port/EKF fusion/flight are not started.

Latest [RAM diagnosis](../../../../.maixpy/imu-ram-run-20261012/RESULT.md):
logger Wait10443us, two publications1498us apart, depth-one overwrite, exact
missing ULog integral4507us. Corresponding link bracket has no rejection/gap;
separate startup loss/rejection retained. Exact Wait subphase still unresolved.
Runtime settings restored,961 typed parameters unchanged; diagnostic firmware
remains installed with both trace writers frozen. Keep disarmed/do not fly.

Next proposed task: offline design of loss-accounted queued logger acquisition.
Do not repeat the consumed trace/capture, reflash, restore an unverified image,
change rates/math/gates or start calibration without separately reviewed scope.
Raw evidence is local/M3C/research-host data, not automatically in a clone.
