# RM27 VIO scope and plan

Updated 2026-10-13: the user requested VIO-only main and archival of the former
SLAM line. The original mixed roadmap is retained on `legacy/slam` at74d44b0;
this page is the active scope index, not a replacement of measured acceptance
thresholds or a declaration that planned gates passed.

## Authorized SchurVINS / RK3576 candidate work

SchurVINS with its included SVO2.0 frontend is the first RK3576 VIO candidate;
OpenVINS remains the comparison baseline. The independent SchurVINS runtime
entry is validated before any general-runner integration. Current work is
public-data PC reproduction/comparison plus ARM64 and UZH-FPV feasibility;
no SLAM mapping, loop closure, hardware capture, firmware or flight.
New independent IMU qualification does not depend on resolving historical
MicoAir/PX4 logger failures. See the single
[integration record](.agents/docs/SCHURVINS_INTEGRATION.md) for actual results
and [tools](tools/schurvins/README.md) for executable commands.

## Historical OpenVINS / PX4 line

OS04A10 Camera + verified PX4 IMU -> evidence-preserving acquisition ->
offline OpenVINS baseline -> existing LocalizationEstimate adapter.
The current priority is qualifying complete, correctly timed IMU input.

Only OpenVINS is enabled in the main runner. No monocular SLAM launcher, map/
loop-closure experiment or ORB-SLAM3/stella_vslam backend is developed here.
SchurVINS is now separately authorized above. Physical calibration, EKF fusion
and flight still require their own scope; historical M3C deployment is not
the new platform qualification.

## Unchanged requirements

- Preserve acquisition timestamps, clock identity, units, axes, device identity,
  integrals versus instantaneous samples, and every observed gap.
- Original camera target20–30Hz and IMU target200–400+Hz remain goals.
  Retained196–197Hz integrated publications do not satisfy the >=200Hz gate.
- Original1cm public-baseline consistency criterion remains unchanged.
- No silent interpolation, retiming, bias removal, scale fitting or replacement
  of an unsuccessful experiment with synthetic evidence.
- A real paired dataset requires exposure semantics, clock mapping and uncertainty,
  rigid geometry, spatial/temporal calibration and an appropriate observation
  model before estimator/fusion qualification.

## Current evidence, not broad acceptance

VIO-P PARTIAL: finite reliable-input PC candidate runs were consistent; original
asynchronous failures remain preserved. VIO-S0 PARTIAL: raw export and transport
are measured, but complete high-rate input/model/clock qualification is unresolved.
Stage A/B is transport/synthetic-only evidence; Stage C remains closed.

Latest physical RAM trace locates one logger Wait/depth-one overwrite. Runtime
settings were restored, diagnostic firmware remains installed. Do not fly it.
Next proposed VIO work is offline review of loss-accounted buffered acquisition;
a firmware change, another capture or recovery needs separately scoped authority.

See [HANDOFF](HANDOFF.md), [data contract](rm27/perception/vision/docs/RM27_VIO_DATA_CONTRACT.md)
and [resume](rm27/perception/vision/docs/CODEX_RESUME.md) for evidence and boundaries.
