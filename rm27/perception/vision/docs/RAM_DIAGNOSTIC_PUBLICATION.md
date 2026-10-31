# Historical RAM diagnostic evidence publication — 2026-10-31

This publication adds the retained 2026-10-12 trace preparation, firmware build,
flash and bounded observation RESULT reports. They describe historical actions,
not current device state, fresh hardware validation or authorization to repeat them.

## Read in order

1. [Offline trace preparation](../../../../.maixpy/imu-ram-trace-20261012/RESULT.md).
2. [Diagnostic firmware build](../../../../.maixpy/imu-ram-build-20261012/RESULT.md).
3. [Historical flash result](../../../../.maixpy/imu-ram-flash-20261012/RESULT.md).
4. [One bounded RAM observation](../../../../.maixpy/imu-ram-run-20261012/RESULT.md).

The observation locates one logger Wait/depth-one subscription overwrite upstream
of UART delivery. It does not isolate the poll/scheduling/notification cause of
the Wait, fix data loss, establish hardware health or qualify VIO. All original
>=200 Hz, completeness, clock and observation-model gates remain. S0/P PARTIAL;
Stage C closed. Diagnostic firmware was still installed at that historical
readback; no claim about the board's current firmware is made here.

## Published versus local-only

Only the four original RESULT.md files and existing source-audit/index notices
are published. Their bytes and historical wording are retained. References to
analysis JSON, traces, source overlays, firmware, parameters, approvals, helper
tests and raw captures describe local artifacts NOT included in this PR.
A clone therefore cannot replay or independently verify that physical experiment.
Do not infer approval from an old report or execute a dated coordinator.
No firmware/parameter backup, binary, raw UART/ULog, credential or device helper
is added. This is documentation publication, not a diagnostic firmware release.

## This publication's verification

Base remote main: 2cd05cc0b342dcb76be5a17db59012c9185c0e32.
Fresh command from repository root, with plugin autoload disabled:
`py -3 -B -m pytest tests -q --ignore=tests/camera_capture.py`.
Observed 148 passed, 1 existing skip, exit 0 (4.34 s). The untracked camera script
is unrelated hardware code and is not collected, modified or published.
No new physical tests or re-execution of old diagnostic tests occurred.
No dependency, estimator, source rate, queue, firmware or parameter changed.

Untracked task packets, stale implementation plan and tmp material remain local;
they are not silently staged. Existing raw evidence is not changed or deleted.
