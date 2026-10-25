> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

# Stage 3 Batch B — Software Completion Snapshot

**2026-10-14: ALL B1–B5 software gates accepted.**

Source branch: `sara/can-replacement`; final tested source revision:
`26217a6ce44bdd1b941edefcdb1bee98ae54e003`.
The user reported successful CM7/CM4 CubeIDE target builds and pasted
a successful full consolidated B5 acceptance transcript.

| Batch | Scope | Software evidence |
| --- | --- | --- |
| B1 | M7 PathExecution and motion binding | PASS |
| B2 | Shared SRAM4 linker reservation and ABI | PASS |
| B3 | Validation SPSC/worker and fail-closed startup | PASS |
| B4 | M4 storage/readback-to-trajectory adapter | PASS |
| B5 | M4 HMI lifecycle and B1–B4 consolidated run | PASS |

Detailed operator evidence: `STAGE3_BATCH_B1_ACCEPTANCE.md`,
`STAGE3_BATCH_B3_ACCEPTANCE.md`,
`STAGE3_BATCH_B4_ACCEPTANCE.md`,
`STAGE3_BATCH_B5_ACCEPTANCE.md`.

**Not yet completed:** Stage C software-system qualification, powered
hardware tests, physical storage HAL provider, drive and safety
commissioning. This is a software baseline, not a production robot release.

## Stage 3 software final qualification (2026-10-14)

B5 was followed by independently accepted C1, C2 and C3 software gates.
The operator's final C3 script passed at source revision
`8c387c9cb09ede9c00ed07896d8d9ddc71844677`: fresh CM7/CM4
images, nested B/C regressions, 19/19 portable host tests and the
candidate evidence manifest. See `STAGE3_C3_ACCEPTANCE.md`.

This closes **Stage 3 software qualification only**; the physical
hardware/safety commissioning H0–H10 remains open and the binaries
are not authorized to control an energized robot.
