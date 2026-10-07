#ifndef STATE_BOOT_H
#define STATE_BOOT_H

#include "../state_machine_types.h"
#include "../../CANComm/CANopen/canopen_master.h"

#include <stdbool.h>
#include <stdint.h>

#define BOOT_EXPECTED_NODE_COUNT 6U

typedef enum
{
    BOOT_PHASE_INIT = 0,
    BOOT_PHASE_RESET_COMMUNICATION,
    BOOT_PHASE_WAIT_BOOTUP,
    BOOT_PHASE_VERIFY_IDENTITIES,
    BOOT_PHASE_CONFIGURE_HEARTBEAT,
    BOOT_PHASE_CONFIGURE_INTERPOLATION_MODE,
    BOOT_PHASE_VERIFY_INTERPOLATION_MODE,
    BOOT_PHASE_REQUEST_OPERATIONAL,
    BOOT_PHASE_WAIT_OPERATIONAL_HEARTBEAT,
    BOOT_PHASE_ENABLE_DRIVES,
    BOOT_PHASE_READ_POSITION_FEEDBACK,
    BOOT_PHASE_INITIAL_PDO_EXCHANGE,
    BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION,
    BOOT_PHASE_VERIFY_SAFETY,
    BOOT_PHASE_COMPLETE,
    BOOT_PHASE_FAILED
} BootPhase;

typedef enum
{
    BOOT_ERROR_NONE = 0,
    BOOT_ERROR_MASTER_INIT,
    BOOT_ERROR_NODE_COUNT,
    BOOT_ERROR_RESET_COMMUNICATION,
    BOOT_ERROR_BOOTUP_TIMEOUT,
    BOOT_ERROR_NODE_IDENTITY,
    BOOT_ERROR_SDO,
    BOOT_ERROR_HEARTBEAT_CONFIGURATION,
    BOOT_ERROR_MODE_CONFIGURATION,
    BOOT_ERROR_MODE_VERIFICATION,
    BOOT_ERROR_OPERATIONAL,
    BOOT_ERROR_HEARTBEAT_TIMEOUT,
    BOOT_ERROR_DRIVE_FAULT,
    BOOT_ERROR_DRIVE_QUICK_STOP,
    BOOT_ERROR_DRIVE_STATE,
    BOOT_ERROR_DRIVE_ENABLE,
    BOOT_ERROR_POSITION_FEEDBACK,
    BOOT_ERROR_CYCLIC_COMMUNICATION,
    BOOT_ERROR_SAFETY
} BootError;

typedef enum
{
    BOOT_ENABLE_READ_STATUS = 0,
    BOOT_ENABLE_WRITE_CONTROLWORD
} BootEnableStage;

typedef struct
{
    BootPhase phase;
    BootError error;
    int failedAxis;

    uint8_t axisIndex;
    uint8_t identitySubindex;
    bool transactionStarted;

    BootEnableStage enableStage;
    uint16_t pendingControlword;
    uint32_t driveEnableAttempts;

    int32_t holdPosition[BOOT_EXPECTED_NODE_COUNT];

    uint32_t stableCycles;
    uint32_t lastTpdoCount[BOOT_EXPECTED_NODE_COUNT];

    uint32_t phaseStartedMs;
    uint32_t cycleStartedMs;
} BootState;

void state_boot_enter(
    BootState *boot
);

/*
 * Execute one non-blocking CANopen BOOT step.
 *
 * Platform-specific CAN setup happens before BOOT. The caller supplies an
 * initialized CanopenMaster backed by STM32 FDCAN, SIL Kit, or a test backend.
 * BOOT then performs CANopen/AVATAR commissioning only.
 */
StateStepResult state_boot_step(
    BootState *boot,
    CanopenMaster *master,
    uint32_t now_ms
);

#endif /* STATE_BOOT_H */
