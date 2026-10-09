#include "state_boot.h"

#include "../../ServoDrive/CiA402/cia402.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BOOT_HEARTBEAT_PRODUCER_MS          100U
#define BOOT_HEARTBEAT_TIMEOUT_MS           300U
#define BOOT_PHASE_TIMEOUT_MS               3000U
#define BOOT_CYCLIC_RESPONSE_TIMEOUT_MS     100U
#define BOOT_DRIVE_ENABLE_MAX_ATTEMPTS      20U
#define BOOT_STABILITY_REQUIRED_CYCLES      20U

typedef enum
{
    BOOT_SDO_WAITING = 0,
    BOOT_SDO_DONE,
    BOOT_SDO_FAILED
} BootSdoPollResult;

static void boot_advance(
    BootState *boot,
    BootPhase next_phase,
    uint32_t now_ms
)
{
    boot->phase = next_phase;
    boot->axisIndex = 0U;
    boot->identitySubindex = 1U;
    boot->transactionStarted = false;
    boot->stableCycles = 0U;
    boot->phaseStartedMs = now_ms;
}

static StateStepResult boot_fail(
    BootState *boot,
    BootError error,
    int failed_axis
)
{
    boot->error = error;
    boot->failedAxis = failed_axis;
    boot->phase = BOOT_PHASE_FAILED;
    return STATE_STEP_FAILED;
}

static bool phase_timed_out(
    const BootState *boot,
    uint32_t now_ms
)
{
    return
        (uint32_t)(now_ms - boot->phaseStartedMs) >=
        BOOT_PHASE_TIMEOUT_MS;
}

static BootSdoPollResult collect_sdo(
    BootState *boot,
    const DriveCommissioningPort *commissioning,
    bool expect_read,
    uint8_t expected_size,
    uint32_t *value
)
{
    const DriveOperationResult result =
        drive_commissioning_port_result(commissioning);

    if (result.status == DRIVE_OPERATION_PENDING)
        return BOOT_SDO_WAITING;

    if (result.status == DRIVE_OPERATION_COMPLETE)
    {
        if (!result.has_response ||
            (expect_read &&
             (!result.is_read || result.data_size != expected_size)) ||
            (!expect_read && !result.is_write_ok))
            return BOOT_SDO_FAILED;

        if (expect_read && value != NULL)
            *value = result.value;

        drive_commissioning_port_clear(commissioning);
        boot->transactionStarted = false;
        return BOOT_SDO_DONE;
    }

    if (result.status == DRIVE_OPERATION_ABORT)
        printf("BOOT: SDO abort 0x%08lX\n",
               (unsigned long)result.abort_code);

    return BOOT_SDO_FAILED;
}

void state_boot_enter(
    BootState *boot
)
{
    if (boot == NULL)
    {
        return;
    }

    memset(boot, 0, sizeof(*boot));

    boot->phase = BOOT_PHASE_INIT;
    boot->error = BOOT_ERROR_NONE;
    boot->enableStage = BOOT_ENABLE_READ_STATUS;
    boot->identitySubindex = 1U;
}

StateStepResult state_boot_step(
    BootState *boot,
    const DriveCommissioningPort *commissioning,
    const JointDrivePort *drive_port,
    uint32_t now_ms
)
{
    if (
        boot == NULL ||
        !drive_commissioning_port_valid(commissioning) ||
        !joint_drive_port_valid(drive_port)
    )
    {
        if (boot != NULL)
        {
            return boot_fail(
                boot,
                BOOT_ERROR_MASTER_INIT,
                0
            );
        }

        return STATE_STEP_FAILED;
    }

    if (!joint_drive_port_poll(drive_port, now_ms))
    {
        return boot_fail(
            boot,
            BOOT_ERROR_SDO,
            boot->axisIndex < BOOT_EXPECTED_NODE_COUNT
                ? (int)boot->axisIndex + 1
                : 0
        );
    }

    switch (boot->phase)
    {
        case BOOT_PHASE_INIT:
        {
            if (!commissioning->is_configured(commissioning->context))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_NODE_COUNT,
                    0
                );
            }

            drive_commissioning_port_reset_runtime(
                commissioning, BOOT_HEARTBEAT_TIMEOUT_MS);

            boot_advance(
                boot,
                BOOT_PHASE_RESET_COMMUNICATION,
                now_ms
            );

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_RESET_COMMUNICATION:
        {
            if (!drive_commissioning_port_network_command(
                    commissioning, DRIVE_NETWORK_RESET_COMMUNICATION))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_RESET_COMMUNICATION,
                    0
                );
            }

            boot_advance(
                boot,
                BOOT_PHASE_WAIT_BOOTUP,
                now_ms
            );

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_WAIT_BOOTUP:
        {
            if (drive_commissioning_port_all_heartbeat_state(
                    commissioning, DRIVE_NODE_BOOTUP))
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_VERIFY_IDENTITIES,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (phase_timed_out(boot, now_ms))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_BOOTUP_TIMEOUT,
                    0
                );
            }

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_VERIFY_IDENTITIES:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_CONFIGURE_HEARTBEAT,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_READ_IDENTITY, boot->axisIndex, boot->identitySubindex, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_NODE_IDENTITY,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            uint32_t value = 0U;

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    true,
                    4U,
                    &value
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (sdo == BOOT_SDO_FAILED)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_NODE_IDENTITY,
                    (int)boot->axisIndex + 1
                );
            }

            if (
                boot->identitySubindex == 1U &&
                value != (uint32_t)commissioning->expected_vendor_id
            )
            {
                printf(
                    "BOOT: Node %u vendor mismatch. "
                    "Expected=0x%08lX Actual=0x%08lX\n",
                    (unsigned)boot->axisIndex + 1U,
                    (unsigned long)commissioning->expected_vendor_id,
                    (unsigned long)value
                );

                return boot_fail(
                    boot,
                    BOOT_ERROR_NODE_IDENTITY,
                    (int)boot->axisIndex + 1
                );
            }

            if (
                boot->identitySubindex == 2U &&
                value != (uint32_t)commissioning->expected_product_code
            )
            {
                printf(
                    "BOOT: Node %u product mismatch. "
                    "Expected=0x%08lX Actual=0x%08lX\n",
                    (unsigned)boot->axisIndex + 1U,
                    (unsigned long)commissioning->expected_product_code,
                    (unsigned long)value
                );

                return boot_fail(
                    boot,
                    BOOT_ERROR_NODE_IDENTITY,
                    (int)boot->axisIndex + 1
                );
            }

            if (boot->identitySubindex == 1U)
            {
                boot->identitySubindex = 2U;
            }
            else
            {
                boot->identitySubindex = 1U;
                boot->axisIndex++;
            }

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_CONFIGURE_HEARTBEAT:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_CONFIGURE_HEARTBEAT_CONSUMER,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_SET_HEARTBEAT_PERIOD, boot->axisIndex, BOOT_HEARTBEAT_PRODUCER_MS, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_HEARTBEAT_CONFIGURATION,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    false,
                    0U,
                    NULL
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (sdo == BOOT_SDO_FAILED)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_HEARTBEAT_CONFIGURATION,
                    (int)boot->axisIndex + 1
                );
            }

            boot->axisIndex++;
            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_CONFIGURE_HEARTBEAT_CONSUMER:
        case BOOT_PHASE_VERIFY_HEARTBEAT_CONSUMER:
        {
            const bool verify =
                boot->phase == BOOT_PHASE_VERIFY_HEARTBEAT_CONSUMER;
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(boot,
                    verify ? BOOT_PHASE_CONFIGURE_INTERPOLATION_MODE
                           : BOOT_PHASE_VERIFY_HEARTBEAT_CONSUMER,
                    now_ms);
                return STATE_STEP_RUNNING;
            }
            if (!boot->transactionStarted)
            {
                const bool sent = verify
                    ? drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_READ_HEARTBEAT_CONSUMER,
                        boot->axisIndex, 0U, now_ms)
                    : drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_SET_HEARTBEAT_CONSUMER,
                        boot->axisIndex, 0U, now_ms);
                if (!sent)
                    return boot_fail(boot, BOOT_ERROR_HEARTBEAT_CONFIGURATION,
                                     (int)boot->axisIndex + 1);
                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }
            uint32_t readback = 0U;
            const BootSdoPollResult result =
                collect_sdo(
                    boot,
                    commissioning, verify, verify ? 4U : 0U,
                            verify ? &readback : NULL);
            if (result == BOOT_SDO_WAITING) return STATE_STEP_RUNNING;
            if (result != BOOT_SDO_DONE ||
                (verify && readback != commissioning->expected_heartbeat_consumer))
                return boot_fail(boot, BOOT_ERROR_HEARTBEAT_CONFIGURATION,
                                 (int)boot->axisIndex + 1);
            boot->axisIndex++;
            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_CONFIGURE_INTERPOLATION_MODE:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_VERIFY_INTERPOLATION_MODE,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_SET_INTERPOLATION_MODE, boot->axisIndex, 0U, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_MODE_CONFIGURATION,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    false,
                    0U,
                    NULL
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (sdo == BOOT_SDO_FAILED)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_MODE_CONFIGURATION,
                    (int)boot->axisIndex + 1
                );
            }

            boot->axisIndex++;
            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_VERIFY_INTERPOLATION_MODE:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_REQUEST_OPERATIONAL,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_READ_MODE_DISPLAY, boot->axisIndex, 0U, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_MODE_VERIFICATION,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            uint32_t value = 0U;

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    true,
                    1U,
                    &value
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (
                sdo == BOOT_SDO_FAILED ||
                (uint8_t)value != (uint8_t)commissioning->expected_mode_display
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_MODE_VERIFICATION,
                    (int)boot->axisIndex + 1
                );
            }

            boot->axisIndex++;
            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_REQUEST_OPERATIONAL:
        {
            if (!drive_commissioning_port_network_command(
                    commissioning, DRIVE_NETWORK_START))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_OPERATIONAL,
                    0
                );
            }

            boot_advance(
                boot,
                BOOT_PHASE_WAIT_OPERATIONAL_HEARTBEAT,
                now_ms
            );

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_WAIT_OPERATIONAL_HEARTBEAT:
        {
            if (drive_commissioning_port_all_heartbeat_state(
                    commissioning, DRIVE_NODE_OPERATIONAL))
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_ENABLE_DRIVES,
                    now_ms
                );

                boot->enableStage =
                    BOOT_ENABLE_READ_STATUS;

                boot->driveEnableAttempts =
                    0U;

                return STATE_STEP_RUNNING;
            }

            if (phase_timed_out(boot, now_ms))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_HEARTBEAT_TIMEOUT,
                    0
                );
            }

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_ENABLE_DRIVES:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_READ_POSITION_FEEDBACK,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (boot->enableStage == BOOT_ENABLE_READ_STATUS)
            {
                if (!boot->transactionStarted)
                {
                    if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_READ_STATUSWORD, boot->axisIndex, 0U, now_ms))
                    {
                        return boot_fail(
                            boot,
                            BOOT_ERROR_DRIVE_ENABLE,
                            (int)boot->axisIndex + 1
                        );
                    }

                    boot->transactionStarted = true;
                    return STATE_STEP_RUNNING;
                }

                uint32_t raw_statusword = 0U;

                const BootSdoPollResult sdo =
                    collect_sdo(
                    boot,
                    commissioning,
                        true,
                        2U,
                        &raw_statusword
                    );

                if (sdo == BOOT_SDO_WAITING)
                {
                    return STATE_STEP_RUNNING;
                }

                if (sdo == BOOT_SDO_FAILED)
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_DRIVE_ENABLE,
                        (int)boot->axisIndex + 1
                    );
                }

                const uint16_t drive_state =
                    cia402_get_state(
                        (uint16_t)raw_statusword
                    );

                if (cia402_is_fault(drive_state))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_DRIVE_FAULT,
                        (int)boot->axisIndex + 1
                    );
                }

                if (cia402_is_quick_stop(drive_state))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_DRIVE_QUICK_STOP,
                        (int)boot->axisIndex + 1
                    );
                }

                if (drive_state == CIA402_STATE_OPERATION_ENABLED)
                {
                    boot->axisIndex++;
                    boot->driveEnableAttempts = 0U;
                    return STATE_STEP_RUNNING;
                }

                if (!cia402_get_enable_controlword(
                        drive_state,
                        &boot->pendingControlword))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_DRIVE_STATE,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->enableStage =
                    BOOT_ENABLE_WRITE_CONTROLWORD;

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_WRITE_CONTROLWORD, boot->axisIndex, boot->pendingControlword, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_DRIVE_ENABLE,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    false,
                    0U,
                    NULL
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (sdo == BOOT_SDO_FAILED)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_DRIVE_ENABLE,
                    (int)boot->axisIndex + 1
                );
            }

            boot->driveEnableAttempts++;

            if (
                boot->driveEnableAttempts >
                BOOT_DRIVE_ENABLE_MAX_ATTEMPTS
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_DRIVE_ENABLE,
                    (int)boot->axisIndex + 1
                );
            }

            boot->enableStage =
                BOOT_ENABLE_READ_STATUS;

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_READ_POSITION_FEEDBACK:
        {
            if (boot->axisIndex >= BOOT_EXPECTED_NODE_COUNT)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_INITIAL_PDO_EXCHANGE,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!drive_commissioning_port_begin(commissioning,
                        DRIVE_COMMISSION_READ_ACTUAL_POSITION, boot->axisIndex, 0U, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_POSITION_FEEDBACK,
                        (int)boot->axisIndex + 1
                    );
                }

                boot->transactionStarted = true;
                return STATE_STEP_RUNNING;
            }

            uint32_t raw_position = 0U;

            const BootSdoPollResult sdo =
                collect_sdo(
                    boot,
                    commissioning,
                    true,
                    4U,
                    &raw_position
                );

            if (sdo == BOOT_SDO_WAITING)
            {
                return STATE_STEP_RUNNING;
            }

            if (sdo == BOOT_SDO_FAILED)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_POSITION_FEEDBACK,
                    (int)boot->axisIndex + 1
                );
            }

            memcpy(
                &boot->holdPosition[boot->axisIndex],
                &raw_position,
                sizeof(raw_position)
            );

            boot->axisIndex++;
            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_INITIAL_PDO_EXCHANGE:
        {
            for (size_t i = 0U; i < BOOT_EXPECTED_NODE_COUNT; ++i)
            {
                boot->lastTpdoCount[i] =
                    joint_drive_port_feedback_sequence(drive_port, i);
            }

            if (!joint_drive_port_send_targets(
                    drive_port, boot->holdPosition))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_CYCLIC_COMMUNICATION,
                    0
                );
            }

            boot_advance(
                boot,
                BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION,
                now_ms
            );

            boot->cycleStartedMs = now_ms;

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION:
        {
            bool all_fresh = true;

            for (size_t i = 0U; i < BOOT_EXPECTED_NODE_COUNT; ++i)
            {
                if (
                    joint_drive_port_feedback_sequence(drive_port, i) <=
                    boot->lastTpdoCount[i]
                )
                {
                    all_fresh = false;
                    break;
                }
            }

            if (!all_fresh)
            {
                if (
                    (uint32_t)(now_ms - boot->cycleStartedMs) >=
                    BOOT_CYCLIC_RESPONSE_TIMEOUT_MS
                )
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_CYCLIC_COMMUNICATION,
                        0
                    );
                }

                return STATE_STEP_RUNNING;
            }

            if (
                !joint_drive_port_all_feedback_valid(drive_port) ||
                !joint_drive_port_all_enabled(drive_port)
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_DRIVE_STATE,
                    0
                );
            }

            boot->stableCycles++;

            if (
                boot->stableCycles >=
                BOOT_STABILITY_REQUIRED_CYCLES
            )
            {
                if (!joint_drive_port_ready(drive_port, now_ms))
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_CYCLIC_COMMUNICATION,
                        0
                    );
                }

                boot_advance(
                    boot,
                    BOOT_PHASE_VERIFY_SAFETY,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            for (size_t i = 0U; i < BOOT_EXPECTED_NODE_COUNT; ++i)
            {
                boot->lastTpdoCount[i] =
                    joint_drive_port_feedback_sequence(drive_port, i);
            }

            if (!joint_drive_port_send_targets(
                    drive_port, boot->holdPosition))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_CYCLIC_COMMUNICATION,
                    0
                );
            }

            boot->cycleStartedMs = now_ms;

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_VERIFY_SAFETY:
        {
            /*
             * Physical E-stop/safety-relay feedback remains a Supervisor
             * responsibility, exactly as in the previous BOOT implementation.
             */
            boot_advance(
                boot,
                BOOT_PHASE_COMPLETE,
                now_ms
            );

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_COMPLETE:
            return STATE_STEP_COMPLETE;

        case BOOT_PHASE_FAILED:
        default:
            return STATE_STEP_FAILED;
    }
}
