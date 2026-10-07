#include "state_boot.h"

#include "../../ServoDrive/AvatarM/avatar_m_registers.h"
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
    CanopenMaster *master,
    bool expect_read,
    uint8_t expected_size,
    uint32_t *value
)
{
    const CanopenMasterSdoState state =
        canopen_master_sdo_state(master);

    if (state == CANOPEN_MASTER_SDO_PENDING)
    {
        return BOOT_SDO_WAITING;
    }

    if (state == CANOPEN_MASTER_SDO_COMPLETE)
    {
        const CanopenSdoResponse *response =
            canopen_master_sdo_response(master);

        if (response == NULL)
        {
            return BOOT_SDO_FAILED;
        }

        if (
            expect_read &&
            (
                response->type != CANOPEN_SDO_RESPONSE_READ ||
                response->data_size != expected_size
            )
        )
        {
            return BOOT_SDO_FAILED;
        }

        if (
            !expect_read &&
            response->type != CANOPEN_SDO_RESPONSE_WRITE_OK
        )
        {
            return BOOT_SDO_FAILED;
        }

        if (expect_read && value != NULL)
        {
            *value = response->value;
        }

        canopen_master_sdo_clear(master);
        boot->transactionStarted = false;

        return BOOT_SDO_DONE;
    }

    if (
        state == CANOPEN_MASTER_SDO_ABORT ||
        state == CANOPEN_MASTER_SDO_TIMEOUT ||
        state == CANOPEN_MASTER_SDO_TRANSPORT_ERROR
    )
    {
        if (state == CANOPEN_MASTER_SDO_ABORT)
        {
            const CanopenSdoResponse *response =
                canopen_master_sdo_response(master);

            if (response != NULL)
            {
                printf(
                    "BOOT: SDO abort 0x%08lX\n",
                    (unsigned long)response->abort_code
                );
            }
        }

        return BOOT_SDO_FAILED;
    }

    return BOOT_SDO_FAILED;
}

static bool all_nodes_report_state(
    const CanopenMaster *master,
    AvatarMHeartbeatState expected
)
{
    if (
        master == NULL ||
        master->node_count != BOOT_EXPECTED_NODE_COUNT
    )
    {
        return false;
    }

    for (size_t i = 0U; i < BOOT_EXPECTED_NODE_COUNT; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(master, i);

        if (
            drive == NULL ||
            !drive->heartbeat_seen ||
            drive->heartbeat_state != expected
        )
        {
            return false;
        }
    }

    return true;
}

static bool expected_node_layout(
    const CanopenMaster *master
)
{
    if (
        master == NULL ||
        !master->initialized ||
        master->node_count != BOOT_EXPECTED_NODE_COUNT
    )
    {
        return false;
    }

    for (size_t i = 0U; i < BOOT_EXPECTED_NODE_COUNT; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(master, i);

        if (
            drive == NULL ||
            drive->node_id != (uint8_t)(i + 1U)
        )
        {
            return false;
        }
    }

    return true;
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
    CanopenMaster *master,
    uint32_t now_ms
)
{
    if (
        boot == NULL ||
        master == NULL ||
        !master->initialized
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

    if (!canopen_master_poll(master, now_ms))
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
            if (!expected_node_layout(master))
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_NODE_COUNT,
                    0
                );
            }

            canopen_master_clear_runtime(master);

            master->heartbeat_timeout_ms =
                BOOT_HEARTBEAT_TIMEOUT_MS;

            boot_advance(
                boot,
                BOOT_PHASE_RESET_COMMUNICATION,
                now_ms
            );

            return STATE_STEP_RUNNING;
        }

        case BOOT_PHASE_RESET_COMMUNICATION:
        {
            if (!canopen_master_send_nmt_all(
                    master,
                    CANOPEN_NMT_RESET_COMMUNICATION))
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
            if (all_nodes_report_state(
                    master,
                    AVATAR_M_HEARTBEAT_STATE_BOOTUP))
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
                if (!canopen_master_begin_read_identity(
                        master,
                        boot->axisIndex,
                        boot->identitySubindex,
                        now_ms))
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
                    master,
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
                value != (uint32_t)AVATAR_M_EXPECTED_VENDOR_ID
            )
            {
                printf(
                    "BOOT: Node %u vendor mismatch. "
                    "Expected=0x%08lX Actual=0x%08lX\n",
                    (unsigned)boot->axisIndex + 1U,
                    (unsigned long)AVATAR_M_EXPECTED_VENDOR_ID,
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
                value != (uint32_t)AVATAR_M_EXPECTED_PRODUCT_CODE
            )
            {
                printf(
                    "BOOT: Node %u product mismatch. "
                    "Expected=0x%08lX Actual=0x%08lX\n",
                    (unsigned)boot->axisIndex + 1U,
                    (unsigned long)AVATAR_M_EXPECTED_PRODUCT_CODE,
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
                    BOOT_PHASE_CONFIGURE_INTERPOLATION_MODE,
                    now_ms
                );

                return STATE_STEP_RUNNING;
            }

            if (!boot->transactionStarted)
            {
                if (!canopen_master_begin_set_heartbeat_period(
                        master,
                        boot->axisIndex,
                        BOOT_HEARTBEAT_PRODUCER_MS,
                        now_ms))
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
                    master,
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
                if (!canopen_master_begin_set_interpolation_mode(
                        master,
                        boot->axisIndex,
                        now_ms))
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
                    master,
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
                if (!canopen_master_begin_read_mode_display(
                        master,
                        boot->axisIndex,
                        now_ms))
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
                    master,
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
                (uint8_t)value != (uint8_t)AVATAR_M_MODE_INTERPOLATION
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
            if (!canopen_master_send_nmt_all(
                    master,
                    CANOPEN_NMT_START))
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
            if (all_nodes_report_state(
                    master,
                    AVATAR_M_HEARTBEAT_STATE_OPERATIONAL))
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
                    if (!canopen_master_begin_read_statusword(
                            master,
                            boot->axisIndex,
                            now_ms))
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
                        master,
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
                if (!canopen_master_begin_write_controlword(
                        master,
                        boot->axisIndex,
                        boot->pendingControlword,
                        now_ms))
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
                    master,
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
                if (!canopen_master_begin_read_actual_position(
                        master,
                        boot->axisIndex,
                        now_ms))
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
                    master,
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
                    canopen_master_tpdo_rx_count(master, i);
            }

            if (!canopen_master_send_target_cycle(
                    master,
                    boot->holdPosition,
                    BOOT_EXPECTED_NODE_COUNT))
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
                    canopen_master_tpdo_rx_count(master, i) <=
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
                !canopen_master_all_feedback_valid(master) ||
                !canopen_master_all_drives_operation_enabled(master)
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
                if (!canopen_master_ready_for_motion(
                        master,
                        now_ms))
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
                    canopen_master_tpdo_rx_count(master, i);
            }

            if (!canopen_master_send_target_cycle(
                    master,
                    boot->holdPosition,
                    BOOT_EXPECTED_NODE_COUNT))
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
