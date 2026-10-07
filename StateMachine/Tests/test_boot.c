#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "state_boot.h"
#include "sim_can_bus.h"

#include "cia402.h"

#define TEST_NUM_AXES 6U

static const char *boot_phase_name(
    BootPhase phase
)
{
    switch (phase)
    {
        case BOOT_PHASE_INIT:
            return "INIT";
        case BOOT_PHASE_RESET_COMMUNICATION:
            return "RESET_COMMUNICATION";
        case BOOT_PHASE_WAIT_BOOTUP:
            return "WAIT_BOOTUP";
        case BOOT_PHASE_VERIFY_IDENTITIES:
            return "VERIFY_IDENTITIES";
        case BOOT_PHASE_CONFIGURE_HEARTBEAT:
            return "CONFIGURE_HEARTBEAT";
        case BOOT_PHASE_CONFIGURE_INTERPOLATION_MODE:
            return "CONFIGURE_INTERPOLATION_MODE";
        case BOOT_PHASE_VERIFY_INTERPOLATION_MODE:
            return "VERIFY_INTERPOLATION_MODE";
        case BOOT_PHASE_REQUEST_OPERATIONAL:
            return "REQUEST_OPERATIONAL";
        case BOOT_PHASE_WAIT_OPERATIONAL_HEARTBEAT:
            return "WAIT_OPERATIONAL_HEARTBEAT";
        case BOOT_PHASE_ENABLE_DRIVES:
            return "ENABLE_DRIVES";
        case BOOT_PHASE_READ_POSITION_FEEDBACK:
            return "READ_POSITION_FEEDBACK";
        case BOOT_PHASE_INITIAL_PDO_EXCHANGE:
            return "INITIAL_PDO_EXCHANGE";
        case BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION:
            return "VERIFY_CYCLIC_COMMUNICATION";
        case BOOT_PHASE_VERIFY_SAFETY:
            return "VERIFY_SAFETY";
        case BOOT_PHASE_COMPLETE:
            return "COMPLETE";
        case BOOT_PHASE_FAILED:
            return "FAILED";
        default:
            return "UNKNOWN";
    }
}

static const char *boot_error_name(
    BootError error
)
{
    switch (error)
    {
        case BOOT_ERROR_NONE:
            return "NONE";
        case BOOT_ERROR_MASTER_INIT:
            return "MASTER_INIT";
        case BOOT_ERROR_NODE_COUNT:
            return "NODE_COUNT";
        case BOOT_ERROR_RESET_COMMUNICATION:
            return "RESET_COMMUNICATION";
        case BOOT_ERROR_BOOTUP_TIMEOUT:
            return "BOOTUP_TIMEOUT";
        case BOOT_ERROR_NODE_IDENTITY:
            return "NODE_IDENTITY";
        case BOOT_ERROR_SDO:
            return "SDO";
        case BOOT_ERROR_HEARTBEAT_CONFIGURATION:
            return "HEARTBEAT_CONFIGURATION";
        case BOOT_ERROR_MODE_CONFIGURATION:
            return "MODE_CONFIGURATION";
        case BOOT_ERROR_MODE_VERIFICATION:
            return "MODE_VERIFICATION";
        case BOOT_ERROR_OPERATIONAL:
            return "OPERATIONAL";
        case BOOT_ERROR_HEARTBEAT_TIMEOUT:
            return "HEARTBEAT_TIMEOUT";
        case BOOT_ERROR_DRIVE_FAULT:
            return "DRIVE_FAULT";
        case BOOT_ERROR_DRIVE_QUICK_STOP:
            return "DRIVE_QUICK_STOP";
        case BOOT_ERROR_DRIVE_STATE:
            return "DRIVE_STATE";
        case BOOT_ERROR_DRIVE_ENABLE:
            return "DRIVE_ENABLE";
        case BOOT_ERROR_POSITION_FEEDBACK:
            return "POSITION_FEEDBACK";
        case BOOT_ERROR_CYCLIC_COMMUNICATION:
            return "CYCLIC_COMMUNICATION";
        case BOOT_ERROR_SAFETY:
            return "SAFETY";
        default:
            return "UNKNOWN";
    }
}

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " CANOPEN / AVATAR BOOT STATE TEST\n"
        "============================================================\n"
    );

    AvatarMSimBus bus = {0};
    CanBackend backend = {0};

    if (!avatar_sim_bus_init(
            &bus,
            &backend))
    {
        printf("Could not initialize AVATAR simulation bus.\n");
        return 1;
    }

    /*
     * Explicit startup fixtures:
     * - start outside interpolation mode so BOOT must configure 0x6060
     * - start Switch On Disabled so BOOT must perform the CiA-402 sequence
     * - give every axis a distinct actual position so hold-position reads are
     *   observable and cross-node mixups are easy to catch.
     */
    for (uint8_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        avatar_m_node_set_work_mode(
            &bus.nodes[i],
            1U
        );

        avatar_m_node_set_statusword(
            &bus.nodes[i],
            CIA402_STATE_SWITCH_ON_DISABLED
        );

        avatar_m_node_set_actual_position(
            &bus.nodes[i],
            (int32_t)((i + 1U) * 1000U)
        );
    }

    CanopenMaster master = {0};

    const CanopenMasterConfig master_config =
    {
        .backend = &backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = TEST_NUM_AXES,
        .heartbeat_timeout_ms = 300U,
        .sdo_timeout_ms = 50U
    };

    if (!canopen_master_init(
            &master,
            &master_config))
    {
        printf("Could not initialize CANopen master.\n");
        return 1;
    }

    BootState boot;

    state_boot_enter(
        &boot
    );

    BootPhase previous_phase =
        boot.phase;

    printf(
        "BOOT -> %s\n",
        boot_phase_name(boot.phase)
    );

    StateStepResult result =
        STATE_STEP_RUNNING;

    for (uint32_t now_ms = 1U; now_ms <= 5000U; ++now_ms)
    {
        avatar_sim_bus_tick(
            &bus,
            1U
        );

        result =
            state_boot_step(
                &boot,
                &master,
                now_ms
            );

        if (boot.phase != previous_phase)
        {
            printf(
                "BOOT -> %s\n",
                boot_phase_name(boot.phase)
            );

            previous_phase =
                boot.phase;
        }

        if (
            result == STATE_STEP_COMPLETE ||
            result == STATE_STEP_FAILED
        )
        {
            break;
        }
    }

    if (result != STATE_STEP_COMPLETE)
    {
        printf(
            "\n"
            "============================================================\n"
            " BOOT TEST FAILED\n"
            "============================================================\n"
            "Phase      : %s\n"
            "Error      : %s\n"
            "Failed axis: %d\n"
            "============================================================\n",
            boot_phase_name(boot.phase),
            boot_error_name(boot.error),
            boot.failedAxis
        );

        canopen_master_close(
            &master
        );

        return 1;
    }

    bool final_state_ok =
        true;

    for (uint8_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(
                &master,
                i
            );

        if (
            bus.nodes[i].nmt_state !=
                AVATAR_M_NMT_OPERATIONAL ||
            bus.nodes[i].work_mode !=
                7U ||
            cia402_get_state(
                bus.nodes[i].statusword
            ) !=
                CIA402_STATE_OPERATION_ENABLED ||
            drive == NULL ||
            !drive->feedback_valid ||
            drive->feedback.actual_position !=
                (int32_t)((i + 1U) * 1000U)
        )
        {
            final_state_ok =
                false;
        }
    }

    if (
        !final_state_ok ||
        !canopen_master_ready_for_motion(
            &master,
            bus.now_ms)
    )
    {
        printf(
            "BOOT reached COMPLETE but final CANopen/drive state is invalid.\n"
        );

        canopen_master_close(
            &master
        );

        return 1;
    }

    printf(
        "\n"
        "============================================================\n"
        " CANOPEN BOOT TEST PASSED\n"
        "============================================================\n"
        "6 nodes identified\n"
        "Interpolation mode 7 verified\n"
        "Operational heartbeat verified\n"
        "All CiA-402 drives Operation Enabled\n"
        "Actual position read through SDO\n"
        "20 fresh RPDO4/SYNC/TPDO4 cycles verified\n"
        "============================================================\n"
    );

    canopen_master_close(
        &master
    );

    return 0;
}
