#include "cia402.h"

#include <stddef.h>

uint16_t cia402_get_state(
    uint16_t statusword
)
{
    /*
     * States whose Statusword definition treats bit 5 as "don't care".
     */

    uint16_t masked4F =
        statusword &
        CIA402_STATUSWORD_MASK_4F;


    if (
        masked4F ==
        CIA402_STATE_NOT_READY_TO_SWITCH_ON
    )
    {
        return
            CIA402_STATE_NOT_READY_TO_SWITCH_ON;
    }


    if (
        masked4F ==
        CIA402_STATE_SWITCH_ON_DISABLED
    )
    {
        return
            CIA402_STATE_SWITCH_ON_DISABLED;
    }


    if (
        masked4F ==
        CIA402_STATE_FAULT_REACTION_ACTIVE
    )
    {
        return
            CIA402_STATE_FAULT_REACTION_ACTIVE;
    }


    if (
        masked4F ==
        CIA402_STATE_FAULT
    )
    {
        return
            CIA402_STATE_FAULT;
    }


    /*
     * States where bit 5 participates in distinguishing the CiA-402 state.
     */

    uint16_t masked6F =
        statusword &
        CIA402_STATUSWORD_MASK_6F;


    if (
        masked6F ==
        CIA402_STATE_READY_TO_SWITCH_ON
    )
    {
        return
            CIA402_STATE_READY_TO_SWITCH_ON;
    }


    if (
        masked6F ==
        CIA402_STATE_SWITCHED_ON
    )
    {
        return
            CIA402_STATE_SWITCHED_ON;
    }


    if (
        masked6F ==
        CIA402_STATE_OPERATION_ENABLED
    )
    {
        return
            CIA402_STATE_OPERATION_ENABLED;
    }


    if (
        masked6F ==
        CIA402_STATE_QUICK_STOP_ACTIVE
    )
    {
        return
            CIA402_STATE_QUICK_STOP_ACTIVE;
    }


    return
        CIA402_STATE_UNKNOWN;
}

bool cia402_is_fault(
    uint16_t drive_state
)
{
    return
        drive_state ==
            CIA402_STATE_FAULT
        ||
        drive_state ==
            CIA402_STATE_FAULT_REACTION_ACTIVE;
}


bool cia402_is_quick_stop(
    uint16_t drive_state
)
{
    return
        drive_state ==
        CIA402_STATE_QUICK_STOP_ACTIVE;
}


bool cia402_is_operation_enabled(
    uint16_t drive_state
)
{
    return
        drive_state ==
        CIA402_STATE_OPERATION_ENABLED;
}

const char *cia402_state_name(
    uint16_t drive_state
)
{
    switch (drive_state)
    {
        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
            return "NOT_READY_TO_SWITCH_ON";


        case CIA402_STATE_SWITCH_ON_DISABLED:
            return "SWITCH_ON_DISABLED";


        case CIA402_STATE_READY_TO_SWITCH_ON:
            return "READY_TO_SWITCH_ON";


        case CIA402_STATE_SWITCHED_ON:
            return "SWITCHED_ON";


        case CIA402_STATE_OPERATION_ENABLED:
            return "OPERATION_ENABLED";


        case CIA402_STATE_QUICK_STOP_ACTIVE:
            return "QUICK_STOP_ACTIVE";


        case CIA402_STATE_FAULT_REACTION_ACTIVE:
            return "FAULT_REACTION_ACTIVE";


        case CIA402_STATE_FAULT:
            return "FAULT";


        default:
            return "UNKNOWN";
    }
}

/* ============================================================================
 *  CiA-402 ENABLE STATE MACHINE
 * ============================================================================
 *
 * Directly extracted from the tested main.c.
 * ============================================================================
 */
bool cia402_get_enable_controlword(
    uint16_t drive_state,
    uint16_t *controlword
)
{
    if (controlword == NULL)
    {
        return false;
    }


    switch (drive_state)
    {
        /* ================================================================
         * AUTOMATIC STARTUP TRANSITION
         * ================================================================
         *
         * Not Ready to Switch On
         *      ->
         * Switch On Disabled
         *
         * This transition is performed automatically by the drive.
         * We therefore send no enable request yet.
         */

        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
        {
            *controlword =
                CIA402_CONTROLWORD_DISABLE_VOLTAGE;

            return true;
        }


        /* ================================================================
         * NORMAL ENABLE SEQUENCE
         * ================================================================
         */

        case CIA402_STATE_SWITCH_ON_DISABLED:
        {
            *controlword =
                CIA402_CONTROLWORD_SHUTDOWN;

            return true;
        }


        case CIA402_STATE_READY_TO_SWITCH_ON:
        {
            *controlword =
                CIA402_CONTROLWORD_SWITCH_ON;

            return true;
        }


        case CIA402_STATE_SWITCHED_ON:
        {
            *controlword =
                CIA402_CONTROLWORD_ENABLE_OPERATION;

            return true;
        }


        case CIA402_STATE_OPERATION_ENABLED:
        {
            *controlword =
                CIA402_CONTROLWORD_ENABLE_OPERATION;

            return true;
        }


        /* ================================================================
         * STATES THAT MUST NOT BE AUTO-ENABLED
         * ================================================================
         */

        case CIA402_STATE_QUICK_STOP_ACTIVE:
        case CIA402_STATE_FAULT_REACTION_ACTIVE:
        case CIA402_STATE_FAULT:
        case CIA402_STATE_UNKNOWN:
        default:
        {
            /*
             * Give the output a deterministic safe value, but return false.
             *
             * Callers MUST NOT treat this as an enable command.
             */
            *controlword =
                CIA402_CONTROLWORD_DISABLE_VOLTAGE;

            return false;
        }
    }
}


/* ============================================================================
 *  CiA-402 DISABLE STATE MACHINE
 * ============================================================================
 *
 * Directly extracted from the tested main.c.
 * ============================================================================
 */

uint16_t cia402_get_disable_controlword(
    uint16_t drive_state
)
{
    if (
        drive_state ==
        CIA402_STATE_OPERATION_ENABLED
    )
    {
        /*
         * Operation Enabled -> Switched On
         */
        return
            CIA402_CONTROLWORD_SWITCH_ON;
    }


    if (
        drive_state ==
        CIA402_STATE_SWITCHED_ON
    )
    {
        /*
         * Switched On -> Ready to Switch On
         */
        return
            CIA402_CONTROLWORD_SHUTDOWN;
    }


    if (
        drive_state ==
        CIA402_STATE_READY_TO_SWITCH_ON
    )
    {
        /*
         * Ready to Switch On -> Switch On Disabled
         */
        return
            CIA402_CONTROLWORD_DISABLE_VOLTAGE;
    }


    /*
     * Existing default shutdown request.
     */
    return
        CIA402_CONTROLWORD_DISABLE_VOLTAGE;
}
