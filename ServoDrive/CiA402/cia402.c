#include "cia402.h"


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

uint16_t cia402_get_enable_controlword(
    uint16_t drive_state
)
{
    if (
        drive_state ==
        CIA402_STATE_SWITCH_ON_DISABLED
    )
    {
        /*
         * Switch On Disabled
         *      ->
         * Shutdown command
         *      ->
         * Ready to Switch On
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
         * Ready to Switch On
         *      ->
         * Switch On
         *      ->
         * Switched On
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
         * Switched On
         *      ->
         * Enable Operation
         *      ->
         * Operation Enabled
         */
        return
            CIA402_CONTROLWORD_ENABLE_OPERATION;
    }


    if (
        drive_state ==
        CIA402_STATE_OPERATION_ENABLED
    )
    {
        /*
         * Keep Operation Enabled.
         */
        return
            CIA402_CONTROLWORD_ENABLE_OPERATION;
    }


    /*
     * Existing fallback from main.c.
     *
     * Real hardware will later need more complete FAULT / QUICK STOP handling,
     * but no behavior is added in this extraction step.
     */
    return
        CIA402_CONTROLWORD_SHUTDOWN;
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
