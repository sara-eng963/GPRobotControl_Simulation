#include "cia402.h"


uint16_t cia402_get_state(
    uint16_t statusword
)
{
    return
        statusword &
        CIA402_STATUSWORD_STATE_MASK;
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
