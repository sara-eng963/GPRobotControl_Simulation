#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>

/*
 * ============================================================================
 *  CiA-402 DRIVE STATE MACHINE
 * ============================================================================
 *
 * This module is a direct extraction of the CiA-402 logic that already existed
 * in the tested main.c.
 *
 * It does NOT know:
 *      - EtherCAT / SOEM
 *      - A6-EC PDO offsets
 *      - robot trajectories
 *      - HMI logic
 *
 * Current tested state identification:
 *
 *      statusword & 0x006F
 *
 * Important states used by the controller:
 *
 *      0x0040  Switch On Disabled
 *      0x0021  Ready to Switch On
 *      0x0023  Switched On
 *      0x0027  Operation Enabled
 * ============================================================================
 */

#define CIA402_STATUSWORD_STATE_MASK          0x006FU

#define CIA402_STATE_SWITCH_ON_DISABLED       0x0040U
#define CIA402_STATE_READY_TO_SWITCH_ON       0x0021U
#define CIA402_STATE_SWITCHED_ON              0x0023U
#define CIA402_STATE_OPERATION_ENABLED        0x0027U


/*
 * Controlwords used by the already tested state-machine sequence.
 */
#define CIA402_CONTROLWORD_DISABLE_VOLTAGE    0x0000U
#define CIA402_CONTROLWORD_SHUTDOWN           0x0006U
#define CIA402_CONTROLWORD_SWITCH_ON          0x0007U
#define CIA402_CONTROLWORD_ENABLE_OPERATION   0x000FU


/*
 * Extract the same state bits that main.c previously calculated with:
 *
 *      statusword & 0x006F
 */
uint16_t cia402_get_state(
    uint16_t statusword
);


/*
 * Same enable sequence previously implemented by get_enable_controlword():
 *
 *      Switch On Disabled  -> 0x0006
 *      Ready To Switch On  -> 0x0007
 *      Switched On         -> 0x000F
 *      Operation Enabled   -> 0x000F
 *
 * Existing fallback is intentionally preserved:
 *
 *      unhandled state -> 0x0006
 */
uint16_t cia402_get_enable_controlword(
    uint16_t drive_state
);


/*
 * Same clean disable sequence previously implemented by
 * get_disable_controlword():
 *
 *      Operation Enabled   -> 0x0007
 *      Switched On         -> 0x0006
 *      Ready To Switch On  -> 0x0000
 *
 * Existing fallback is intentionally preserved:
 *
 *      unhandled state -> 0x0000
 */
uint16_t cia402_get_disable_controlword(
    uint16_t drive_state
);


#endif /* CIA402_H */
