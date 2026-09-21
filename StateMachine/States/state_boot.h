#ifndef STATE_BOOT_H
#define STATE_BOOT_H


#include "../state_machine_types.h"

#include "../../EtherCATComm/ethercat_types.h"

#include <stdint.h>


/* ============================================================================
 * BOOT PHASES
 * ============================================================================
 */

typedef enum
{
    BOOT_PHASE_INIT = 0,

    BOOT_PHASE_OPEN_BUS,

    BOOT_PHASE_DISCOVER_SLAVES,

    BOOT_PHASE_VERIFY_SLAVE_IDENTITIES,

    BOOT_PHASE_CONFIGURE_DRIVES,

    BOOT_PHASE_MAP_PDOS,

    BOOT_PHASE_CONFIGURE_DC,

    BOOT_PHASE_SAFE_OP,

    BOOT_PHASE_INITIAL_PDO_EXCHANGE,

    BOOT_PHASE_OPERATIONAL,

    BOOT_PHASE_VERIFY_WKC,

    BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION,

    BOOT_PHASE_ENABLE_DRIVES,

    BOOT_PHASE_VERIFY_POSITION_FEEDBACK,

    BOOT_PHASE_VERIFY_SAFETY,

    BOOT_PHASE_COMPLETE,

    BOOT_PHASE_FAILED

} BootPhase;


/* ============================================================================
 * BOOT FAILURE REASONS
 * ============================================================================
 */

typedef enum
{
    BOOT_ERROR_NONE = 0,

    BOOT_ERROR_MASTER_INIT,

    BOOT_ERROR_OPEN_BUS,

    BOOT_ERROR_SLAVE_COUNT,

    BOOT_ERROR_SLAVE_IDENTITY,

    BOOT_ERROR_DRIVE_CONFIGURATION,

    BOOT_ERROR_PDO_MAPPING,

    BOOT_ERROR_DC_CONFIGURATION,

    BOOT_ERROR_SAFE_OP,

    BOOT_ERROR_OPERATIONAL,

    BOOT_ERROR_WKC,

    BOOT_ERROR_CYCLIC_COMMUNICATION,

    BOOT_ERROR_DRIVE_FAULT,

    BOOT_ERROR_DRIVE_QUICK_STOP,

    BOOT_ERROR_DRIVE_STATE,

    BOOT_ERROR_DRIVE_ENABLE,

    BOOT_ERROR_POSITION_FEEDBACK,

    BOOT_ERROR_SAFETY

} BootError;


/* ============================================================================
 * BOOT STATE DATA
 * ============================================================================
 */

typedef struct
{
    BootPhase phase;

    BootError error;

    /*
     * 0 = failure is not axis-specific.
     * 1..6 = axis that caused the failure.
     */
    int failedAxis;

    /*
     * Used when verifying several consecutive
     * healthy PDO cycles.
     */
    uint32_t stableCycles;

    /*
 * Number of EtherCAT recovery attempts made
 * during the current BOOT phase.
 */
uint32_t recoveryAttempts;

} BootState;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/*
 * Called whenever the robot enters BOOT.
 */
void state_boot_enter(
    BootState *boot
);


/*
 * Execute one step of the BOOT state.
 *
 * This function will eventually call:
 *
 *      EtherCATComm
 *      A6EC
 *      CiA402
 *      Safety interface
 *
 * but does not implement their low-level behavior itself.
 */
StateStepResult state_boot_step(
    BootState *boot,
    const EtherCATMasterConfig *ethercatConfig
);


#endif /* STATE_BOOT_H */