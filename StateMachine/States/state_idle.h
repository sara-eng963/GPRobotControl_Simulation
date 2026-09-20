#ifndef STATE_IDLE_H
#define STATE_IDLE_H


#include "../state_machine_types.h"

#include "../../ControlCore/Config/robot_config.h"

#include <stdint.h>


/* ============================================================================
 * IDLE PHASES
 * ============================================================================
 */

typedef enum
{
    IDLE_PHASE_INIT = 0,

    IDLE_PHASE_HOLDING,

    IDLE_PHASE_COMPLETE,

    IDLE_PHASE_FAILED

} IdlePhase;


/* ============================================================================
 * OPERATOR COMMANDS
 * ============================================================================
 *
 * These are intentionally simple for now.
 *
 * Later the HMI / main robot FSM can provide these commands.
 */

typedef enum
{
    IDLE_COMMAND_NONE = 0,

    IDLE_COMMAND_TEACH,

    IDLE_COMMAND_REPLAY

} IdleCommand;


/* ============================================================================
 * IDLE FAILURE REASONS
 * ============================================================================
 */

typedef enum
{
    IDLE_ERROR_NONE = 0,

    IDLE_ERROR_PDO_UNAVAILABLE,

    IDLE_ERROR_POSITION_FEEDBACK,

    IDLE_ERROR_DRIVE_NOT_ENABLED,

    IDLE_ERROR_WKC,

    IDLE_ERROR_SAFETY,

    IDLE_ERROR_INVALID_COMMAND

} IdleError;


/* ============================================================================
 * IDLE STATE DATA
 * ============================================================================
 */

typedef struct
{
    IdlePhase phase;

    IdleError error;

    /*
     * 0 = failure is not axis-specific
     * 1..6 = failed axis
     */
    int failedAxis;


    /*
     * Hold target captured when IDLE begins.
     *
     * Stored directly in A6-EC position units so the drive
     * keeps exactly the position it had when entering IDLE.
     */
    int32_t holdPositionUnits[ROBOT_DOF];


    /*
     * Command that caused IDLE to finish.
     *
     * The outer FSM will later use this to decide
     * which robot state comes next.
     */
    IdleCommand exitCommand;


    /*
     * Number of healthy cyclic holding cycles completed.
     */
    uint32_t cyclesHeld;

} IdleState;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

void state_idle_enter(
    IdleState *idle
);


StateStepResult state_idle_step(
    IdleState *idle,
    IdleCommand command
);


#endif /* STATE_IDLE_H */