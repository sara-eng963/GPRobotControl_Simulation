#ifndef STATE_IDLE_H
#define STATE_IDLE_H

#include "../state_machine_types.h"
#include "../../CANComm/CANopen/canopen_master.h"
#include "../../ControlCore/Config/robot_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IDLE_PHASE_INIT = 0,
    IDLE_PHASE_HOLDING,
    IDLE_PHASE_COMPLETE,
    IDLE_PHASE_FAILED
} IdlePhase;

typedef enum
{
    IDLE_COMMAND_NONE = 0,
    IDLE_COMMAND_TEACH,
    IDLE_COMMAND_REPLAY
} IdleCommand;

typedef enum
{
    IDLE_ERROR_NONE = 0,

    /* Numeric slot 1 used to be EtherCAT PDO-unavailable. */
    IDLE_ERROR_COMMUNICATION,

    IDLE_ERROR_POSITION_FEEDBACK,
    IDLE_ERROR_DRIVE_NOT_ENABLED,

    /* Numeric slot 4 used to be EtherCAT WKC failure. */
    IDLE_ERROR_CYCLIC_FEEDBACK,

    IDLE_ERROR_SAFETY,
    IDLE_ERROR_INVALID_COMMAND
} IdleError;

typedef struct
{
    IdlePhase phase;
    IdleError error;

    /* 0 = not axis-specific; 1..6 = failed axis. */
    int failedAxis;

    /*
     * Exact raw AVATAR position captured on IDLE entry.
     * Holding raw units avoids introducing a conversion round-trip while
     * the robot is supposed to remain stationary.
     */
    int32_t holdPositionUnits[ROBOT_DOF];

    IdleCommand exitCommand;

    /* Number of hold cycles confirmed by fresh TPDO4 feedback. */
    uint32_t cyclesHeld;

    uint32_t commandPeriodMs;
    uint32_t lastCommandMs;
    bool commandClockStarted;
    bool awaitingFeedback;
    uint32_t commandTpdoCount[ROBOT_DOF];

} IdleState;

void state_idle_enter(
    IdleState *idle
);

StateStepResult state_idle_step(
    IdleState *idle,
    IdleCommand command,
    CanopenMaster *master,
    uint32_t now_ms
);

#endif /* STATE_IDLE_H */
