#ifndef STATE_MACHINE_TYPES_H
#define STATE_MACHINE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Result returned whenever one robot state executes one non-blocking step
 * of its internal logic.
 *
 * Existing values are preserved for compatibility with BOOT, HOMING,
 * IDLE and TEACHING.
 */
typedef enum
{
    STATE_STEP_RUNNING = 0,
    STATE_STEP_COMPLETE,
    STATE_STEP_FAILED

} StateStepResult;


/*
 * Global robot states.
 *
 * The Supervisor is the only module allowed to change the active state.
 */
typedef enum
{
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_IDLE,
    ROBOT_STATE_TEACHING,
    ROBOT_STATE_PATH_VALIDATION,
    ROBOT_STATE_APPROACH,
    ROBOT_STATE_PATH_EXECUTION,

    /*
     * Legacy identifiers retained temporarily for source compatibility.
     * New supervisory code must use ROBOT_STATE_PATH_EXECUTION. These values
     * exist only so the pre-migration regression suite still compiles; the
     * unified runtime does not perform arc stabilization or enter them.
     */
    ROBOT_STATE_ARC_STABILIZING,
    ROBOT_STATE_WELDING,
    ROBOT_STATE_PAUSED,
    ROBOT_STATE_RETRACTING,
    ROBOT_STATE_FAULT,
    ROBOT_STATE_EMERGENCY_STOP,

    ROBOT_STATE_COUNT

} RobotStateId;

/*
 * Distinguishes a dry Preview from real welding execution.
 *
 * Preview follows the generated trajectory without enabling the welding
 * process. Production includes the welding process sequence.
 */
typedef enum
{
    ROBOT_EXECUTION_NONE = 0,
    ROBOT_EXECUTION_PREVIEW,
    ROBOT_EXECUTION_PRODUCTION

} RobotExecutionMode;
/*
 * Classification used by the supervisory fault logic.
 *
 * The detailed fault code will later be supplied by the fault-handling
 * matrix.
 */
typedef enum
{
    ROBOT_FAULT_SEVERITY_NONE = 0,
    ROBOT_FAULT_SEVERITY_WARNING,
    ROBOT_FAULT_SEVERITY_RECOVERABLE,
    ROBOT_FAULT_SEVERITY_PROTECTIVE_STOP,
    ROBOT_FAULT_SEVERITY_EMERGENCY

} RobotFaultSeverity;


/*
 * Preliminary fault codes used by the Supervisor.
 *
 * Permanent codes can later be aligned with the project's
 * fault-handling matrix.
 */
typedef enum
{
    ROBOT_FAULT_CODE_NONE = 0x0000U,
    ROBOT_FAULT_CODE_PROTECTIVE_STOP = 0x1001U,
    ROBOT_FAULT_CODE_EMERGENCY_STOP = 0x2001U,
    ROBOT_FAULT_CODE_EXECUTION_INVALID = 0x3001U,
    ROBOT_FAULT_CODE_EXECUTION_IO = 0x3002U,
    ROBOT_FAULT_CODE_WIRE_FEED_RELAY = 0x3003U,
    ROBOT_FAULT_CODE_RETRACTION = 0x3004U

} RobotFaultCode;


/*
 * Events accepted by the global Supervisor.
 *
 * Segment selection, Record and speed-control events remain TeachingState
 * events and are not duplicated here.
 */
typedef enum
{
    SUPERVISOR_EVENT_NONE = 0,

    /* Operator/HMI events. */
    SUPERVISOR_EVENT_TEACH,
    SUPERVISOR_EVENT_VALIDATE_PREVIEW,
    SUPERVISOR_EVENT_START_REPLAY,
    SUPERVISOR_EVENT_PAUSE_RESUME,
    SUPERVISOR_EVENT_RESET,
    SUPERVISOR_EVENT_HOME,

    /* State-module results. Each item must appear exactly once. */
    SUPERVISOR_EVENT_STATE_COMPLETE,
    SUPERVISOR_EVENT_STATE_FAILED,
    SUPERVISOR_EVENT_VALIDATION_REJECTED,
    SUPERVISOR_EVENT_STATE_ABORTED_HOME,
    SUPERVISOR_EVENT_STATE_ABORTED_RESET,

    /* Safety and fault events. */
    SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
    SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED,
    SUPERVISOR_EVENT_ESTOP_ASSERTED,
    SUPERVISOR_EVENT_ESTOP_RELEASED,
    SUPERVISOR_EVENT_FAULT_DETECTED,
    SUPERVISOR_EVENT_FAULT_ACKNOWLEDGED

} SupervisorEventType;


/*
 * One message transferred to the Supervisor.
 *
 * faultCode is intentionally generic until the project fault-handling
 * matrix assigns permanent codes.
 */
typedef struct
{
    SupervisorEventType type;

    RobotStateId sourceState;

    RobotFaultSeverity faultSeverity;

    uint32_t faultCode;
    uint32_t timestampMs;

} SupervisorEvent;


/*
 * Latest safety status observed by software.
 *
 * This structure represents monitoring information. It does not replace
 * the independent hardware E-stop or safe drive-stop chain.
 */
typedef struct
{
    bool estopActive;
    bool protectiveStopActive;
    bool globalFaultActive;

    bool motionPermitted;
    bool drivesReady;
    bool ethercatHealthy;

    uint32_t timestampMs;

    bool statusValid;

} RobotSafetySnapshot;

#endif /* STATE_MACHINE_TYPES_H */
