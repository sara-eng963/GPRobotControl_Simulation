#ifndef STATE_PATH_EXECUTION_H
#define STATE_PATH_EXECUTION_H

#include "../state_machine_types.h"
#include "state_path_validation.h"
#include "../../CANComm/CANopen/canopen_master.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PATH_EXEC_PHASE_IDLE = 0,
    PATH_EXEC_PHASE_CHECK_PREREQUISITES,
    PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED,
    PATH_EXEC_PHASE_FOLLOW_TRAJECTORY,
    PATH_EXEC_PHASE_DISABLE_WIRE_FEED,
    PATH_EXEC_PHASE_PREPARE_RETRACTION,
    PATH_EXEC_PHASE_EXECUTE_RETRACTION,
    PATH_EXEC_PHASE_VERIFY_CLEARANCE,
    PATH_EXEC_PHASE_CONTROLLED_STOP,
    PATH_EXEC_PHASE_COMPLETE,
    PATH_EXEC_PHASE_ABORTED,
    PATH_EXEC_PHASE_FAILED
} PathExecutionPhase;

typedef enum
{
    PATH_EXEC_ERR_NONE = 0,
    PATH_EXEC_ERR_NULL_ARGUMENT,
    PATH_EXEC_ERR_INVALID_CONFIG,
    PATH_EXEC_ERR_INVALID_REQUEST,
    PATH_EXEC_ERR_TRAJECTORY_NOT_READY,
    PATH_EXEC_ERR_TRAJECTORY_MISMATCH,
    PATH_EXEC_ERR_STORAGE_READ,

    /* Numeric slot formerly used by the generic command-write callback. */
    PATH_EXEC_ERR_COMMUNICATION,

    PATH_EXEC_ERR_WIRE_FEED_RELAY,
    PATH_EXEC_ERR_MOTION_PERMISSION,
    PATH_EXEC_ERR_FOLLOWING_ERROR,
    PATH_EXEC_ERR_RETRACTION,
    PATH_EXEC_ERR_CLEARANCE,
    PATH_EXEC_ERR_STOP_TIMEOUT,
    PATH_EXEC_ERR_EXTERNAL_FAULT,
    PATH_EXEC_ERR_ESTOP,
    PATH_EXEC_ERR_RESET_REQUESTED,
    PATH_EXEC_ERR_HOME_REQUESTED,

    PATH_EXEC_ERR_CYCLIC_FEEDBACK
} PathExecutionError;

typedef enum
{
    PATH_EXEC_RESULT_NONE = 0,
    PATH_EXEC_RESULT_RUNNING,
    PATH_EXEC_RESULT_COMPLETE,
    PATH_EXEC_RESULT_ABORTED,
    PATH_EXEC_RESULT_FAILED
} PathExecutionResult;

typedef enum
{
    PATH_EXEC_ABORT_NONE = 0,
    PATH_EXEC_ABORT_RESET,
    PATH_EXEC_ABORT_HOME
} PathExecutionAbortReason;

typedef struct
{
    RobotExecutionMode mode;
    const ValidatedTrajectory *trajectory;
    bool trajectory_ready;
    uint32_t expected_program_id;
    uint32_t expected_source_revision;
    uint32_t expected_artifact_crc;
} PathExecutionRequest;

typedef struct
{
    bool motion_permission;
    bool following_error;
    bool pause_requested;
    bool reset_requested;
    bool home_requested;
    bool estop_active;
    bool protective_stop_active;
    bool external_fault_active;
    uint32_t now_ms;
} PathExecutionInputs;

typedef struct
{
    uint32_t controlled_stop_timeout_ms;
    uint32_t clearance_stable_cycles;
} PathExecutionConfig;

typedef bool (*PathExecReadSampleFn)(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
);

typedef bool (*PathExecSetWireFeedFn)(bool enable, void *context);
typedef bool (*PathExecPrepareRetractionFn)(void *context);
typedef StateStepResult (*PathExecRetractionStepFn)(void *context);
typedef bool (*PathExecClearanceFn)(void *context);
typedef bool (*PathExecControlledStopFn)(bool *stopped, void *context);

typedef struct
{
    PathExecReadSampleFn read_sample;

    /*
     * Non-drive services remain callbacks. Six-axis motion targets are sent
     * directly through CanopenMaster so PATH EXECUTION owns the same RPDO4 /
     * SYNC / fresh-TPDO4 contract as APPROACH.
     */
    PathExecSetWireFeedFn set_wire_feed_enabled;
    PathExecPrepareRetractionFn prepare_retraction;
    PathExecRetractionStepFn step_retraction;
    PathExecClearanceFn clearance_verified;
    PathExecControlledStopFn controlled_stop;
    void *context;
} PathExecutionServices;

typedef struct
{
    PathExecutionPhase phase;
    PathExecutionResult result;
    PathExecutionError error;
    PathExecutionAbortReason abort_reason;
    uint32_t sample_index;
    uint32_t sample_count;
    uint32_t phase_started_ms;
    uint32_t clearance_stable_cycles;
    uint32_t command_period_ms;
    bool wire_feed_commanded;
} PathExecutionOutputs;

typedef struct
{
    PathExecutionPhase phase;
    PathExecutionResult result;
    PathExecutionError error;
    PathExecutionAbortReason abort_reason;

    CanopenMaster *master;

    PathExecutionRequest request;
    PathExecutionConfig config;
    PathExecutionServices services;

    uint32_t sample_index;
    uint32_t phase_started_ms;
    uint32_t clearance_stable_cycles;

    uint32_t command_period_ms;
    uint32_t last_command_ms;
    bool command_clock_started;
    bool awaiting_feedback;
    uint32_t command_tpdo_count[CANOPEN_MASTER_MAX_NODES];

    bool wire_feed_commanded;
    bool initialized;
} PathExecutionState;

void state_path_execution_enter(
    PathExecutionState *state,
    CanopenMaster *master,
    const PathExecutionRequest *request,
    const PathExecutionConfig *config,
    const PathExecutionServices *services
);

StateStepResult state_path_execution_step(
    PathExecutionState *state,
    const PathExecutionInputs *inputs,
    PathExecutionOutputs *outputs
);

void state_path_execution_get_outputs(
    const PathExecutionState *state,
    PathExecutionOutputs *outputs
);

/* Re-enable production wire feed after PAUSED forced the relay off. */
void state_path_execution_resume(
    PathExecutionState *state,
    uint32_t now_ms
);

const char *state_path_execution_phase_name(PathExecutionPhase phase);
const char *state_path_execution_error_name(PathExecutionError error);

#endif
