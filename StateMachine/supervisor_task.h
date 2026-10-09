#ifndef SUPERVISOR_TASK_H
#define SUPERVISOR_TASK_H

#include "state_machine.h"
/* Composition root still owns the concrete CANopen coordinator. */
#include "../CANComm/CANopen/canopen_master.h"
#include "supervisor_io.h"
#include "supervisor_hmi.h"
#include "States/state_approach.h"
#include "States/state_boot.h"
#include "States/state_homing.h"
#include "States/state_idle.h"
#include "States/state_path_validation.h"
#include "States/state_path_execution.h"
#include "States/state_paused.h"
#include "States/state_fault.h"
#include "States/state_emergency_stop.h"
#include "States/state_teaching.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SUPERVISOR_MESSAGE_EVENT = 0,
    SUPERVISOR_MESSAGE_SAFETY,
    SUPERVISOR_MESSAGE_TEACHING_EVENT,
    SUPERVISOR_MESSAGE_TEACHING_RUNTIME,
    SUPERVISOR_MESSAGE_APPROACH_CONTROL,
    SUPERVISOR_MESSAGE_PATH_EXECUTION_INPUTS,
    SUPERVISOR_MESSAGE_HMI_COMMAND,
    /* Nonblocking six-axis target request, accepted only during TEACHING. */
    SUPERVISOR_MESSAGE_GUIDED_TARGET
} SupervisorMessageType;

typedef struct
{
    SupervisorMessageType type;
    union
    {
        SupervisorEvent event;
        RobotSafetySnapshot safety;
        TeachingEvent teaching_event;
        TeachingRuntimeInputs teaching_runtime;
        ApproachControlInputs approach_control;
        PathExecutionInputs path_execution_inputs;
        SupervisorHmiCommand hmi_command;
        int32_t guided_target_units[JOINT_DRIVE_AXES];
    } data;
} SupervisorMessage;

/* Published only by Supervisor. Readers copy under the same-core critical
 * section; Phase 2 replaces that transport with an inter-core mailbox. */
typedef struct {
    bool valid;
    bool drives_ready;
    bool heartbeats_operational;
    bool feedback_valid[JOINT_DRIVE_AXES];
    int32_t actual_position_units[JOINT_DRIVE_AXES];
    uint32_t timestamp_ms;
} SupervisorDriveSnapshot;

typedef struct
{
    /*
     * Communication object used by BOOT.
     * The platform creates and initializes this before SupervisorTask starts.
     */
    CanopenMaster *canopen_master;
    const AvatarMPositionScale *avatar_position_scales;

    const RobotConfig *robot;
    const HomingConfig *homing_config;
    const TeachingConfig *teaching_config;

    const PathValidationConfig *validation_config;
    const PathValidationServices *validation_services;
    PathValidationWorkspace *validation_workspace;
    const PathValidationStorage *validation_storage;
    ValidatedTrajectory *validated_trajectory;

    const ApproachConfig *approach_config;
    const ApproachServices *approach_services;

    const PathExecutionConfig *path_execution_config;
    const PathExecutionServices *path_execution_services;
    const PausedServices *paused_services;
    const FaultServices *fault_services;
    const EmergencyStopServices *emergency_stop_services;

    /* Optional nonblocking entry gate. If false, Supervisor retries next
     * tick rather than sleeping while the prefetch worker is reading. */
    bool (*validation_enter_ready)(void *context);
    void *validation_enter_context;

    /* If enabled, ALL validation (including blocking storage begin/write/
     * commit) runs in a lower-priority task, not the 1 kHz Supervisor.
     * Never report VALID until that worker completed durable commit.
     * The worker priority must be strictly below Supervisor priority. */
    bool validation_run_in_worker;
    UBaseType_t validation_worker_priority;
    /* Caller-provided persistent snapshot; avoids forcing another ~KB-sized
     * TaughtProgram into MCU SRAM when async mode is disabled. */
    TaughtProgram *validation_worker_program;

    uint16_t validation_sample_budget;
    uint32_t first_program_id;
    TickType_t period_ticks;
    /* Optional board adapter; NULL callbacks allow incremental integration. */
    SupervisorIoServices io;
} SupervisorTaskConfig;

/* Read-only diagnostics; state contexts remain owned by SupervisorTask. */
typedef struct {
    StateMachine machine;
    BootState boot;
    HomingPhase homing_phase;
    HomingError homing_error;
    TeachingOutputs teaching;
    PathValidationOutputs validation;
    ApproachOutputs approach;
    PathExecutionOutputs execution;
    TeachingSegmentType selected_geometry;
    TaughtSegment working_segment;
    uint8_t captured_points;
    uint16_t segment_count;
} SupervisorDiagnostics;
bool supervisor_task_get_diagnostics(SupervisorDiagnostics *out);

bool supervisor_task_init(const SupervisorTaskConfig *config);
bool supervisor_task_start(UBaseType_t priority);

bool supervisor_task_post(const SupervisorMessage *message, TickType_t wait_ticks);
bool supervisor_task_post_from_isr(
    const SupervisorMessage *message,
    BaseType_t *higher_priority_task_woken
);

bool supervisor_task_get_state(StateMachine *state);
bool supervisor_task_get_inputs(SupervisorInputSnapshot *inputs);
bool supervisor_task_get_drive_snapshot(SupervisorDriveSnapshot *out);
bool supervisor_task_post_hmi(SupervisorHmiCommand command, TickType_t wait_ticks);
QueueHandle_t supervisor_task_queue(void);
TaskHandle_t supervisor_task_handle(void);

#endif /* SUPERVISOR_TASK_H */
