/* PC-only integration test: real FreeRTOS queue and scheduler, mock board I/O.
 * This does not test physical stopping, a physical CAN bus, or hard real-time deadlines.
 * Zero-valued motion configurations are placeholders and are never executed:
 * an E-stop is queued before the supervisor gets its first scheduler turn.
 */
#include "supervisor_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned passed, failed;
static unsigned reads, writes, safe_calls;
static bool acquisition_ok = true;
static bool context_ok = true;
static SupervisorInputSnapshot board_inputs;
static SupervisorOutputSnapshot board_outputs;
static GpIpcShared test_ipc;
static void check(bool ok, const char *name)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) ++passed; else ++failed;
}
static bool read_board(void *context, SupervisorInputSnapshot *out)
{
    context_ok = context_ok && context == &board_inputs;
    ++reads;
    *out = board_inputs;
    return acquisition_ok;
}
static bool write_board(void *context, const SupervisorOutputSnapshot *out)
{
    (void)context;
    ++writes;
    board_outputs = *out;
    return true;
}
static bool safe_outputs(void *context)
{
    (void)context;
    ++safe_calls;
    return true;
}
static void finish(void)
{
    printf("\nTests passed: %u\nTests failed: %u\n", passed, failed);
    fflush(stdout);
    exit(failed ? EXIT_FAILURE : EXIT_SUCCESS);
}
/* Only call this helper from the test task, with the scheduler running. */
static void wait_for_cycles(void)
{
    vTaskDelay(pdMS_TO_TICKS(10));
}
static void test_task(void *argument)
{
    (void)argument;
    StateMachine state;
    SupervisorInputSnapshot inputs;
    SupervisorDriveSnapshot drive_snapshot;
    SupervisorMessage message = {0}, received;
    check(supervisor_task_get_state(&state), "Initialized state snapshot is readable");
    check(state.activeState == ROBOT_STATE_BOOT, "Task begins in BOOT before supervisor starts");
    check(!supervisor_task_get_state(NULL), "Null state destination is rejected");
    check(!supervisor_task_get_inputs(NULL), "Null input destination is rejected");
    check(!supervisor_task_get_drive_snapshot(NULL), "Null drive snapshot is rejected");
    check(supervisor_task_get_drive_snapshot(&drive_snapshot) &&
          !drive_snapshot.valid, "Uninitialized CANopen publishes invalid snapshot");
    check(!supervisor_task_post(NULL, 0), "Null message is rejected");
    check(!supervisor_task_post_hmi((SupervisorHmiCommand)-1, 0), "Negative HMI command is rejected");
    check(!supervisor_task_post_hmi(SUP_HMI_COMMAND_COUNT, 0), "Out-of-range HMI command is rejected");

    /* Transport checks: receive directly here before starting the consumer.
     * They prove message contents/order, not command acceptance by a state. */
    for (unsigned i = 0; i < SUP_HMI_COMMAND_COUNT; ++i)
    {
        check(supervisor_task_post_hmi((SupervisorHmiCommand)i, 0), "Valid HMI command is queued");
        memset(&received, 0, sizeof(received));
        check(xQueueReceive(supervisor_task_queue(), &received, 0) == pdPASS,
              "Queued HMI command can be received");
        check(received.type == SUPERVISOR_MESSAGE_HMI_COMMAND &&
              received.data.hmi_command == (SupervisorHmiCommand)i,
              "HMI envelope preserves the command ID");
    }
    unsigned queued = 0;
    while (queued < 256 && supervisor_task_post(&message, 0)) ++queued;
    check(queued > 0 && queued < 256, "Queue has a finite nonzero capacity");
    check(!supervisor_task_post(&message, 0), "Full queue reports send failure");
    unsigned drained = 0;
    while (xQueueReceive(supervisor_task_queue(), &received, 0) == pdPASS) ++drained;
    check(drained == queued, "Every successfully queued message remains available");

    message.type = SUPERVISOR_MESSAGE_EVENT;
    message.data.event.type = SUPERVISOR_EVENT_ESTOP_ASSERTED;
    check(supervisor_task_post(&message, 0), "E-stop event is queued before first state step");
    check(supervisor_task_start(2), "Supervisor task creation succeeds");
    check(supervisor_task_handle() != NULL, "Created supervisor has a task handle");
    check(!supervisor_task_start(2), "Duplicate task start is rejected");
    wait_for_cycles();
    check(supervisor_task_get_state(&state), "Running supervisor publishes its state");
    check(state.activeState == ROBOT_STATE_EMERGENCY_STOP, "Queued E-stop enters emergency state");
    check(reads > 0 && writes > 0, "Supervisor invokes both board callbacks");
    check(context_ok, "GPIO callback receives configured context");
    check(safe_calls > 0, "Emergency step dispatch invokes safe-output service");
    check(supervisor_task_get_inputs(&inputs), "Input snapshot is readable");
    check(inputs.valid && inputs.timestamp_ms == 1234 && inputs.asserted[SUP_IO_HOME_J3],
          "Input snapshot preserves validity, timestamp and channel value");
    check(board_outputs.asserted[SUP_IO_STATUS_FAULT] &&
          !board_outputs.asserted[SUP_IO_STATUS_READY] &&
          !board_outputs.asserted[SUP_IO_STATUS_RUNNING],
          "Emergency state publishes fault status without ready/running");

    uint32_t rejections = state.rejectedEventCount;
    check(supervisor_task_post_hmi(SUP_HMI_START_REPLAY, 0), "START is delivered through HMI queue");
    wait_for_cycles();
    supervisor_task_get_state(&state);
    check(state.activeState == ROBOT_STATE_EMERGENCY_STOP &&
          state.rejectedEventCount == rejections + 1,
          "Supervisor routes HMI START and rejects it during E-stop");

    /* Simulate M4 HMI: only Supervisor may mutate state. */
    GpIpcMessage ipc_hmi = {0};
    ipc_hmi.type = GP_IPC_HMI_COMMAND;
    ipc_hmi.sequence = 1U;
    ipc_hmi.data[0] = SUP_HMI_START_REPLAY;
    rejections = state.rejectedEventCount;
    check(gp_ipc_m4_send(&test_ipc, &ipc_hmi), "M4 sends IPC HMI request");
    wait_for_cycles();
    check(supervisor_task_get_state(&state) &&
          state.activeState == ROBOT_STATE_EMERGENCY_STOP &&
          state.rejectedEventCount == rejections + 1,
          "M7 Supervisor handles and rejects M4 START during E-stop");

    GpIpcMessage ipc_status = {0};
    bool saw_ipc_status = false;
    while (gp_ipc_m4_receive(&test_ipc, &ipc_status)) {
        if (ipc_status.type == GP_IPC_CONTROLLER_STATUS &&
            ipc_status.data[0] == ROBOT_STATE_EMERGENCY_STOP &&
            ipc_status.data[5] == 1U) saw_ipc_status = true;
    }
    check(saw_ipc_status, "M4 receives M7 status with last received command");

    rejections = state.rejectedEventCount;
    check(gp_ipc_m4_send(&test_ipc, &ipc_hmi), "M4 sends duplicate sequence");
    wait_for_cycles();
    check(supervisor_task_get_state(&state) &&
          state.rejectedEventCount == rejections,
          "M7 ignores replayed M4 command");

    acquisition_ok = false;
    wait_for_cycles();
    supervisor_task_get_inputs(&inputs);
    check(!inputs.valid, "Failed GPIO acquisition invalidates the published snapshot");
    acquisition_ok = true;
    board_inputs.timestamp_ms = 5678;
    wait_for_cycles();
    supervisor_task_get_inputs(&inputs);
    check(inputs.valid && inputs.timestamp_ms == 5678, "GPIO acquisition can recover on a later cycle");
    finish();
}
int main(void)
{
    /* CTest supplies the process timeout without changing port signals. */
    puts("SUPERVISOR INTERFACE SCHEDULER TEST");
    check(!supervisor_task_init(NULL), "Null configuration is rejected");
    check(!supervisor_task_start(2), "Starting before initialization is rejected");
    check(!supervisor_task_post_hmi(SUP_HMI_LINE, 0), "Posting before queue creation fails");

    static CanopenMaster canopen_master;
    static AvatarMPositionScale position_scales[ROBOT_DOF];
    static RobotConfig robot;
    static HomingConfig homing;
    static TeachingConfig teaching;
    static PathValidationConfig validation;
    static PathValidationWorkspace workspace;
    static PathValidationStorage storage;
    static ValidatedTrajectory trajectory;
    static ApproachConfig approach;
    static ApproachServices approach_services;
    static PathExecutionConfig execution;
    static PathExecutionServices execution_services;
    static PausedServices paused;
    static FaultServices fault;
    static EmergencyStopServices emergency;
    SupervisorTaskConfig config = {0};
    check(!supervisor_task_init(&config), "Missing required configuration is rejected");
    config.canopen_master = &canopen_master;
    config.avatar_position_scales = position_scales;
    config.robot = &robot;
    config.homing_config = &homing;
    config.teaching_config = &teaching;
    config.validation_config = &validation;
    config.validation_workspace = &workspace;
    config.validation_storage = &storage;
    config.validated_trajectory = &trajectory;
    config.approach_config = &approach;
    config.approach_services = &approach_services;
    config.path_execution_config = &execution;
    config.path_execution_services = &execution_services;
    config.paused_services = &paused;
    config.fault_services = &fault;
    emergency.force_safe_outputs = safe_outputs;
    config.emergency_stop_services = &emergency;
    config.validation_sample_budget = 1;
    check(!supervisor_task_init(&config), "Zero task period is rejected");
    config.period_ticks = pdMS_TO_TICKS(1);
    gp_ipc_initialize(&test_ipc);
    config.ipc = &test_ipc;
    config.io.context = &board_inputs;
    config.io.read_inputs = read_board;
    config.io.write_outputs = write_board;
    board_inputs.valid = true;
    board_inputs.timestamp_ms = 1234;
    board_inputs.asserted[SUP_IO_HOME_J3] = true;
    check(supervisor_task_init(&config), "Complete pointer configuration creates supervisor queue");
    check(supervisor_task_queue() != NULL, "Supervisor queue handle is available");
    if (failed) finish();
    check(xTaskCreate(test_task, "InterfaceTest", 2048, NULL, 3, NULL) == pdPASS,
          "Interface test task is created");
    if (failed) finish();
    vTaskStartScheduler();
    check(false, "Scheduler must run the test task");
    finish();
    return EXIT_FAILURE;
}
