/* Real FreeRTOS workload driver. No mocked queues, scheduler, longjmp, private
 * Supervisor state injection, or replacement numerical algorithms. */
#include "application.h"
#include <string.h>

static StaticTask_t scenario_tcb, peer_tcb;
static StackType_t scenario_stack[4096], peer_stack[512];
static SupervisorDiagnostics diagnostics;

static void inspect(void)
{
    MB_ASSERT(supervisor_task_get_diagnostics(&diagnostics));
    const uint32_t state=diagnostics.machine.activeState;
    if (membench_application[MB_LAST_STATE]!=state) ++membench_application[MB_TRANSITIONS];
    membench_application[MB_LAST_STATE]=state;
    membench_application[MB_LAST_BOOT_PHASE]=diagnostics.boot.phase;
    membench_application[MB_LAST_FAULT]=diagnostics.machine.activeFaultCode;
    membench_application[MB_LAST_VALIDATION_RESULT]=diagnostics.validation.report.result;
    membench_application[MB_LAST_VALIDATION_ERROR]=diagnostics.validation.report.error;
    membench_application[MB_LAST_EXECUTION_ERROR]=diagnostics.execution.error;
    membench_application[MB_LAST_EXECUTION_PHASE]=diagnostics.execution.phase;
    membench_application[MB_GUIDANCE_OK]=diagnostics.guidance_dry_run_ok_cycles;
    if (diagnostics.segment_count>membench_application[MB_MAX_PROGRAM_SEGMENTS])
        membench_application[MB_MAX_PROGRAM_SEGMENTS]=diagnostics.segment_count;
    SupervisionProgressSample sample;
    if (application_ipc_service_get_progress(&sample)) membench_application[MB_PEER_PROGRESS]=sample.sequence;
}
static void phase(unsigned id)
{
    membench_application[MB_PHASE]=id;
    membench_application[MB_PHASE_TICK]=xTaskGetTickCount();
}
static void pass(unsigned id) { membench_application[MB_COMPLETED] |= 1U << id; }
static void wait_state(RobotStateId desired, uint32_t limit)
{
    membench_application[MB_WAITING_STATE]=desired;
    const TickType_t start=xTaskGetTickCount();
    for (;;) {
        inspect();
        if (diagnostics.machine.activeState==desired) return;
        MB_ASSERT(xTaskGetTickCount()-start<limit);
        MB_ASSERT(diagnostics.machine.activeState!=ROBOT_STATE_FAULT &&
                  diagnostics.machine.activeState!=ROBOT_STATE_EMERGENCY_STOP);
        vTaskDelay(2);
    }
}
static void hmi(SupervisorHmiCommand command)
{
    const TickType_t start=xTaskGetTickCount();
    ApplicationControllerStatus status;
    MB_ASSERT(application_ipc_service_read_status(&status,100));
    const uint32_t before=status.last_hmi_received_sequence;
    for (;;) {
        /* Queue acceptance is not M7 acceptance. Retry an unacknowledged
         * operator request with varying phase against the 10ms status cycle.
         * The real peer's challenge/session checks are never bypassed. */
        if (application_ipc_service_submit_hmi(command)) vTaskDelay(13);
        else vTaskDelay(2);
        if (application_ipc_service_read_status(&status,100) &&
            status.last_hmi_received_sequence!=before) break;
        MB_ASSERT(xTaskGetTickCount()-start<500);
    }
    inspect();
}
static void wait_points(unsigned count)
{
    const TickType_t start=xTaskGetTickCount();
    do { vTaskDelay(2); inspect(); MB_ASSERT(xTaskGetTickCount()-start<100); }
    while (diagnostics.captured_points<count);
}
static void teach_segment(unsigned *endpoint)
{
    hmi(SUP_HMI_LINE);
    wait_state(ROBOT_STATE_TEACHING,100);
    if (!diagnostics.captured_points) { hmi(SUP_HMI_RECORD); wait_points(1); }
    *endpoint ^= 1;
    mb_set_manual_pose(*endpoint);
    vTaskDelay(20);
    hmi(SUP_HMI_RECORD);
    wait_points(2);
}
static void peer(void *unused)
{
    (void)unused;
    application_ipc_service_run();
}
static void playback(bool production, bool pause)
{
    MB_ASSERT(mb_cache_first_sample());
    hmi(production ? SUP_HMI_START_REPLAY : SUP_HMI_VALIDATE_PREVIEW);
    wait_state(ROBOT_STATE_PATH_EXECUTION,5000);
    if (pause) {
        vTaskDelay(30);
        hmi(SUP_HMI_PAUSE);
        wait_state(ROBOT_STATE_PAUSED,100);
        vTaskDelay(20);
        hmi(SUP_HMI_RESUME);
        wait_state(ROBOT_STATE_PATH_EXECUTION,100);
    }
    const TickType_t start=xTaskGetTickCount();
    while (diagnostics.machine.activeState!=ROBOT_STATE_IDLE) {
        vTaskDelay(2); inspect();
        MB_ASSERT(xTaskGetTickCount()-start<10000);
        MB_ASSERT(diagnostics.machine.activeState!=ROBOT_STATE_FAULT);
    }
    MB_ASSERT(diagnostics.machine.validatedTrajectoryAvailable);
    MB_ASSERT(diagnostics.machine.previewAccepted);
    ++membench_application[MB_PLAYBACKS];
    membench_application[MB_EXPECTED_PLAYBACK_SAMPLES]+=mb_artifact.sample_count;
    MB_ASSERT(membench_application[MB_PLAYBACK_SAMPLES]+membench_application[MB_SKIPPED_PLAYBACK_SAMPLES]
                ==membench_application[MB_EXPECTED_PLAYBACK_SAMPLES]);
}
void mb_scenario(void *unused)
{
    (void)unused;
    phase(MB_STARTUP);
    mb_providers_initialize();
    MB_ASSERT(application_validation_service_start());
    MB_ASSERT(application_ipc_service_initialize());
    MB_ASSERT(application_ft_service_start());
    MB_ASSERT(xTaskCreateStatic(peer,"M4IPC",512,NULL,5,peer_stack,&peer_tcb));
    MB_ASSERT(supervisor_task_init(&mb_config));
    MB_ASSERT(supervisor_task_start(9));
    const TickType_t start=xTaskGetTickCount();
    do {
        vTaskDelay(2); inspect();
        MB_ASSERT(xTaskGetTickCount()-start<3000);
        MB_ASSERT(diagnostics.machine.activeState!=ROBOT_STATE_FAULT);
    } while (diagnostics.boot.phase!=BOOT_PHASE_WAIT_AUTHORIZATION);
    vTaskDelay(10); /* observe released approval after valid startup evidence */
    mb_operator_pressed=true;
    wait_state(ROBOT_STATE_IDLE,1000);
    MB_ASSERT(membench_application[MB_CAN_SYNCS]>20);
    pass(MB_STARTUP);
    phase(MB_IDLE);
    const uint32_t held=membench_application[MB_CAN_SYNCS];
    vTaskDelay(40); inspect();
    MB_ASSERT(diagnostics.machine.activeState==ROBOT_STATE_IDLE);
    MB_ASSERT(membench_application[MB_CAN_SYNCS]>held);
    pass(MB_IDLE);
    membench_application[MB_HEAP_AT_START]=xPortGetFreeHeapSize();
    membench_application[MB_TASK_COUNT]=uxTaskGetNumberOfTasks();
    unsigned endpoint=0;
    for (unsigned round=0;round<2;round++) {
        phase(MB_TEACHING);
        teach_segment(&endpoint);
        MB_ASSERT(diagnostics.segment_count==1);
        vTaskDelay(60); inspect();
        MB_ASSERT(membench_application[MB_FT_FRAMES]>0);
        MB_ASSERT(diagnostics.guidance_dry_run_ok_cycles>0);
        pass(MB_TEACHING);
        phase(MB_VALIDATION);
        hmi(SUP_HMI_VALIDATE_PREVIEW);
        wait_state(ROBOT_STATE_IDLE,15000);
        MB_ASSERT(diagnostics.machine.validatedTrajectoryAvailable);
        MB_ASSERT(diagnostics.validation.report.result==PV_RESULT_VALID);
        MB_ASSERT(mb_artifact.sample_count>=TRAJECTORY_CAPACITY);
        ++membench_application[MB_VALIDATIONS];
        membench_application[MB_VALIDATED_SAMPLES]+=mb_artifact.sample_count;
        pass(MB_VALIDATION);
        phase(MB_PLAYBACK);
        playback(false,round==0);
        if (round==0) playback(true,false);
        pass(MB_PLAYBACK);
        hmi(SUP_HMI_RESET);
        MB_ASSERT(!diagnostics.machine.validatedTrajectoryAvailable);
        membench_application[round==0 ? MB_HEAP_AFTER_FIRST : MB_HEAP_AFTER_SECOND]=xPortGetFreeHeapSize();
        MB_ASSERT(xPortGetFreeHeapSize()==membench_application[MB_HEAP_AT_START]);
    }
    phase(MB_TRANSITIONS_PHASE);
    MB_ASSERT(membench_application[MB_TRANSITIONS]>=15);
    pass(MB_TRANSITIONS_PHASE);
    phase(MB_CAPACITY);
    for (unsigned i=0;i<TEACHING_MAX_SEGMENTS;i++) {
        teach_segment(&endpoint);
        MB_ASSERT(diagnostics.segment_count==i+1);
    }
    membench_application[MB_MAX_PROGRAM_SEGMENTS]=diagnostics.segment_count;
    hmi(SUP_HMI_LINE);
    MB_ASSERT(diagnostics.segment_count==TEACHING_MAX_SEGMENTS);
    endpoint ^= 1;
    mb_set_manual_pose(endpoint);
    vTaskDelay(20);
    hmi(SUP_HMI_RECORD);
    MB_ASSERT(diagnostics.segment_count==TEACHING_MAX_SEGMENTS);
    MB_ASSERT(diagnostics.teaching.error==TEACH_ERR_SEGMENT_CAPACITY);
    mb_ft_capacity();
    MB_ASSERT(membench_application[MB_MAX_RAW_GEOMETRY]==512);
    MB_ASSERT(membench_application[MB_MAX_TRAJECTORY_RING]==TRAJECTORY_CAPACITY);
    MB_ASSERT(membench_application[MB_MAX_VALIDATION_QUEUE]==VALIDATION_QUEUE_CAPACITY);
    /* Final capacity stress intentionally exercises the production overload
     * fault, after the positive operation cycles completed. */
    taskENTER_CRITICAL();
    SupervisorMessage message={.type=SUPERVISOR_MESSAGE_GUIDED_TARGET};
    unsigned sent=0;
    while (supervisor_task_post(&message,0)) ++sent;
    membench_application[MB_MAX_SUPERVISOR_QUEUE]=sent;
    MB_ASSERT(sent==32);
    taskEXIT_CRITICAL();
    pass(MB_CAPACITY);
    phase(MB_FAULTS);
    vTaskDelay(10); inspect();
    MB_ASSERT(diagnostics.machine.activeState==ROBOT_STATE_FAULT);
    MB_ASSERT(membench_application[MB_SAFE_OUTPUT_CALLS]>0);
    const uint32_t stopped=membench_application[MB_CAN_TARGETS];
    MB_ASSERT(supervisor_task_post_hmi(SUP_HMI_RESUME,0));
    vTaskDelay(10); inspect();
    MB_ASSERT(diagnostics.machine.activeState==ROBOT_STATE_FAULT);
    MB_ASSERT(membench_application[MB_CAN_TARGETS]==stopped);
    membench_application[MB_RESTART_DENIED]=1;
    vTaskDelay(10);
    FtSensorSample sample;
    MB_ASSERT(mb_ft_stopped);
    MB_ASSERT(!application_ft_service_read(NULL,&sample));
    membench_application[MB_FT_FAULT_VERIFIED]=1;
    mb_estop=true;
    vTaskDelay(10); inspect();
    MB_ASSERT(diagnostics.machine.activeState==ROBOT_STATE_EMERGENCY_STOP);
    MB_ASSERT(membench_application[MB_CAN_TARGETS]==stopped);
    pass(MB_FAULTS);
    membench_application[MB_STATUS]=1;
    for (;;) vTaskDelay(100);
}
void membench_application_start(void)
{
    /* Test device bytes are outside MCU SRAM. Add a single explicit MPU region
     * for this virtual NOR backing store; preserve all production MPU regions. */
    MPU_Region_InitTypeDef region={.Enable=MPU_REGION_ENABLE,.Number=MPU_REGION_NUMBER2,
        .BaseAddress=0x90000000,.Size=MPU_REGION_SIZE_512KB,.AccessPermission=MPU_REGION_FULL_ACCESS,
        .DisableExec=MPU_INSTRUCTION_ACCESS_DISABLE,.TypeExtField=MPU_TEX_LEVEL1};
    HAL_MPU_Disable();
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    MB_ASSERT(xTaskCreateStatic(mb_scenario,"BenchScenario",4096,NULL,3,scenario_stack,&scenario_tcb));
}
