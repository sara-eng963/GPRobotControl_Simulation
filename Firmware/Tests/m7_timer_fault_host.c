/* Host HAL/FreeRTOS stubs; selected functions are inserted unmodified from
 * the real repository at runtime by run_m7_timer_fault_injection.py. */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <limits.h>

#define TIM2 ((void *)(uintptr_t)2)
#define TIM3 ((void *)(uintptr_t)3)
#define TIM_FLAG_UPDATE 1U
#define RESET 0
#define HAL_OK 0
#define RCC_APB1_DIV2 2U
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) (x)
typedef unsigned TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct {
    void *Instance;
    struct { uint32_t Prescaler, Period; } Init;
    uint32_t Counter;
    int UIF;
} TIM_HandleTypeDef;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
static TIM_HandleTypeDef htim2, htim3;
static volatile uint32_t g_tim2_wraps;
static volatile bool g_timing_running;
static unsigned critical_depth, notifications, starts, tim2_stops, yields;
static uint32_t pclk = 100000000U;
static int fail_tim3;
static TaskHandle_t supervisor = (void *)(uintptr_t)1;
#define taskENTER_CRITICAL() (void)(++critical_depth)
#define taskEXIT_CRITICAL() (void)(--critical_depth)
#define __HAL_TIM_GET_COUNTER(t) ((t)->Counter)
#define __HAL_TIM_GET_FLAG(t,f) ((t)->UIF)
#define __HAL_TIM_SET_COUNTER(t,n) ((t)->Counter = (n))
#define __HAL_TIM_CLEAR_FLAG(t,f) ((t)->UIF = 0)
static void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c, uint32_t *l)
{ c->APB1CLKDivider = RCC_APB1_DIV2; *l = 1U; }
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
static int HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t)
{ ++starts; return t == &htim3 && fail_tim3 ? 1 : HAL_OK; }
static int HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *t)
{ if (t == &htim2) ++tim2_stops; return HAL_OK; }
static TaskHandle_t supervisor_task_handle(void) { return supervisor; }
static void vTaskNotifyGiveFromISR(TaskHandle_t owner, BaseType_t *w)
{ assert(owner == supervisor); ++notifications; *w = 1; }
#define portYIELD_FROM_ISR(w) do { if (w) ++yields; } while (0)
static uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{ assert(clear == pdTRUE); (void)ticks; uint32_t x = notifications; notifications = 0; return x; }

/* Minimal Supervisor context used by the exact extracted wait/overrun blocks. */
#define ROBOT_STATE_FAULT 50
#define ROBOT_STATE_EMERGENCY_STOP 51
#define ROBOT_STATE_PATH_EXECUTION 42
typedef struct {
    struct { int activeState; } machine;
    struct { struct {
        uint32_t (*wait_next)(void *, TickType_t);
        void *context;
    } timing; TickType_t period_ticks; } config;
    uint32_t timer_missed_periods, timer_timeouts;
    uint32_t timer_overruns, timer_max_cycle_us;
} SupervisorTaskContext;
static unsigned faults;
static uint32_t last_fault;
static void emit_step_failure(SupervisorTaskContext *ctx, uint32_t code)
{ ++faults; last_fault = code; ctx->machine.activeState = ROBOT_STATE_FAULT; }
static uint64_t now_us;
static uint64_t supervisor_now_us(void) { return now_us; }
static uint32_t delivered;
static uint32_t fake_wait(void *unused, TickType_t timeout)
{ (void)unused; assert(timeout == 6); return delivered; }

/* PRODUCTION_TIMER_FUNCTIONS */

static void supervisor_wait_block(SupervisorTaskContext *context)
{
    const bool hardware_timed = true;
    /* PRODUCTION_WAIT_BLOCK */
}
static void supervisor_elapsed_block(SupervisorTaskContext *context,
                                     uint64_t cycle_start_us)
{
    const bool hardware_timed = true;
    /* PRODUCTION_ELAPSED_BLOCK */
}
static void reset(void)
{
    htim2.Instance=TIM2; htim2.Init.Prescaler=199; htim2.Init.Period=UINT32_MAX;
    htim3.Instance=TIM3; htim3.Init.Prescaler=199; htim3.Init.Period=1999;
    htim2.Counter=0; htim3.Counter=0; htim2.UIF=0; htim3.UIF=0;
    g_tim2_wraps=0; g_timing_running=false; critical_depth=0;
    notifications=0; starts=0; tim2_stops=0; yields=0; fail_tim3=0;
    pclk=100000000U; supervisor=(void *)(uintptr_t)1;
}
static SupervisorTaskContext new_context(void)
{
    SupervisorTaskContext c={0};
    c.machine.activeState=ROBOT_STATE_PATH_EXECUTION;
    c.config.timing.wait_next=fake_wait;
    return c;
}
static void test_timestamp_wrap(void)
{
    reset(); assert(timer_start(NULL));
    htim2.Counter=UINT32_MAX-1U;
    assert(time_now_us(NULL)==UINT64_C(4294967294));
    htim2.Counter=1U; htim2.UIF=1;
    assert(time_now_us(NULL)==UINT64_C(4294967297));
    assert(critical_depth==0);
    application_supervisor_timer_period_elapsed_isr(&htim2);
    htim2.UIF=0;
    assert(time_now_us(NULL)==UINT64_C(4294967297));
    puts("PASS: TIM2 pending-wrap and serviced-overflow timestamp");
}
static void test_start_guard(void)
{
    reset(); pclk=32000000U;
    assert(!timer_start(NULL) && starts==0);
    reset(); fail_tim3=1;
    assert(!timer_start(NULL) && !g_timing_running && tim2_stops==1);
    puts("PASS: invalid timer clock and partial HAL startup roll back");
}
static void test_notifications(void)
{
    reset(); application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(notifications==0);
    assert(timer_start(NULL));
    for(int i=0;i<3;i++) application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(notifications==3 && yields==3);
    assert(timer_wait(NULL,6)==3 && timer_wait(NULL,6)==0);
    puts("PASS: TIM3 ISR counting notifications and drain");
}
static void test_missed_cycles(void)
{
    SupervisorTaskContext c=new_context(); faults=0;
    delivered=1; supervisor_wait_block(&c);
    assert(faults==0);
    delivered=4; supervisor_wait_block(&c);
    assert(faults==1 && last_fault==0xE003U && c.timer_missed_periods==3);
    c=new_context(); faults=0;
    delivered=0; supervisor_wait_block(&c);
    assert(faults==1 && last_fault==0xE003U && c.timer_timeouts==1);
    c=new_context(); faults=0;
    c.timer_missed_periods=UINT32_MAX-1U; delivered=5;
    supervisor_wait_block(&c);
    assert(c.timer_missed_periods==UINT32_MAX);
    puts("PASS: missing/coalesced periods fault and counter saturation");
}
static void test_overrun(void)
{
    SupervisorTaskContext c=new_context(); faults=0; now_us=1999;
    supervisor_elapsed_block(&c,0);
    assert(faults==0 && c.timer_max_cycle_us==1999);
    now_us=2000; supervisor_elapsed_block(&c,0);
    assert(faults==1 && last_fault==0xE004U && c.timer_overruns==1);
    now_us=10000; supervisor_elapsed_block(&c,0);
    assert(faults==1 && c.timer_overruns==2 && c.timer_max_cycle_us==10000);
    puts("PASS: 2 ms Supervisor cycle overrun triggers fault");
}
int main(void)
{
    test_timestamp_wrap();
    test_start_guard();
    test_notifications();
    test_missed_cycles();
    test_overrun();
    puts("PASS: real C timing blocks compiled and exercised with host mocks");
    return 0;
}
