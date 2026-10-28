/* Host HAL/FreeRTOS stubs. The timing implementation, including its ISR,
 * is extracted directly from production sources at run time. */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include "supervisor_timing_quality.h"

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

/* Production Supervisor has further application/state fields not needed for
 * the extracted wait/elapsed logic. All timing fields are represented. */
#define ROBOT_STATE_FAULT 50
#define ROBOT_STATE_EMERGENCY_STOP 51
#define ROBOT_STATE_PATH_EXECUTION 42
typedef struct {
    struct { int activeState; } machine;
    struct { struct {
        uint32_t (*wait_next)(void *, TickType_t);
        bool (*observe_wake)(void *, uint32_t, SupervisorTimingWake *);
        void *context;
    } timing; TickType_t period_ticks; } config;
    uint32_t timer_missed_periods, timer_timeouts;
    uint32_t timer_overruns, timer_max_cycle_us;
    uint32_t timer_service_gap_warnings, timer_service_gap_severe;
    uint32_t timer_wake_gap_warnings, timer_wake_gap_severe;
    uint32_t timer_late_wakeups, timer_deadline_misses;
    uint32_t timer_invalid_timestamps, timer_max_isr_to_wake_us;
    uint32_t timer_max_isr_to_completion_us, timer_max_wake_gap_us;
    SupervisorTimingWake last_timing_wake;
    uint64_t previous_timing_wake_us;
    bool last_timing_wake_available, previous_timing_wake_valid;
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
static SupervisorTimingWake injected_wake;
static bool fake_observe(void *unused, uint32_t periods, SupervisorTimingWake *out)
{ (void)unused; (void)periods; *out = injected_wake; return true; }

/* PRODUCTION_TIMER_FUNCTIONS */

static void supervisor_wait_block(SupervisorTaskContext *context)
{
    const bool hardware_timed = true;
    bool progress_cycle_ok = true;
    /* PRODUCTION_WAIT_BLOCK */
    (void)progress_cycle_ok; /* Stage B extraction also checks C2A validity. */
}
static void supervisor_elapsed_block(SupervisorTaskContext *context,
                                     uint64_t cycle_start_us)
{
    const bool hardware_timed = true;
    bool progress_cycle_ok = true;
    /* PRODUCTION_ELAPSED_BLOCK */
    (void)progress_cycle_ok;
}
static void reset(void)
{
    htim2.Instance=TIM2; htim2.Init.Prescaler=199; htim2.Init.Period=UINT32_MAX;
    htim3.Instance=TIM3; htim3.Init.Prescaler=199; htim3.Init.Period=1999;
    htim2.Counter=0; htim3.Counter=0; htim2.UIF=0; htim3.UIF=0;
    g_tim2_wraps=0; g_timing_running=false; critical_depth=0;
    notifications=0; starts=0; tim2_stops=0; yields=0; fail_tim3=0;
    pclk=100000000U; supervisor=(void *)(uintptr_t)1;
    faults=0; last_fault=0; delivered=0; now_us=0;
    injected_wake=(SupervisorTimingWake){0};
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
    for(int i=0;i<3;i++) {
        htim2.Counter=(uint32_t)(2000*(i+1));
        application_supervisor_timer_period_elapsed_isr(&htim3);
    }
    assert(notifications==3 && yields==3);
    assert(timer_wait(NULL,6)==3 && timer_wait(NULL,6)==0);
    SupervisorTimingWake w={0};
    htim2.Counter=6050;
    assert(timer_observe_wake(NULL,3,&w));
    assert(w.timestamp_valid && w.isr_gap_valid && w.isr_service_gap_us==2000);
    assert(w.isr_service_us==6000 && w.wake_sample_us==6050);
    puts("PASS: TIM3 counted notifications and sequence-matched sample");
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
    puts("PASS: missing/coalesced notifications and counter saturation");
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
    puts("PASS: original 2 ms execution overrun behavior remains");
}
static SupervisorTimingWake sample(uint64_t isr,uint64_t wake,uint32_t gap)
{
    SupervisorTimingWake s={0};
    s.timestamp_valid=true; s.isr_service_us=isr;
    s.wake_sample_us=wake; s.isr_service_gap_us=gap;
    s.isr_gap_valid=gap!=0; return s;
}
static void test_nominal(void)
{
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe; injected_wake=sample(2000,2020,0);
    delivered=1; supervisor_wait_block(&c);
    now_us=2400; supervisor_elapsed_block(&c,2020);
    assert(faults==0 && c.timer_max_isr_to_wake_us==20);
    assert(c.timer_max_isr_to_completion_us==400 && c.timer_deadline_misses==0);
    injected_wake=sample(4000,4020,2000); supervisor_wait_block(&c);
    now_us=4400; supervisor_elapsed_block(&c,4020);
    assert(faults==0 && c.timer_max_wake_gap_us==2000);
    puts("PASS: 500 Hz serviced ISR, wake and completion path");
}
static void test_late_wakeup(void)
{
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe;
    injected_wake=sample(2000,3850,0); delivered=1;
    supervisor_wait_block(&c); assert(faults==0);
    now_us=4100; supervisor_elapsed_block(&c,3850);
    assert(c.timer_late_wakeups==0 && c.timer_deadline_misses==1);
    assert(c.timer_overruns==0 && faults==1 && last_fault==0xE004U);
    puts("PASS: one notification + delayed wake + short execution still faults on completion");
}
static void test_delayed_isr_one_notification(void)
{
    reset(); assert(timer_start(NULL));
    htim2.Counter=2000; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    SupervisorTimingWake w={0}; htim2.Counter=2020;
    assert(timer_observe_wake(NULL,1,&w) && w.timestamp_valid);
    htim2.Counter=6500; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    htim2.Counter=6510; assert(timer_observe_wake(NULL,1,&w));
    assert(w.isr_gap_valid && w.isr_service_gap_us==4500);
    SupervisorTimingAssessment a=supervisor_timing_assess(&w,6700,false,0);
    assert(a.valid && a.service_gap_severe);
    SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe; injected_wake=w;
    delivered=1; supervisor_wait_block(&c);
    now_us=6700; supervisor_elapsed_block(&c,6510);
    assert(faults==1 && last_fault==0xE003U && c.timer_service_gap_severe==1);
    puts("PASS: delayed ISR service interval with one delivered notification");
}
static void test_warning_only_gap(void)
{
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe;
    injected_wake=sample(4200,4210,2200); delivered=1;
    supervisor_wait_block(&c); now_us=4400;
    supervisor_elapsed_block(&c,4210);
    assert(faults==0 && c.timer_service_gap_warnings==1);
    assert(c.timer_service_gap_severe==0);
    puts("PASS: >2 ms ISR service jitter is a warning, not automatically a missed update");
}
static void test_matching_during_next_irq(void)
{
    reset(); assert(timer_start(NULL));
    htim2.Counter=2000; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    htim2.Counter=4000; application_supervisor_timer_period_elapsed_isr(&htim3);
    SupervisorTimingWake w={0}; htim2.Counter=4001;
    assert(timer_observe_wake(NULL,1,&w));
    assert(w.timestamp_valid && w.isr_service_us==2000);
    assert(timer_wait(NULL,6)==1);
    assert(timer_observe_wake(NULL,1,&w) && w.isr_service_us==4000);
    puts("PASS: IRQ arriving between notify drain and timestamp read does not mispair");
}
static void test_stale_and_ordering(void)
{
    reset(); assert(timer_start(NULL));
    htim2.Counter=2000; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    for (unsigned i=0;i<8;i++) {
        htim2.Counter=4000U+i*2000U;
        application_supervisor_timer_period_elapsed_isr(&htim3);
    }
    SupervisorTimingWake w={0}; htim2.Counter=18100;
    assert(timer_observe_wake(NULL,1,&w) && !w.timestamp_valid);
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe; delivered=1;
    injected_wake=sample(5000,4500,0);
    supervisor_wait_block(&c); now_us=5500;
    supervisor_elapsed_block(&c,4500);
    assert(c.timer_invalid_timestamps==1 && faults==0);
    reset(); c=new_context(); c.config.timing.observe_wake=fake_observe;
    injected_wake=sample(1000,1200,0); delivered=1;
    supervisor_wait_block(&c); now_us=1199;
    supervisor_elapsed_block(&c,1199);
    assert(c.timer_invalid_timestamps==1 && faults==0);
    reset(); assert(timer_start(NULL));
    htim2.Counter=4000; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    htim2.Counter=4010; assert(timer_observe_wake(NULL,1,&w));
    htim2.Counter=3900; application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    htim2.Counter=3910; assert(timer_observe_wake(NULL,1,&w));
    assert(!w.timestamp_valid);  /* TIM2 moved backward across ISR services. */
    puts("PASS: stale sequence and invalid timestamp ordering reject misleading latency");
}
static void test_task_wake_gap(void)
{
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe; delivered=1;
    injected_wake=sample(2000,2010,0);
    supervisor_wait_block(&c); now_us=2200;
    supervisor_elapsed_block(&c,2010);
    assert(faults==0);
    /* Task has received one notification but wakes more than two periods
     * after the prior wake. The mocked time is not a hardware UIF model. */
    injected_wake=sample(4000,6110,2000);
    supervisor_wait_block(&c); now_us=6300;
    supervisor_elapsed_block(&c,6110);
    assert(faults==1 && last_fault==0xE003U);
    assert(c.timer_wake_gap_severe==1 && c.timer_max_wake_gap_us==4100);
    puts("PASS: two-period task wake gap triggers existing E003 fault");
}
static void test_isr_pending_tim2_wrap(void)
{
    reset(); assert(timer_start(NULL));
    htim2.Counter=1U; htim2.UIF=1;
    application_supervisor_timer_period_elapsed_isr(&htim3);
    assert(timer_wait(NULL,6)==1);
    htim2.Counter=5U;
    SupervisorTimingWake w={0};
    assert(timer_observe_wake(NULL,1,&w));
    assert(w.timestamp_valid && w.isr_service_us==UINT64_C(4294967297));
    assert(w.wake_sample_us==UINT64_C(4294967301));
    application_supervisor_timer_period_elapsed_isr(&htim2);
    htim2.UIF=0;
    assert(time_now_us(NULL)==UINT64_C(4294967301));
    puts("PASS: TIM3 ISR timestamps correct pending TIM2 overflow");
}
static void test_saturating_new_counters(void)
{
    reset(); SupervisorTaskContext c=new_context();
    c.config.timing.observe_wake=fake_observe; delivered=1;
    c.timer_deadline_misses=UINT32_MAX;
    injected_wake=sample(2000,2100,0);
    supervisor_wait_block(&c); now_us=4100;
    supervisor_elapsed_block(&c,2100);
    assert(c.timer_deadline_misses==UINT32_MAX && faults==1);
    puts("PASS: new long-running counters saturate");
}
int main(void)
{
    test_timestamp_wrap();
    test_start_guard();
    test_notifications();
    test_missed_cycles();
    test_overrun();
    test_nominal();
    test_late_wakeup();
    test_delayed_isr_one_notification();
    test_warning_only_gap();
    test_matching_during_next_irq();
    test_stale_and_ordering();
    test_task_wake_gap();
    test_isr_pending_tim2_wrap();
    test_saturating_new_counters();
    puts("PASS: extracted production timer/diagnostic C exercised with host mocks");
    return 0;
}
