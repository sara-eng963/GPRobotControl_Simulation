/* P04 deterministic integrated source-level fault injection.
 * Compiles the actual production Supervisor translation unit through the
 * existing P01/P02/P03 alignment fixture. No synthetic Supervisor is defined.
 * The existing fixture's source-owned helpers and fake devices supply only
 * controllable scheduling, queue, IPC and drive observations.
 */
#define main p04_legacy_supervisor_main
#include "test_production_supervisor.c"
#undef main

#ifndef ALIGNMENT_DESKTOP
#define P04_REQUIRE(x) do { if (!(x)) { \
    fprintf(stderr, "P04_ASSERT_FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); \
    exit(41); \
} } while (0)
#define P04_PASS(id) puts("P04_SCENARIO " id " PASS injected-time")

static SupervisorTimingWake p04_wake(uint64_t isr, uint64_t wake,
                                    uint32_t service_gap)
{
    SupervisorTimingWake s = {0};
    s.timestamp_valid = true;
    s.isr_service_us = isr;
    s.wake_sample_us = wake;
    s.isr_gap_valid = service_gap != 0U;
    s.isr_service_gap_us = service_gap;
    return s;
}
static void p04_prep(RobotStateId state, uint32_t now)
{
    setup(state);
    p03_ready_evidence(&ctx, now);
    p03_prepare_pre_dispatch();
    p02_motion_ready_master(now);
    now_tick = now;
}
static void p04_timing_fault(uint32_t detail, uint64_t isr,
                             uint64_t wake, uint64_t task_now,
                             uint32_t service_gap)
{
    p04_prep(ROBOT_STATE_APPROACH, 100U);
    ctx.last_timing_wake_available = true;
    ctx.last_timing_wake = p04_wake(isr,wake,service_gap);
    P04_REQUIRE(!supervisor_timing_pre_dispatch_check(&ctx,task_now));
    P04_REQUIRE(ctx.machine.activeState == ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.machine.activeFaultCode ==
               state_error_code(ROBOT_STATE_APPROACH, detail));
    P04_REQUIRE(ctx.restart_gate.inhibited);
    P04_REQUIRE(!master.drive_command_permission);
    unsigned previous_stop = stop_calls;
    run_active_state(&ctx);
    P04_REQUIRE(ctx.fault_response_step_ok);
    P04_REQUIRE(stop_calls == previous_stop);
    P04_REQUIRE(drive.commands == 0U);
}
static void p04_scenario_01(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.last_timing_wake_available=true;
    ctx.last_timing_wake=p04_wake(200000U,200100U,2000U);
    P04_REQUIRE(supervisor_timing_pre_dispatch_check(&ctx,200101U));
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_APPROACH);
    P04_REQUIRE(master.drive_command_permission);
    P04_PASS("P04-01");
}
static void p04_scenario_02(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.peer_evidence.last_response_ms=49U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    P04_REQUIRE(ctx.machine.activeFaultCode ==
               state_error_code(ROBOT_STATE_APPROACH,0xE018U));
    P04_PASS("P04-02");
}
static void p04_scenario_03(void)
{
    p04_timing_fault(0xE004U,200000U,202000U,202000U,2000U);
    p04_timing_fault(0xE003U,200000U,200010U,200011U,4000U);
    p04_timing_fault(0xE019U,200020U,200010U,200030U,2000U);
    p04_prep(ROBOT_STATE_PATH_EXECUTION,100U);
    ctx.last_timing_wake_available=true;
    ctx.last_timing_wake=p04_wake(200000U,200010U,2000U);
    P04_REQUIRE(supervisor_timing_pre_dispatch_check(&ctx,200011U));
    /* Deadline-overrun detection itself is exercised by the existing
     * source-extracted m7_timer_fault_host 1999/2000 boundary tests. */
    P04_PASS("P04-03");
}
static void p04_scenario_04(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    p02_enqueue_noop_batch(SUPERVISOR_MAX_MESSAGES_CYCLE+2U);
    ctx.safety_evidence.last_source_ms=0U;
    ctx.safety_evidence.last_receipt_ms=0U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.machine.activeFaultCode==
               state_error_code(ROBOT_STATE_APPROACH,0xE016U));
    P04_REQUIRE(!master.drive_command_permission);
    /* Expiry must independently fault without the overload masking it.
     * The 100 ms freshness boundary is inclusive; 101 ms is expired. */
    p04_prep(ROBOT_STATE_APPROACH,101U);
    ctx.safety_evidence.last_source_ms=0U;
    ctx.safety_evidence.last_receipt_ms=0U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.machine.activeFaultCode==
               state_error_code(ROBOT_STATE_APPROACH,0xE017U));
    P04_REQUIRE(!master.drive_command_permission);
    P04_PASS("P04-04");
}
static void p04_scenario_05(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    P04_REQUIRE(supervisor_motion_containment_check(&ctx,100U,false));
    drive.axis[2].feedback_valid=false;
    P04_REQUIRE(!supervisor_motion_containment_check(&ctx,100U,false));
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.machine.activeFaultCode==
               state_error_code(ROBOT_STATE_APPROACH,0xE202U));
    P04_REQUIRE(runtime_peer_fresh(&ctx.peer_evidence,100U));
    P04_REQUIRE(!master.drive_command_permission);
    P04_PASS("P04-05");
}
static void p04_scenario_06(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    drive.axis[0].feedback_valid=false;
    ctx.peer_evidence.last_response_ms=0U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    run_active_state(&ctx);
    P04_REQUIRE(ctx.fault_response_step_ok);
    P04_PASS("P04-06");
}
static void p04_scenario_07(void)
{
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.peer_evidence.last_response_ms=0U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    p03_ready_evidence(&ctx,101U);
    drive.axis[0].feedback_valid=true;
    now_tick=101U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.restart_gate.inhibited);
    P04_REQUIRE(!master.drive_command_permission);
    P04_PASS("P04-07");
}
static void p04_scenario_08(void)
{
    p04_prep(ROBOT_STATE_PATH_EXECUTION,100U);
    for(unsigned cycle=0;cycle<3U;cycle++) {
        p02_enqueue_noop_batch(SUPERVISOR_MAX_MESSAGES_CYCLE+1U);
        now_tick=100U+cycle;
        supervisor_pre_dispatch_cycle(&ctx);
        P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
        P04_REQUIRE(!master.drive_command_permission);
        run_active_state(&ctx);
        P04_REQUIRE(ctx.fault_response_step_ok);
        p02_queue_reset();
    }
    P04_PASS("P04-08");
}
static void p04_scenario_09(void)
{
    RuntimePeerEvidence p={.expected_session=71U};
    IpcMessage hello=p03_hello(71U);
    P04_REQUIRE(runtime_peer_on_hello(&p,&hello));
    uint32_t challenge=0;
    P04_REQUIRE(runtime_peer_challenge(&p,UINT32_MAX-5U,&challenge));
    IpcMessage reply=p03_progress(71U,1U,challenge);
    P04_REQUIRE(runtime_peer_accept(&p,&reply,UINT32_MAX));
    P04_REQUIRE(runtime_peer_fresh(&p,UINT32_MAX));
    P04_REQUIRE(!runtime_peer_fresh(&p,50U));
    P04_REQUIRE(!runtime_peer_accept(&p,&reply,0U));
    RuntimeSafetyPort port={.verify=p03_fixture_verify,.max_age_ms=100U};
    RuntimeSafetyEvidence safety={.accepted=true,.last_source_ms=UINT32_MAX-1U,
        .last_receipt_ms=UINT32_MAX-1U};
    P04_REQUIRE(runtime_safety_fresh(&safety,&port,2U));
    P04_REQUIRE(!runtime_safety_fresh(&safety,&port,200U));
    P04_PASS("P04-09");
}
static void p04_scenario_10(void)
{
    p04_prep(ROBOT_STATE_PATH_EXECUTION,100U);
    p02_enqueue_noop_batch(SUPERVISOR_MAX_MESSAGES_CYCLE+1U);
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    p02_queue_reset();
    ctx.last_timing_wake_available=true;
    ctx.last_timing_wake=p04_wake(200000U,202001U,2020U);
    P04_REQUIRE(!supervisor_timing_pre_dispatch_check(&ctx,202001U));
    run_active_state(&ctx);
    P04_REQUIRE(ctx.fault_response_step_ok);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    P04_PASS("P04-10");
}
/* P04 director verification: run the actual production task entry, not
 * merely its separately callable timing policy helper. One synthetic release
 * is completed; the next wait exits through a test-only setjmp boundary.
 * All time is injected task-local CM7 time (not a hardware measurement). */
static jmp_buf p04_task_loop_exit;
static uint64_t p04_task_clock_us;
static unsigned p04_task_release_calls;
static SupervisorTimingWake p04_task_observation;
static bool p04_task_observation_present;
static unsigned p04_task_input_calls;

static uint64_t p04_task_clock(void *unused)
{
    (void)unused;
    return p04_task_clock_us;
}
static bool p04_task_timer_start(void *unused)
{
    (void)unused;
    return true;
}
static uint32_t p04_task_wait_release(void *unused, TickType_t ticks)
{
    (void)unused;
    P04_REQUIRE(ticks == pdMS_TO_TICKS(6U));
    if (++p04_task_release_calls == 2U)
        longjmp(p04_task_loop_exit, 1);
    P04_REQUIRE(p04_task_release_calls == 1U);
    return 1U;
}
static bool p04_task_observe(void *unused, uint32_t releases,
                             SupervisorTimingWake *out)
{
    (void)unused;
    P04_REQUIRE(releases == 1U && out != NULL);
    if (!p04_task_observation_present) return false;
    *out = p04_task_observation;
    return true;
}
/* Invalid timing must be rejected BEFORE any normal input acquisition.
 * If the initial preflight call is removed, this assertion fails even if
 * the later recheck would eventually detect the same invalid timestamp. */
static bool p04_task_input_requires_prior_fault(void *unused,
                                                  SupervisorInputSnapshot *out)
{
    ++p04_task_input_calls;
    P04_REQUIRE(ctx.machine.activeState == ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.machine.activeFaultCode ==
                state_error_code(ROBOT_STATE_APPROACH, 0xE019U));
    return p03_good_board_inputs(unused, out);
}
/* The initial wake is legal; model bounded input processing consuming time.
 * This is an injected delay, NOT a claim that GPIO acquisition takes 1901 us. */
static bool p04_task_input_expires_deadline(void *unused,
                                             SupervisorInputSnapshot *out)
{
    ++p04_task_input_calls;
    P04_REQUIRE(ctx.machine.activeState == ROBOT_STATE_APPROACH);
    p04_task_clock_us = UINT64_C(102001); /* 2001 us from serviced TIM3 ISR */
    return p03_good_board_inputs(unused, out);
}
/* The desktop fixture's hardware-timed task path does not call this API,
 * but the real task function includes the non-hardware fallback branch. */
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks)
{
    (void)previous;
    (void)ticks;
    P04_REQUIRE(false); /* unexpected non-hardware scheduling path */
}
static void p04_task_entry_setup(bool initial_valid)
{
    p04_prep(ROBOT_STATE_APPROACH, 100U);
    ctx.config.ipc = (IpcShared *)&ctx; /* established mock mailbox */
    ctx.config.reset_diagnostics.captured = true;
    ctx.config.reset_diagnostics.cause = RESET_CAUSE_POWER_ON;
    ctx.config.drive_restart_board.read = p02_board_read;
    ctx.boot.preenable_status_checked = true;
    p02_authorization_press = true;
    master.controller_heartbeat_sent = true;
    master.last_controller_heartbeat_ms = 100U;
    /* Task entry itself binds the actual production CANopen owner; the
     * existing fixture normally installs its fake owner before this point. */
    master.owner_check = NULL;
    master.owner_check_context = NULL;
    for (unsigned axis = 0U; axis < JOINT_DRIVE_AXES; ++axis)
        drive.axis[axis].timestamp_us = UINT64_C(100000);
    p04_task_clock_us = UINT64_C(100100);
    p04_task_release_calls = 0U;
    p04_task_input_calls = 0U;
    p04_task_observation_present = initial_valid;
    p04_task_observation = p04_wake(UINT64_C(100000),
                                     UINT64_C(100100), 2000U);
    ctx.config.timing = (SupervisorTimingPort){
        .monotonic_us = p04_task_clock,
        .start = p04_task_timer_start,
        .wait_next = p04_task_wait_release,
        .observe_wake = p04_task_observe
    };
    g_timing = ctx.config.timing; /* same source used by supervisor_now_us */
    ctx.approach_control.home_requested = true; /* sentinel: untouched if no Approach step */
}
static void p04_task_entry_one_cycle(void)
{
    if (setjmp(p04_task_loop_exit) == 0)
        supervisor_task_entry(&ctx);
    P04_REQUIRE(p04_task_release_calls == 2U);
    P04_REQUIRE(p04_task_input_calls == 1U);
    P04_REQUIRE(ctx.machine.activeState == ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.restart_gate.inhibited);
    P04_REQUIRE(!master.drive_command_permission);
    /* The next synthetic iteration clears fault_response_step_ok
     * before wait_next longjmps. Check persistent evidence that
     * first-cycle FAULT safe-output servicing completed instead. */
    P04_REQUIRE(ctx.fault.initialized);
    P04_REQUIRE(ctx.fault.phase == FAULT_PHASE_WAIT_CAUSE_CLEAR);
    P04_REQUIRE(off_calls > 0U);
    P04_REQUIRE(ctx.approach_control.home_requested); /* no motion-state step */
    P04_REQUIRE(drive.commands == 0U);
    P04_REQUIRE(critical_depth == 0U);
}
static void p04_task_entry_timing_contracts(void)
{
    /* Case 1: prove the INITIAL production call site precedes input work. */
    p04_task_entry_setup(false);
    ctx.config.io.read_inputs = p04_task_input_requires_prior_fault;
    p04_task_entry_one_cycle();
    P04_REQUIRE(ctx.machine.activeFaultCode ==
                state_error_code(ROBOT_STATE_APPROACH, 0xE019U));

    /* Case 2: initial preflight passes; input work crosses 2 ms. The
     * dispatch-adjacent second check must fault BEFORE Approach stepping. */
    p04_task_entry_setup(true);
    ctx.config.io.read_inputs = p04_task_input_expires_deadline;
    p04_task_entry_one_cycle();
    P04_REQUIRE(ctx.machine.activeFaultCode ==
                state_error_code(ROBOT_STATE_APPROACH, 0xE004U));
    P04_REQUIRE(ctx.timer_overruns == 0U); /* distinct from execution overrun */
    P04_REQUIRE(ctx.timer_deadline_misses == 1U);
}

static void p04_boundary_tests(void)
{
    const uint64_t isr=UINT64_C(100000);
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.last_timing_wake_available=true;
    ctx.last_timing_wake=p04_wake(isr,isr+1999U,2000U);
    P04_REQUIRE(supervisor_timing_pre_dispatch_check(&ctx,isr+1999U));
    p04_timing_fault(0xE004U,isr,isr+2000U,isr+2000U,2000U);
    p04_timing_fault(0xE004U,isr,isr+2001U,isr+2001U,2000U);
    /* A perfectly legal 1999us wake can become late before dispatch. */
    p04_timing_fault(0xE004U,isr,isr+1999U,isr+2000U,2000U);
    p04_prep(ROBOT_STATE_APPROACH,100U);
    P04_REQUIRE(!supervisor_timing_pre_dispatch_check(&ctx,100001U));
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_PASS("P04-BOUNDARY");
}
#endif
int main(void)
{
#ifndef ALIGNMENT_DESKTOP
    robot_config_init_ur5(&robot);
    for(unsigned i=0;i<ROBOT_DOF;++i)
        P04_REQUIRE(avatar_m_position_scale_default(&scales[i],50.0));
    p04_scenario_01();p04_scenario_02();p04_scenario_03();
    p04_scenario_04();p04_scenario_05();p04_scenario_06();
    p04_scenario_07();p04_scenario_08();p04_scenario_09();
    p04_scenario_10();p04_task_entry_timing_contracts();p04_boundary_tests();
#endif
    return p04_legacy_supervisor_main();
}
