/* P05 production-source safety verification reuses the P04 fixture and the
 * actual STM32 CM7 supervisor_task.c, not the desktop state machine.
 * This is injected host time and fake external devices, not physical WCET.
 */
/* Compiled immediately after the original P04 fixture in a single
 * translation unit by run_state_machine_alignment_tests.py. */

int main(void)
{
#ifndef ALIGNMENT_DESKTOP
    robot_config_init_ur5(&robot);
    for (unsigned i=0U;i<ROBOT_DOF;++i)
        P04_REQUIRE(avatar_m_position_scale_default(&scales[i],50.0));

    /* Authority must fall before the next Approach dispatch. */
    p04_prep(ROBOT_STATE_APPROACH,100U);
    P04_REQUIRE(master.drive_command_permission);
    drive.axis[3].feedback_valid=false;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    P04_REQUIRE(ctx.restart_gate.inhibited);
    puts("P05_SUPERVISOR feedback_loss PASS");

    /* A fresh CM4 challenge/response is mandatory. Restoring simulated
     * feedback cannot re-enter a motion state on a previously latched fault. */
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.peer_evidence.last_response_ms=0U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    p03_ready_evidence(&ctx,101U);
    now_tick=101U;
    supervisor_pre_dispatch_cycle(&ctx);
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    puts("P05_SUPERVISOR cm4_lost_and_recovered PASS");

    /* Deadline exceeded before dispatch: actual production timing evaluator
     * revokes permission. No later recovery from clock alone. */
    p04_prep(ROBOT_STATE_APPROACH,100U);
    ctx.last_timing_wake_available=true;
    ctx.last_timing_wake=p04_wake(100000U,102000U,2000U);
    P04_REQUIRE(!supervisor_timing_pre_dispatch_check(&ctx,102000U));
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(ctx.restart_gate.inhibited);
    P04_REQUIRE(!master.drive_command_permission);
    ctx.last_timing_wake=p04_wake(104000U,104100U,2000U);
    P04_REQUIRE(supervisor_timing_pre_dispatch_check(&ctx,104100U));
    P04_REQUIRE(ctx.machine.activeState==ROBOT_STATE_FAULT);
    P04_REQUIRE(!master.drive_command_permission);
    puts("P05_SUPERVISOR deadline_and_no_reauth PASS");

    /* Actual production task loop's early and late-before-dispatch checks. */
    p04_task_entry_timing_contracts();
    puts("P05_SUPERVISOR real_task_entry PASS");
#endif
    return 0;
}
