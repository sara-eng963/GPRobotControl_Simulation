#include "drive_restart_gate.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static DriveRestartBoardSample board;
static unsigned reads;
static bool read_board(void *ctx, DriveRestartBoardSample *out)
{
    (void)ctx;
    ++reads;
    *out = board;
    return true;
}
static bool denied_board(void *ctx, DriveRestartBoardSample *out)
{ (void)ctx; (void)out; return false; }
static const DriveRestartBoardPort provider = {read_board, 0};
static const DriveRestartBoardPort denied_provider = {denied_board, 0};
static DriveRestartSystemSample good = {true,true,true,true,true,true,true,true};
#define DENIED(g,r,p,s) assert(!drive_restart_gate_tick((g),(r),(p),(s)))
#define ALLOWED(g,r,p,s) assert(drive_restart_gate_tick((g),(r),(p),(s)))

static void base(void)
{
    board = (DriveRestartBoardSample){true,false,true,false,true,true,true};
    reads = 0;
}
static void startup_case(uint32_t raw, bool reviewed)
{
    ResetDiagnostics reset = reset_diagnostics_from_raw(raw);
    DriveRestartGate g;
    drive_restart_gate_init(&g);
    base(); board.reset_reviewed = reviewed;
    DENIED(&g,&reset,&provider,&good);
    board.operator_pressed = true;
    ALLOWED(&g,&reset,&provider,&good);
    assert(g.authorized && reads == 2);
}
int main(void)
{
    /* Power-on flags can include BOR/PIN/core/domain flags concurrently. */
    startup_case(RESET_FLAG_POR | RESET_FLAG_BOR | RESET_FLAG_PIN |
                 RESET_FLAG_C1 | RESET_FLAG_C2, false);
    startup_case(RESET_FLAG_IWDG1 | RESET_FLAG_PIN, true);
    startup_case(RESET_FLAG_WWDG2 | RESET_FLAG_C2, true);
    startup_case(RESET_FLAG_BOR | RESET_FLAG_PIN, true);
    startup_case(RESET_FLAG_SOFT1 | RESET_FLAG_PIN, true);
    startup_case(RESET_FLAG_SOFT2 | RESET_FLAG_C2, true);
    startup_case(RESET_FLAG_C1, true);
    startup_case(RESET_FLAG_POR | RESET_FLAG_SOFT1, true);
    assert(reset_diagnostics_classify(RESET_FLAG_POR) == RESET_CAUSE_POWER_ON);
    assert(reset_diagnostics_classify(RESET_FLAG_BOR) == RESET_CAUSE_BROWNOUT);
    assert(reset_diagnostics_classify(RESET_FLAG_IWDG1) == RESET_CAUSE_WATCHDOG);
    assert(reset_diagnostics_classify(RESET_FLAG_SOFT1) == RESET_CAUSE_SOFTWARE);
    assert(reset_diagnostics_classify(RESET_FLAG_C2) == RESET_CAUSE_CPU_OR_DOMAIN);
    assert(reset_diagnostics_classify(0U) == RESET_CAUSE_UNKNOWN);
    assert(reset_diagnostics_classify(RESET_FLAG_POR | RESET_FLAG_SOFT1) == RESET_CAUSE_MULTIPLE);

    ResetDiagnostics r = reset_diagnostics_from_raw(RESET_FLAG_POR);
    DriveRestartGate g;
    drive_restart_gate_init(&g);
    base();
    DriveRestartSystemSample invalid_state = good;
    invalid_state.state_ready = false;
    DENIED(&g,&r,&provider,&invalid_state);
    DENIED(&g,&r,NULL,&good); /* absent production provider */
    DENIED(&g,&r,&denied_provider,&good);
    board.operator_pressed = true; /* stale/held input from prior boot */
    DENIED(&g,&r,&provider,&good);
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = false;
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = true;
    ALLOWED(&g,&r,&provider,&good);
    /* Falling edge is not withdrawal; independent readiness is. */
    board.operator_pressed = false;
    ALLOWED(&g,&r,&provider,&good);
    board.board_ready = false;
    DENIED(&g,&r,&provider,&good);
    board.board_ready = true;
    board.operator_pressed = false;
    DENIED(&g,&r,&provider,&good); /* latched inhibition */
    board.operator_pressed = true;
    DENIED(&g,&r,&provider,&good);
    drive_restart_gate_init(&g);
    base(); board.ipc_session_fresh = false;
    DENIED(&g,&r,&provider,&good);
    board.ipc_session_fresh = true;
    board.old_commands_discarded = false;
    DENIED(&g,&r,&provider,&good);
    board.old_commands_discarded = true;
    board.feedback_revalidated = false;
    DENIED(&g,&r,&provider,&good);
    board.feedback_revalidated = true;
    DriveRestartSystemSample broken = good;
    broken.estop_clear = false;
    DENIED(&g,&r,&provider,&broken);
    board.operator_pressed = true; /* never observed release while ready */
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = false;
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = true;
    ALLOWED(&g,&r,&provider,&good);
    broken = good;
    broken.fault_clear = false;
    DENIED(&g,&r,&provider,&broken);
    DENIED(&g,&r,&provider,&good); /* reset cannot bypass */
    drive_restart_gate_init(&g);
    base();
    ResetDiagnostics missing = {0};
    DENIED(&g,&missing,&provider,&good);
    r = reset_diagnostics_from_raw(RESET_FLAG_IWDG1);
    board.reset_reviewed = false;
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = true;
    DENIED(&g,&r,&provider,&good);
    board.reset_reviewed = true;
    DENIED(&g,&r,&provider,&good); /* held approval cannot become fresh */
    board.operator_pressed = false;
    DENIED(&g,&r,&provider,&good);
    board.operator_pressed = true;
    ALLOWED(&g,&r,&provider,&good);
    puts("PASS: C2B reset classification and live-edge default-deny policy");
    return 0;
}
