#include "../hmi_lifecycle.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static GpIpcMessage message(uint32_t sequence, RobotStateId state)
{
    GpIpcMessage msg = {0};
    msg.type = GP_IPC_CONTROLLER_STATUS;
    msg.sequence = sequence;
    msg.data[0] = (uint32_t)state;
    msg.data[1] = 1U;
    msg.data[3] = 10U;
    return msg;
}

static void test_status_input_validation(void)
{
    GpHmiControllerStatus status = {0};
    GpIpcMessage msg = message(100U, ROBOT_STATE_IDLE);
    assert(gp_hmi_status_decode(&msg, 0U, &status));
    assert(status.ipc_sequence == 100U && status.robot_state == ROBOT_STATE_IDLE);
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg.sequence = 99U;
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg = message(102U, ROBOT_STATE_COUNT);
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg = message(103U, ROBOT_STATE_IDLE);
    msg.data[1] = 2U;
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg.data[1] = 1U;
    msg.data[4] = 7U;
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg.data[4] = 0U;
    msg.type = GP_IPC_STORAGE_RESULT;
    assert(!gp_hmi_status_decode(&msg, 100U, &status));
    msg = message(UINT32_MAX, ROBOT_STATE_IDLE);
    assert(gp_hmi_status_decode(&msg, UINT32_MAX-1U, &status));
    msg = message(1U, ROBOT_STATE_IDLE);
    assert(gp_hmi_status_decode(&msg, UINT32_MAX, &status));
    assert(!gp_hmi_status_decode(&msg, UINT32_MAX, &status));
    puts("[PASS] M4 status ABI, invalid frames, stale sequences and rollover");
}

static void test_freshness_and_policy(void)
{
    assert(gp_hmi_status_is_fresh(1020U, 1000U, 100U));
    assert(!gp_hmi_status_is_fresh(1101U, 1000U, 100U));
    assert(!gp_hmi_status_is_fresh(1001U, 1000U, 0U));
    assert(gp_hmi_status_is_fresh(3U, UINT32_MAX-4U, 10U));
    GpHmiControllerStatus status = {0};
    status.robot_state = ROBOT_STATE_IDLE;
    assert(gp_hmi_command_allowed(&status, SUP_HMI_START_REPLAY));
    assert(gp_hmi_command_allowed(&status, SUP_HMI_HOME));
    assert(!gp_hmi_command_allowed(&status, SUP_HMI_COMMAND_COUNT));
    GpHmiControllerStatus prior = status;
    status.robot_state = ROBOT_STATE_PATH_EXECUTION;
    assert(gp_hmi_should_drop_pending(&prior, &status));
    status.robot_state = ROBOT_STATE_FAULT;
    status.active_fault = 0x1001U;
    assert(gp_hmi_should_drop_pending(&prior, &status));
    assert(!gp_hmi_command_allowed(&status, SUP_HMI_START_REPLAY));
    assert(gp_hmi_command_allowed(&status, SUP_HMI_RESET));
    status.estop_active = true;
    assert(!gp_hmi_command_allowed(&status, SUP_HMI_RESET));
    status.estop_active = false;
    status.robot_state = ROBOT_STATE_EMERGENCY_STOP;
    assert(!gp_hmi_command_allowed(&status, SUP_HMI_HOME));
    puts("[PASS] CM4 local freshness, fault, estop and state-change flush policy");
}

int main(void)
{
    test_status_input_validation();
    test_freshness_and_policy();
    puts("PASS: B5 transport-neutral HMI lifecycle host regression");
    return 0;
}
