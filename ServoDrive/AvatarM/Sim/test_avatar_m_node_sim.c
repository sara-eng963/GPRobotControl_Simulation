#include "avatar_m_node_sim.h"

#include "../avatar_m_pdo.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(condition, message)         \
    do                                    \
    {                                     \
        if (condition)                    \
        {                                 \
            printf("[PASS] %s\n", message); \
        }                                 \
        else                              \
        {                                 \
            printf("[FAIL] %s\n", message); \
            failures++;                   \
        }                                 \
    } while (0)

int main(void)
{
    AvatarMNodeSim node;
    CanFrame response = {0};

    CHECK(
        avatar_m_node_init(&node, 1U),
        "Node 1 initialized"
    );

    avatar_m_node_set_work_mode(&node, 7U);
    avatar_m_node_set_actual_position(&node, 3448);
    avatar_m_node_set_statusword(&node, 0x0437U);

    /* NMT Start for node 1. */
    CanFrame nmt_start =
    {
        .id = 0x000U,
        .dlc = 2U,
        .data = {0x01U, 0x01U}
    };

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &nmt_start,
            &response
        ) == AVATAR_M_NODE_NO_RESPONSE,
        "NMT Start accepted"
    );

    CHECK(
        node.nmt_state == AVATAR_M_NMT_OPERATIONAL,
        "Node entered operational state"
    );

    /* Manual interpolation example: Node 1 target position = 50000. */
    CanFrame rpdo4;

    CHECK(
        avatar_m_build_rpdo4(
            1U,
            50000,
            &rpdo4
        ),
        "RPDO4 built"
    );

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &rpdo4,
            &response
        ) == AVATAR_M_NODE_RESPONSE,
        "RPDO4 produced immediate TPDO4"
    );

    CHECK(
        node.cached_target_valid,
        "RPDO4 target cached"
    );

    CHECK(
        node.cached_target_position == 50000,
        "Cached target is 50000"
    );

    CHECK(
        !node.active_target_valid,
        "Target not active before SYNC"
    );

    CHECK(
        node.actual_position == 3448,
        "Actual position unchanged before SYNC"
    );

    /* Manual TPDO4 example: 481 78 0D 00 00 37 04. */
    CHECK(
        response.id == 0x481U,
        "TPDO4 CAN-ID is 0x481"
    );

    CHECK(
        response.dlc == 6U,
        "TPDO4 DLC is 6"
    );

    CHECK(
        response.data[0] == 0x78U &&
        response.data[1] == 0x0DU &&
        response.data[2] == 0x00U &&
        response.data[3] == 0x00U &&
        response.data[4] == 0x37U &&
        response.data[5] == 0x04U,
        "TPDO4 payload matches manual example"
    );

    CanFrame sync;
    avatar_m_build_sync(&sync);

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &sync,
            &response
        ) == AVATAR_M_NODE_NO_RESPONSE,
        "SYNC processed"
    );

    CHECK(
        node.active_target_valid,
        "Target became active after SYNC"
    );

    CHECK(
        node.active_target_position == 50000,
        "Active target is 50000"
    );

    CHECK(
        node.actual_position == 3448,
        "SYNC does not fake physical motion"
    );

    CanFrame heartbeat = {0};

    CHECK(
        avatar_m_node_tick_ms(
            &node,
            999U,
            &heartbeat
        ) == false,
        "Heartbeat not generated before 1000 ms"
    );

    CHECK(
        avatar_m_node_tick_ms(
            &node,
            1U,
            &heartbeat
        ),
        "Heartbeat generated at 1000 ms"
    );

    CHECK(
        heartbeat.id == 0x701U,
        "Heartbeat CAN-ID is 0x701"
    );

    CHECK(
        heartbeat.dlc == 1U &&
        heartbeat.data[0] == 0x05U,
        "Operational heartbeat payload is 0x05"
    );

    /* Pre-operational: PDO must not be processed; heartbeat becomes 0x7F. */
    CanFrame nmt_preop =
    {
        .id = 0x000U,
        .dlc = 2U,
        .data = {0x80U, 0x01U}
    };

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &nmt_preop,
            &response
        ) == AVATAR_M_NODE_NO_RESPONSE,
        "NMT pre-operational accepted"
    );

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &rpdo4,
            &response
        ) == AVATAR_M_NODE_NO_RESPONSE,
        "RPDO4 ignored in pre-operational state"
    );

    CHECK(
        avatar_m_node_tick_ms(
            &node,
            1000U,
            &heartbeat
        ),
        "Pre-operational heartbeat generated"
    );

    CHECK(
        heartbeat.data[0] == 0x7FU,
        "Pre-operational heartbeat payload is 0x7F"
    );

    /* NMT reset communication: one boot-up heartbeat 0x00 then pre-op. */
    CanFrame nmt_reset_comm =
    {
        .id = 0x000U,
        .dlc = 2U,
        .data = {0x82U, 0x01U}
    };

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &nmt_reset_comm,
            &response
        ) == AVATAR_M_NODE_RESPONSE,
        "NMT reset communication emits boot-up heartbeat"
    );

    CHECK(
        response.id == 0x701U &&
        response.dlc == 1U &&
        response.data[0] == 0x00U,
        "Reset boot-up heartbeat is 0x00"
    );

    CHECK(
        node.nmt_state == AVATAR_M_NMT_PRE_OPERATIONAL,
        "Reset returns node to pre-operational state"
    );

    /* NMT stop: heartbeat must be disabled by the vendor NMT table. */
    CanFrame nmt_stop =
    {
        .id = 0x000U,
        .dlc = 2U,
        .data = {0x02U, 0x01U}
    };

    CHECK(
        avatar_m_node_process_frame(
            &node,
            &nmt_stop,
            &response
        ) == AVATAR_M_NODE_NO_RESPONSE,
        "NMT Stop accepted"
    );

    CHECK(
        avatar_m_node_tick_ms(
            &node,
            1000U,
            &heartbeat
        ) == false,
        "Heartbeat disabled in stopped state"
    );

    AvatarMNodeSim wd;
    CanFrame wd_reply = {0};
    CHECK(avatar_m_node_init(&wd, 2U), "Watchdog drive initialized");
    CHECK(wd.heartbeat_consumer_value == 0x007F07D0UL,
          "Watchdog default 0x007F07D0 matches manual");
    CanFrame host_hb = {.id = 0x77FU, .dlc = 1U, .data = {0x05U}};
    CHECK(!wd.consumer_heartbeat_seen, "No watchdog before first heartbeat");
    (void)avatar_m_node_process_frame(&wd, &host_hb, &wd_reply);
    CHECK(wd.consumer_heartbeat_seen, "Node 127 heartbeat starts monitoring");
    (void)avatar_m_node_tick_ms(&wd, 1999U, &wd_reply);
    CHECK(!wd.communication_drop_alarm, "No timeout before 2000ms");
    (void)avatar_m_node_tick_ms(&wd, 1U, &wd_reply);
    CHECK(wd.communication_drop_alarm, "2000ms timeout stops simulated drive");
    (void)avatar_m_node_process_frame(&wd, &host_hb, &wd_reply);
    CHECK(!wd.communication_drop_alarm, "Resumed heartbeat clears stop");
    CanFrame hb_sdo = {.id = 0x602U, .dlc = 8U,
                      .data = {0x23U, 0x16U, 0x10U, 0x01U,
                               0xF4U, 0x01U, 0x7FU, 0x00U}};
    CHECK(avatar_m_node_process_frame(&wd, &hb_sdo, &wd_reply) ==
          AVATAR_M_NODE_RESPONSE && wd_reply.data[0] == 0x60U,
          "0x1016:01 accepts 500ms watch interval");
    CHECK(wd.heartbeat_consumer_value == 0x007F01F4UL,
          "Read-back state stores 500ms consumer interval");
    (void)avatar_m_node_process_frame(&wd, &host_hb, &wd_reply);
    (void)avatar_m_node_tick_ms(&wd, 500U, &wd_reply);
    CHECK(wd.communication_drop_alarm, "500ms timeout triggers");
    printf("\nTests failed: %d\n", failures);

    return failures != 0;
}
