#include "../CANopen/canopen_master.h"
#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define FAKE_CAPACITY 64U

typedef struct
{
    CanFrame tx[FAKE_CAPACITY];
    size_t tx_count;

    CanFrame rx[FAKE_CAPACITY];
    size_t rx_head;
    size_t rx_tail;

} FakeCan;

static int failures = 0;

#define CHECK(condition, message)     do     {         if (condition)         {             printf("[PASS] %s\n", message);         }         else         {             printf("[FAIL] %s\n", message);             failures++;         }     } while (0)

static CanBackendResult fake_send(
    void *context,
    const CanFrame *frame
)
{
    FakeCan *fake =
        (FakeCan *)context;

    if (
        fake == NULL ||
        frame == NULL ||
        fake->tx_count >= FAKE_CAPACITY
    )
    {
        return CAN_BACKEND_ERROR;
    }

    fake->tx[fake->tx_count++] =
        *frame;

    return CAN_BACKEND_OK;
}

static CanBackendResult fake_receive(
    void *context,
    CanFrame *frame
)
{
    FakeCan *fake =
        (FakeCan *)context;

    if (
        fake == NULL ||
        frame == NULL
    )
    {
        return CAN_BACKEND_ERROR;
    }

    if (
        fake->rx_head ==
        fake->rx_tail
    )
    {
        return CAN_BACKEND_WOULD_BLOCK;
    }

    *frame =
        fake->rx[
            fake->rx_head++
        ];

    return CAN_BACKEND_OK;
}

static void fake_close(
    void *context
)
{
    (void)context;
}

static void fake_push_rx(
    FakeCan *fake,
    const CanFrame *frame
)
{
    if (
        fake != NULL &&
        frame != NULL &&
        fake->rx_tail < FAKE_CAPACITY
    )
    {
        fake->rx[fake->rx_tail++] =
            *frame;
    }
}

static void fake_clear_tx(
    FakeCan *fake
)
{
    fake->tx_count =
        0U;
}

static void push_operational_nodes(
    FakeCan *fake
)
{
    for (uint8_t node = 1U; node <= 6U; ++node)
    {
        CanFrame tpdo;

        (void)avatar_m_build_tpdo4(
            node,
            (int32_t)(1000 * node),
            0x0027U,
            &tpdo
        );

        fake_push_rx(
            fake,
            &tpdo
        );

        CanFrame heartbeat =
        {
            .id = (uint16_t)(0x700U + node),
            .dlc = 1U,
            .data = {0x05U}
        };

        fake_push_rx(
            fake,
            &heartbeat
        );
    }
}

int main(void)
{
    FakeCan fake =
        {0};

    CanBackend backend =
    {
        .context = &fake,
        .send = fake_send,
        .receive = fake_receive,
        .close = fake_close
    };

    CanopenMaster master;

    CanopenMasterConfig config =
    {
        .backend = &backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = 6U,
        .heartbeat_timeout_ms = 20U,
        .sdo_timeout_ms = 10U
    };

    CHECK(
        canopen_master_init(
            &master,
            &config
        ),
        "Initialize six-node CANopen master"
    );

    CHECK(
        canopen_master_send_nmt_all(
            &master,
            CANOPEN_NMT_START
        ) &&
        fake.tx_count == 1U &&
        fake.tx[0].id == 0x000U &&
        fake.tx[0].dlc == 2U &&
        fake.tx[0].data[0] == 0x01U &&
        fake.tx[0].data[1] == 0x00U,
        "NMT Start is broadcast once to all nodes"
    );

    fake_clear_tx(
        &fake
    );

    const int32_t targets[6] =
    {
        1000,
        2000,
        3000,
        4000,
        5000,
        6000
    };

    CHECK(
        canopen_master_send_target_cycle(
            &master,
            targets,
            6U
        ) &&
        fake.tx_count == 7U,
        "Six RPDO4 targets plus one SYNC form one command cycle"
    );

    bool cycle_ids_ok =
        fake.tx_count == 7U;

    for (uint8_t i = 0U; i < 6U && cycle_ids_ok; ++i)
    {
        cycle_ids_ok =
            fake.tx[i].id ==
                (uint16_t)(0x501U + i) &&
            fake.tx[i].dlc == 4U;
    }

    cycle_ids_ok =
        cycle_ids_ok &&
        fake.tx[6].id == 0x080U &&
        fake.tx[6].dlc == 0U;

    CHECK(
        cycle_ids_ok,
        "Command cycle uses RPDO4 0x501..0x506 then SYNC 0x080"
    );

    push_operational_nodes(
        &fake
    );

    CHECK(
        canopen_master_poll(
            &master,
            100U
        ),
        "Poll accepts six TPDO4 frames and six heartbeats"
    );

    CHECK(
        canopen_master_all_feedback_valid(
            &master
        ),
        "TPDO4 feedback becomes valid for all six axes"
    );

    CHECK(
        canopen_master_all_drives_operation_enabled(
            &master
        ),
        "CiA-402 statusword reports all six drives Operation Enabled"
    );

    CHECK(
        canopen_master_healthy(
            &master,
            110U
        ),
        "Fresh OPERATIONAL heartbeats mark communication healthy"
    );

    CHECK(
        !canopen_master_healthy(
            &master,
            121U
        ),
        "Heartbeat timeout marks communication unhealthy"
    );

    fake_clear_tx(
        &fake
    );

    CHECK(
        canopen_master_begin_set_interpolation_mode(
            &master,
            0U,
            200U
        ) &&
        fake.tx_count == 1U &&
        fake.tx[0].id == 0x601U &&
        fake.tx[0].data[0] == 0x2FU &&
        fake.tx[0].data[1] == 0x60U &&
        fake.tx[0].data[2] == 0x60U &&
        fake.tx[0].data[3] == 0x00U &&
        fake.tx[0].data[4] == 0x07U,
        "Boot SDO writes AVATAR interpolation mode 0x6060 = 7"
    );

    CanFrame sdo_ok =
    {
        .id = 0x581U,
        .dlc = 8U,
        .data =
        {
            0x60U,
            0x60U,
            0x60U,
            0x00U,
            0x00U,
            0x00U,
            0x00U,
            0x00U
        }
    };

    fake_push_rx(
        &fake,
        &sdo_ok
    );

    CHECK(
        canopen_master_poll(
            &master,
            201U
        ) &&
        canopen_master_sdo_state(
            &master
        ) ==
            CANOPEN_MASTER_SDO_COMPLETE,
        "Matching SDO acknowledgement completes nonblocking transaction"
    );

    canopen_master_sdo_clear(
        &master
    );

    CHECK(
        canopen_master_begin_read_work_mode(
            &master,
            1U,
            300U
        ),
        "Start readback transaction for node 2"
    );

    CHECK(
        canopen_master_poll(
            &master,
            311U
        ) &&
        canopen_master_sdo_state(
            &master
        ) ==
            CANOPEN_MASTER_SDO_TIMEOUT,
        "Unanswered SDO transaction times out"
    );

    AvatarMPositionScale scale;

    CHECK(
        avatar_m_position_scale_default(
            &scale,
            51.0
        ),
        "Create AVATAR joint scale for 51:1 reducer"
    );

    int32_t units = 0;

    const double one_motor_turn_at_output =
        2.0 * 3.14159265358979323846 /
        51.0;

    CHECK(
        avatar_m_joint_rad_to_position_units(
            &scale,
            one_motor_turn_at_output,
            &units
        ) &&
        units == 32768,
        "One motor revolution maps to 32768 AVATAR position units"
    );

    double represented_rad = 0.0;

    CHECK(
        avatar_m_position_units_to_joint_rad(
            &scale,
            units,
            &represented_rad
        ) &&
        fabs(
            represented_rad -
            one_motor_turn_at_output
        ) < 1e-12,
        "AVATAR position conversion round-trips joint radians"
    );

    printf(
        "\nTests failed: %d\n",
        failures
    );

    return
        failures != 0;
}
