/* Stage C2: exercise exact CANopen master with a continuously ready RX backend.
 * Host only; measures operation count, not STM32 WCET or physical bus timing. */
#include "../CANopen/canopen_master.h"
#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum { FAKE_UNLIMITED, FAKE_EMPTY, FAKE_BUS_ERROR } FakeMode;
typedef struct {
    uint32_t calls, sends, good_frames;
    FakeMode mode;
} FakeBus;

static CanBackendResult fake_send(void *ctx, const CanFrame *frame)
{
    FakeBus *bus = ctx;
    if (frame == NULL) return CAN_BACKEND_ERROR;
    ++bus->sends;
    return CAN_BACKEND_OK;
}
static CanBackendResult fake_receive(void *ctx, CanFrame *out)
{
    FakeBus *bus = ctx;
    ++bus->calls;
    if (bus->mode == FAKE_EMPTY) return CAN_BACKEND_WOULD_BLOCK;
    if (bus->mode == FAKE_BUS_ERROR) return CAN_BACKEND_ERROR;
    assert(avatar_m_build_tpdo4(1U, (int32_t)bus->good_frames,
                                0x0027U, out));
    ++bus->good_frames;
    return CAN_BACKEND_OK;
}
static void fake_close(void *ctx) {(void)ctx;}

int main(void)
{
    FakeBus bus = {0};
    CanBackend backend = {
        .context = &bus, .send = fake_send,
        .receive = fake_receive, .close = fake_close
    };
    CanopenMaster master;
    CanopenMasterConfig config = {
        .backend = &backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = 6U,
        .heartbeat_timeout_ms = 1000U,
        .sdo_timeout_ms = 100U
    };
    assert(CANOPEN_MASTER_MAX_RX_PER_POLL == 32U);
    assert(canopen_master_init(&master, &config));
    assert(canopen_master_poll(&master, 0U));
    assert(bus.calls == CANOPEN_MASTER_MAX_RX_PER_POLL);
    assert(master.tpdo_rx_count[0] == CANOPEN_MASTER_MAX_RX_PER_POLL);
    assert(bus.sends == 1U); /* controller heartbeat */
    assert(canopen_master_poll(&master, 2U));
    assert(bus.calls == 2U * CANOPEN_MASTER_MAX_RX_PER_POLL);
    assert(master.tpdo_rx_count[0] == 2U * CANOPEN_MASTER_MAX_RX_PER_POLL);
    puts("[PASS] infinite incoming RX backlog is bounded to 32 frames per poll");

    bus.mode = FAKE_EMPTY;
    assert(canopen_master_poll(&master, 4U));
    assert(bus.calls == 2U * CANOPEN_MASTER_MAX_RX_PER_POLL + 1U);
    bus.mode = FAKE_BUS_ERROR;
    assert(!canopen_master_poll(&master, 6U));
    assert(bus.calls == 2U * CANOPEN_MASTER_MAX_RX_PER_POLL + 2U);
    puts("[PASS] empty RX returns normally; CAN transport error propagates");

    canopen_master_close(&master);
    puts("PASS: Stage C2 bounded CANopen RX host regression");
    return 0;
}
