/*
 * Stage C2B: black-box fake-backend tests of production STM32
 * Firmware/STM32H745/CM7/Application/CANComm/CANopen/canopen_master.c.
 * Do not replace this source with top-level desktop CANopen implementation.
 * These tests verify the software TX veto, not physical drive safety.
 */
#include "canopen_master.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#cond); \
    exit(1); \
} } while (0)

typedef struct {
    CanFrame sent[128];
    CanFrame incoming[32];
    size_t sent_count, rx_head, rx_tail;
    bool fail_next_send;
} FakeBus;

typedef struct {
    FakeBus bus;
    CanBackend backend;
    CanopenMaster master;
    bool owner_allowed;
} Fixture;

static CanBackendResult fake_send(void *ctx, const CanFrame *frame)
{
    FakeBus *b = (FakeBus *)ctx;
    if (b->fail_next_send) {
        b->fail_next_send = false;
        return CAN_BACKEND_ERROR;
    }
    CHECK(b->sent_count < sizeof(b->sent)/sizeof(b->sent[0]));
    b->sent[b->sent_count++] = *frame;
    return CAN_BACKEND_OK;
}
static CanBackendResult fake_receive(void *ctx, CanFrame *frame)
{
    FakeBus *b = (FakeBus *)ctx;
    if (b->rx_head == b->rx_tail) return CAN_BACKEND_WOULD_BLOCK;
    *frame = b->incoming[b->rx_head++];
    return CAN_BACKEND_OK;
}
static void fake_close(void *ctx) { (void)ctx; }
static bool owner_check(void *ctx) { return *(const bool *)ctx; }
static void enqueue_response(Fixture *f, const CanFrame *frame)
{
    CHECK(f->bus.rx_tail < sizeof(f->bus.incoming)/sizeof(f->bus.incoming[0]));
    f->bus.incoming[f->bus.rx_tail++] = *frame;
}
static void fixture_init(Fixture *f)
{
    memset(f, 0, sizeof(*f));
    f->backend = (CanBackend){ .context = &f->bus, .send = fake_send,
                               .receive = fake_receive, .close = fake_close };
    CanopenMasterConfig cfg = { .backend = &f->backend,
                                .node_count = JOINT_DRIVE_AXES,
                                .heartbeat_timeout_ms = 300U,
                                .sdo_timeout_ms = 100U };
    for (size_t i = 0; i < JOINT_DRIVE_AXES; i++) cfg.node_ids[i] = (uint8_t)(i + 1U);
    CHECK(canopen_master_init(&f->master, &cfg));
    CHECK(f->master.initialized);
    CHECK(!f->master.drive_command_permission);
}
static void bind_owner(Fixture *f)
{
    f->owner_allowed = true;
    CHECK(canopen_master_bind_runtime_owner(&f->master, owner_check,
                                            &f->owner_allowed));
}
static void approved(Fixture *f)
{
    bind_owner(f);
    canopen_master_set_drive_permission(&f->master, true);
    CHECK(f->master.drive_command_permission);
}
static void targets(int32_t out[JOINT_DRIVE_AXES])
{
    for (size_t i = 0; i < JOINT_DRIVE_AXES; i++) out[i] = (int32_t)(1000 + 17 * i);
}
static void test_cold_and_prebind_denial(void)
{
    Fixture f;
    fixture_init(&f);
    int32_t q[JOINT_DRIVE_AXES]; targets(q);
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(!canopen_master_send_target_cycle(&f.master,q,JOINT_DRIVE_AXES));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0,0x0006U,5U));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0,0x0007U,5U));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0,0x000FU,5U));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0,0x0080U,5U));
    CHECK(f.bus.sent_count == 0U);
    /* Vulnerability in original C2B: the unbound master could grant itself permission. */
    canopen_master_set_drive_permission(&f.master, true);
    CHECK(!f.master.drive_command_permission);
    CHECK(!canopen_master_send_sync(&f.master));
    bind_owner(&f);
    CHECK(!f.master.drive_command_permission);
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(f.bus.sent_count == 0U);
}
static void test_approved_transmissions(void)
{
    Fixture f;
    fixture_init(&f);
    approved(&f);
    int32_t q[JOINT_DRIVE_AXES]; targets(q);
    CHECK(canopen_master_send_target_cycle(&f.master,q,JOINT_DRIVE_AXES));
    CHECK(f.bus.sent_count == JOINT_DRIVE_AXES + 1U);
    for (size_t i=0; i<JOINT_DRIVE_AXES; ++i) {
        CHECK(f.bus.sent[i].id != 0x080U);
        CHECK(f.bus.sent[i].dlc <= 8U);
    }
    CHECK(f.bus.sent[JOINT_DRIVE_AXES].id == 0x080U);
    CHECK(f.bus.sent[JOINT_DRIVE_AXES].dlc == 0U);
    CHECK(canopen_master_send_sync(&f.master));
    CHECK(f.bus.sent_count == JOINT_DRIVE_AXES + 2U);
    CHECK(f.bus.sent[f.bus.sent_count-1].id == 0x080U);
    CHECK(canopen_master_begin_write_controlword(&f.master, 0U, 0x000FU, 10U));
    CHECK(canopen_master_sdo_state(&f.master) == CANOPEN_MASTER_SDO_PENDING);
    const CanFrame *w = &f.bus.sent[f.bus.sent_count-1];
    CHECK(w->id == 0x601U); /* SDO Rx for configured node 1 */
    CHECK(w->dlc == 8U);
    CHECK(w->data[0] == 0x2BU && w->data[1] == 0x40U &&
          w->data[2] == 0x60U && w->data[3] == 0x00U);
    CHECK(w->data[4] == 0x0FU && w->data[5] == 0x00U);
}
static void test_revocation_and_stale_sdo_ack(void)
{
    Fixture f;
    fixture_init(&f);
    approved(&f);
    int32_t q[JOINT_DRIVE_AXES]; targets(q);
    CHECK(canopen_master_begin_write_controlword(&f.master, 0U, 0x000FU, 10U));
    size_t sent_before = f.bus.sent_count;
    CHECK(canopen_master_sdo_state(&f.master) == CANOPEN_MASTER_SDO_PENDING);
    canopen_master_set_drive_permission(&f.master, false);
    CHECK(!f.master.drive_command_permission);
    CHECK(canopen_master_sdo_state(&f.master) == CANOPEN_MASTER_SDO_TRANSPORT_ERROR);
    CHECK(canopen_master_sdo_response(&f.master) == NULL);
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(!canopen_master_send_target_cycle(&f.master,q,JOINT_DRIVE_AXES));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0U,0x000FU,20U));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0U,0x0080U,20U));
    CHECK(f.bus.sent_count == sent_before);
    /* An expedited write-OK received after revocation must NOT complete the old SDO. */
    const CanFrame ack = { .id = 0x581U, .dlc = 8U,
                           .data = {0x60U,0x40U,0x60U,0x00U,0U,0U,0U,0U} };
    enqueue_response(&f,&ack);
    CHECK(canopen_master_poll(&f.master,20U));
    CHECK(canopen_master_sdo_state(&f.master) == CANOPEN_MASTER_SDO_TRANSPORT_ERROR);
    CHECK(canopen_master_sdo_response(&f.master) == NULL);
    /* Explicit voltage-disable/quick-stop requests remain available; not safety-rated. */
    CHECK(canopen_master_begin_write_controlword(&f.master,0U,0x0000U,22U));
    canopen_master_sdo_clear(&f.master);
    CHECK(canopen_master_begin_write_controlword(&f.master,0U,0x0002U,23U));
}
static void test_nmt_reset_revokes_and_cancels(void)
{
    Fixture f;
    fixture_init(&f);
    approved(&f);
    CHECK(canopen_master_begin_write_controlword(&f.master,0U,0x000FU,12U));
    CHECK(canopen_master_send_nmt_all(&f.master,CANOPEN_NMT_RESET_COMMUNICATION));
    CHECK(!f.master.drive_command_permission);
    CHECK(canopen_master_sdo_state(&f.master) == CANOPEN_MASTER_SDO_TRANSPORT_ERROR);
    const CanFrame *reset = &f.bus.sent[f.bus.sent_count-1];
    CHECK(reset->id == 0U && reset->dlc == 2U && reset->data[0] == 0x82U);
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0U,0x000FU,13U));

    Fixture failed;
    fixture_init(&failed);
    approved(&failed);
    failed.bus.fail_next_send = true;
    CHECK(!canopen_master_send_nmt_all(&failed.master,CANOPEN_NMT_RESET_APPLICATION));
    CHECK(!failed.master.drive_command_permission); /* even if reset TX failed */
    CHECK(!canopen_master_send_sync(&failed.master));

    Fixture runtime;
    fixture_init(&runtime);
    approved(&runtime);
    CHECK(canopen_master_begin_write_controlword(&runtime.master,0U,0x000FU,1U));
    canopen_master_clear_runtime(&runtime.master);
    CHECK(!runtime.master.drive_command_permission);
    CHECK(canopen_master_sdo_state(&runtime.master) == CANOPEN_MASTER_SDO_IDLE);
}
static void test_runtime_owner_enforcement(void)
{
    Fixture f;
    fixture_init(&f);
    bind_owner(&f);
    int32_t q[JOINT_DRIVE_AXES]; targets(q);
    f.owner_allowed = false;
    canopen_master_set_drive_permission(&f.master,true);
    CHECK(!f.master.drive_command_permission);
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(!canopen_master_send_target_cycle(&f.master,q,JOINT_DRIVE_AXES));
    CHECK(!canopen_master_begin_write_controlword(&f.master,0U,0x000FU,0U));
    CHECK(!canopen_master_send_nmt_all(&f.master,CANOPEN_NMT_RESET_COMMUNICATION));
    CHECK(f.bus.sent_count == 0U);
    f.owner_allowed = true;
    canopen_master_set_drive_permission(&f.master,true);
    CHECK(f.master.drive_command_permission);
    f.owner_allowed = false;
    canopen_master_set_drive_permission(&f.master,false);
    CHECK(f.master.drive_command_permission); /* foreign caller cannot change the gate */
    CHECK(!canopen_master_send_sync(&f.master));
    CHECK(!canopen_master_send_nmt_all(&f.master,CANOPEN_NMT_RESET_COMMUNICATION));
    CHECK(f.bus.sent_count == 0U);
    f.owner_allowed = true;
    CHECK(canopen_master_send_sync(&f.master));
    canopen_master_set_drive_permission(&f.master,false);
    CHECK(!f.master.drive_command_permission);
}
int main(void)
{
    test_cold_and_prebind_denial();
    test_approved_transmissions();
    test_revocation_and_stale_sdo_ack();
    test_nmt_reset_revokes_and_cancels();
    test_runtime_owner_enforcement();
    puts("PASS: production STM32 CANopen master authorization, revocation, NMT, SDO and owner checks");
    return 0;
}
