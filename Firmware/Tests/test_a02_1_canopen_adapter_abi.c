/* A02.1 -- intentionally failing integration regression on d8924d3.
 * This TU sees the production CanopenMaster. Both actual adapters are built
 * separately and retain their relative includes of the root declaration.
 * Do not replace either adapter with a mock and do not link root master.c.
 */
#include "Firmware/STM32H745/CM7/Application/CANComm/CANopen/canopen_master.h"
#include "ServoDrive/JointDrive/canopen_drive_commissioning_port.h"
#include "ServoDrive/JointDrive/canopen_joint_drive_port.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
#define EXPECT(pred) do { \
    if (!(pred)) { \
        fprintf(stderr, "FAIL A02.1:%d: %s\n", __LINE__, #pred); \
        ++failures; \
    } \
} while (0)

typedef struct {
    unsigned transmissions;
} FakeCan;

static CanBackendResult fake_send(void *context, const CanFrame *frame)
{
    FakeCan *bus = (FakeCan *)context;
    (void)frame;
    ++bus->transmissions;
    return CAN_BACKEND_OK;
}
static CanBackendResult fake_receive(void *context, CanFrame *frame)
{
    (void)context;
    (void)frame;
    return CAN_BACKEND_WOULD_BLOCK;
}
static void fake_close(void *context) { (void)context; }
static bool owns_runtime(void *context) { return *(const bool *)context; }

int main(void)
{
    FakeCan bus = {0};
    CanBackend backend = {
        .context = &bus,
        .send = fake_send,
        .receive = fake_receive,
        .close = fake_close,
    };
    CanopenMasterConfig config = {
        .backend = &backend,
        .node_count = JOINT_DRIVE_AXES,
        .heartbeat_timeout_ms = 300U,
        .sdo_timeout_ms = 100U,
    };
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        config.node_ids[i] = (uint8_t)(i + 1U);

    CanopenMaster master;
    memset(&master, 0, sizeof(master));
    EXPECT(canopen_master_init(&master, &config));
    EXPECT(master.initialized);
    EXPECT(!master.drive_command_permission);

    /* These are real production-linked adapter functions, not mocks. */
    const DriveCommissioningPort commissioning =
        canopen_drive_commissioning_port_make(&master);
    const JointDrivePort drive = canopen_joint_drive_port_make(&master);
    EXPECT(commissioning.is_initialized != NULL);
    EXPECT(commissioning.is_configured != NULL);
    EXPECT(drive.is_configured != NULL);

    const bool init_when_denied =
        commissioning.is_initialized(commissioning.context);
    const bool configured_when_denied =
        drive.is_configured(drive.context);
    const bool commissioning_configured_when_denied =
        commissioning.is_configured(commissioning.context);
    printf("A02.1 denied: production_initialized=%d permission=%d "
           "commissioning_initialized=%d commissioning_configured=%d "
           "joint_configured=%d\n",
           (int)master.initialized, (int)master.drive_command_permission,
           (int)init_when_denied, (int)commissioning_configured_when_denied,
           (int)configured_when_denied);
    fflush(stdout);
    EXPECT(init_when_denied);                   /* expected FAIL on baseline */
    EXPECT(configured_when_denied);             /* expected FAIL on baseline */
    EXPECT(commissioning_configured_when_denied); /* expected FAIL on baseline */

    int32_t targets[JOINT_DRIVE_AXES] = {0};
    EXPECT(!canopen_master_begin_write_controlword(&master, 0U, 0x000FU, 1U));
    EXPECT(!canopen_master_send_target_cycle(&master, targets,
                                              JOINT_DRIVE_AXES));
    EXPECT(!canopen_master_send_sync(&master));
    EXPECT(bus.transmissions == 0U);

    bool owner_valid = true;
    EXPECT(canopen_master_bind_runtime_owner(&master,
                                              owns_runtime, &owner_valid));
    canopen_master_set_drive_permission(&master, true);
    EXPECT(master.drive_command_permission);
    const bool init_when_granted =
        commissioning.is_initialized(commissioning.context);
    const bool configured_when_granted = drive.is_configured(drive.context);
    printf("A02.1 granted: commissioning_initialized=%d joint_configured=%d\n",
           (int)init_when_granted, (int)configured_when_granted);
    fflush(stdout);
    EXPECT(init_when_granted);
    EXPECT(configured_when_granted);

    canopen_master_set_drive_permission(&master, false);
    EXPECT(!master.drive_command_permission);
    const bool init_after_revoke =
        commissioning.is_initialized(commissioning.context);
    const bool configured_after_revoke = drive.is_configured(drive.context);
    printf("A02.1 revoked: commissioning_initialized=%d joint_configured=%d\n",
           (int)init_after_revoke, (int)configured_after_revoke);
    fflush(stdout);
    EXPECT(init_after_revoke);                  /* expected FAIL on baseline */
    EXPECT(configured_after_revoke);            /* expected FAIL on baseline */
    EXPECT(init_when_denied == init_when_granted);
    EXPECT(configured_when_denied == configured_when_granted);
    EXPECT(bus.transmissions == 0U);

    canopen_master_close(&master);
    if (failures != 0U) {
        fprintf(stderr, "A02.1: %u violated assertions (expected baseline defect).\n",
                failures);
        return 1;
    }
    puts("PASS A02.1: production master and real adapters agree");
    return 0;
}
