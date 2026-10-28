/* Compile twice, once against each real header, in separate translation units. */
#if defined(A02_PROBE_FIRMWARE) && !defined(A02_PROBE_ROOT)
#include "Firmware/STM32H745/CM7/Application/CANComm/CANopen/canopen_master.h"
#define HEADER_VARIANT "firmware"
#elif defined(A02_PROBE_ROOT) && !defined(A02_PROBE_FIRMWARE)
#include "CANComm/CANopen/canopen_master.h"
#define HEADER_VARIANT "root"
#else
#error "Define exactly one of A02_PROBE_FIRMWARE or A02_PROBE_ROOT"
#endif
#include <stddef.h>
#include <stdio.h>
int main(void)
{
    printf("LAYOUT variant=%s sizeof=%zu align=%zu sdo=%zu initialized=%zu "
           "owner_check=%zu\n", HEADER_VARIANT, sizeof(CanopenMaster),
           _Alignof(CanopenMaster), offsetof(CanopenMaster, sdo),
           offsetof(CanopenMaster, initialized),
           offsetof(CanopenMaster, owner_check));
#if defined(A02_PROBE_FIRMWARE)
    printf("LAYOUT variant=firmware permission=%zu\n",
           offsetof(CanopenMaster, drive_command_permission));
#endif
    return 0;
}
