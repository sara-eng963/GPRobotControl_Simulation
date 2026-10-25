#ifndef CANOPEN_OBJECTS_H
#define CANOPEN_OBJECTS_H

#include <stdint.h>

/*
 * Generic CANopen communication-profile object dictionary entries.
 *
 * Keep device/drive-specific objects out of this file.
 */
#define CANOPEN_OD_DEVICE_TYPE                  0x1000U
#define CANOPEN_OD_SYNC_COB_ID                  0x1005U
#define CANOPEN_OD_HEARTBEAT_CONSUMER_TIME      0x1016U
#define CANOPEN_OD_HEARTBEAT_PRODUCER_TIME      0x1017U
#define CANOPEN_OD_IDENTITY                     0x1018U

#define CANOPEN_SUBINDEX_0                      0x00U

#endif /* CANOPEN_OBJECTS_H */
