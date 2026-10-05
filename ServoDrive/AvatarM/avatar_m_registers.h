#ifndef AVATAR_M_REGISTERS_H
#define AVATAR_M_REGISTERS_H

#include "../CiA402/cia402_objects.h"
#include "../../CANComm/CANopen/canopen_objects.h"

#include <stdint.h>

/*
 * AVATAR M-Series object-dictionary and vendor constants.
 *
 * Standard CANopen communication-profile objects remain defined in
 * CANComm/CANopen/canopen_objects.h.
 * Standard CiA-402 drive-profile objects remain defined in
 * ServoDrive/CiA402/cia402_objects.h.
 *
 * The aliases below make AVATAR drive code readable without duplicating the
 * authoritative numeric definitions.
 */

/* Standard CANopen communication-profile objects used by AVATAR M. */
#define AVATAR_M_OD_DEVICE_TYPE                 CANOPEN_OD_DEVICE_TYPE
#define AVATAR_M_OD_SYNC_COB_ID                 CANOPEN_OD_SYNC_COB_ID
#define AVATAR_M_OD_HEARTBEAT_CONSUMER_TIME     CANOPEN_OD_HEARTBEAT_CONSUMER_TIME
#define AVATAR_M_OD_HEARTBEAT_PRODUCER_TIME     CANOPEN_OD_HEARTBEAT_PRODUCER_TIME
#define AVATAR_M_OD_IDENTITY                    CANOPEN_OD_IDENTITY

/* Standard CiA-402 objects used by AVATAR M. */
#define AVATAR_M_OD_CONTROLWORD                 CIA402_OD_CONTROLWORD
#define AVATAR_M_OD_STATUSWORD                  CIA402_OD_STATUSWORD
#define AVATAR_M_OD_MODES_OF_OPERATION          CIA402_OD_MODES_OF_OPERATION
#define AVATAR_M_OD_MODES_OF_OPERATION_DISPLAY  CIA402_OD_MODES_OF_OPERATION_DISPLAY
#define AVATAR_M_OD_POSITION_ACTUAL_INTERNAL    CIA402_OD_POSITION_ACTUAL_INTERNAL
#define AVATAR_M_OD_POSITION_ACTUAL_VALUE       CIA402_OD_POSITION_ACTUAL_VALUE
#define AVATAR_M_OD_VELOCITY_ACTUAL_VALUE       CIA402_OD_VELOCITY_ACTUAL_VALUE
#define AVATAR_M_OD_CURRENT_ACTUAL_VALUE        CIA402_OD_CURRENT_ACTUAL_VALUE
#define AVATAR_M_OD_TARGET_POSITION             CIA402_OD_TARGET_POSITION
#define AVATAR_M_OD_PROFILE_VELOCITY            CIA402_OD_PROFILE_VELOCITY
#define AVATAR_M_OD_PROFILE_ACCELERATION        CIA402_OD_PROFILE_ACCELERATION
#define AVATAR_M_OD_HOMING_METHOD               CIA402_OD_HOMING_METHOD

#define AVATAR_M_SUBINDEX_0                     CIA402_SUBINDEX_0

/* Modes explicitly documented by the AVATAR M manual. */
#define AVATAR_M_MODE_POSITION                  CIA402_MODE_PROFILE_POSITION
#define AVATAR_M_MODE_SPEED                     CIA402_MODE_PROFILE_VELOCITY
#define AVATAR_M_MODE_HOMING                    CIA402_MODE_HOMING
#define AVATAR_M_MODE_INTERPOLATION             CIA402_MODE_INTERPOLATED_POSITION

/* AVATAR vendor-specific objects. */
#define AVATAR_M_OD_GEAR_NUMERATOR              0x260AU
#define AVATAR_M_OD_GEAR_DENOMINATOR            0x260BU
#define AVATAR_M_OD_INCREMENTAL_POSITION        0x260CU
#define AVATAR_M_OD_ALARM                       0x260EU
#define AVATAR_M_OD_SPECIAL_FUNCTION_MODE       0x2619U
#define AVATAR_M_OD_SYNC_CONTROL                0x261CU

/* AVATAR manual default for object 0x1017. */
#define AVATAR_M_DEFAULT_HEARTBEAT_PRODUCER_MS  1000U

/* Vendor heartbeat meaning documented by AVATAR M. */
#define AVATAR_M_HEARTBEAT_ALARM                0x04U

#endif /* AVATAR_M_REGISTERS_H */
