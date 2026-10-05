#ifndef CIA402_OBJECTS_H
#define CIA402_OBJECTS_H

#include <stdint.h>

/*
 * Generic CiA-402 drive-profile object dictionary entries.
 *
 * These constants are transport-independent and device-independent.
 * EtherCAT-, CANopen-, A6-EC-, and AVATAR-specific behavior belongs elsewhere.
 */
#define CIA402_OD_ERROR_CODE                    0x603FU
#define CIA402_OD_CONTROLWORD                   0x6040U
#define CIA402_OD_STATUSWORD                    0x6041U
#define CIA402_OD_POSITION_ACTUAL_INTERNAL      0x6063U
#define CIA402_OD_POSITION_ACTUAL_VALUE         0x6064U
#define CIA402_OD_MODES_OF_OPERATION            0x6060U
#define CIA402_OD_MODES_OF_OPERATION_DISPLAY    0x6061U
#define CIA402_OD_VELOCITY_ACTUAL_VALUE         0x606CU
#define CIA402_OD_CURRENT_ACTUAL_VALUE          0x6078U
#define CIA402_OD_TARGET_POSITION               0x607AU
#define CIA402_OD_PROFILE_VELOCITY              0x6081U
#define CIA402_OD_PROFILE_ACCELERATION          0x6083U
#define CIA402_OD_HOMING_METHOD                 0x6098U

#define CIA402_SUBINDEX_0                       0x00U

/* CiA-402 Modes of Operation used by the project/manuals. */
#define CIA402_MODE_PROFILE_POSITION             ((int8_t)1)
#define CIA402_MODE_PROFILE_VELOCITY             ((int8_t)3)
#define CIA402_MODE_HOMING                       ((int8_t)6)
#define CIA402_MODE_INTERPOLATED_POSITION        ((int8_t)7)
#define CIA402_MODE_CSP                          ((int8_t)8)

#endif /* CIA402_OBJECTS_H */
