#ifndef A6EC_REGISTERS_H
#define A6EC_REGISTERS_H

#include "../CiA402/cia402_objects.h"

#include <stdint.h>

/*
 * A6-EC object-dictionary / drive constants.
 *
 * Standard CiA-402 object numbers are defined once in ServoDrive/CiA402 and
 * aliased here so existing A6-EC code keeps its device-specific names.
 */
#define A6EC_OD_ERROR_CODE                  CIA402_OD_ERROR_CODE
#define A6EC_OD_CONTROLWORD                 CIA402_OD_CONTROLWORD
#define A6EC_OD_STATUSWORD                  CIA402_OD_STATUSWORD
#define A6EC_OD_MODES_OF_OPERATION          CIA402_OD_MODES_OF_OPERATION
#define A6EC_OD_MODES_OF_OPERATION_DISPLAY  CIA402_OD_MODES_OF_OPERATION_DISPLAY
#define A6EC_OD_POSITION_ACTUAL_VALUE       CIA402_OD_POSITION_ACTUAL_VALUE
#define A6EC_OD_TARGET_POSITION             CIA402_OD_TARGET_POSITION

#define A6EC_SUBINDEX_0                     CIA402_SUBINDEX_0
#define A6EC_MODE_CSP                       CIA402_MODE_CSP

/*
 * Current position-reference conversion used by the KickCAT test.
 *
 * A6-EC encoder:
 *      17 bit
 *      131072 position units / revolution
 *
 * Current simulation assumption:
 *      electronic gear = 1:1
 */
#define A6EC_POSITION_UNITS_PER_REV          131072.0

#endif /* A6EC_REGISTERS_H */
