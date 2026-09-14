#ifndef A6EC_REGISTERS_H
#define A6EC_REGISTERS_H

#include <stdint.h>

/*
 * ============================================================================
 *  A6-EC OBJECT-DICTIONARY / DRIVE CONSTANTS
 * ============================================================================
 *
 * This file contains the A6-EC / CiA-402 object constants currently used or
 * explicitly referenced by the tested controller.
 *
 * No new register behavior is introduced here.
 * ============================================================================
 */


/* CiA-402 drive objects. */
#define A6EC_OD_CONTROLWORD                 0x6040U
#define A6EC_OD_STATUSWORD                  0x6041U

#define A6EC_OD_MODES_OF_OPERATION          0x6060U
#define A6EC_OD_MODES_OF_OPERATION_DISPLAY  0x6061U

#define A6EC_OD_POSITION_ACTUAL_VALUE       0x6064U
#define A6EC_OD_TARGET_POSITION             0x607AU


/* Objects above use subindex 0 in the current controller. */
#define A6EC_SUBINDEX_0                     0x00U


/*
 * CiA-402 Modes of Operation:
 *
 *      8 = Cyclic Synchronous Position
 *
 * Same value previously defined in main.c as CSP_MODE.
 */
#define A6EC_MODE_CSP                       ((int8_t)8)


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
