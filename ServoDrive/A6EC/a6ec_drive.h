#ifndef A6EC_DRIVE_H
#define A6EC_DRIVE_H

#include "a6ec_pdo.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * ============================================================================
 *  A6-EC DRIVE INTERFACE
 * ============================================================================
 *
 * This module groups the A6-EC-specific operations that were previously spread
 * through main.c:
 *
 *      - set 0x6060 = 8 for CSP
 *      - read back 0x6060
 *      - read/write the current simulator PDO mapping
 *      - convert radians <-> A6 position reference units
 *
 * CiA-402 state-machine decisions remain in ServoDrive/CiA402/.
 * EtherCAT/SOEM transport remains in EtherCATComm/.
 * ============================================================================
 */


/*
 * Configure one A6-EC drive exactly as main.c did:
 *
 *      SDO write 0x6060:00 = 8
 *      SDO read  0x6060:00
 *
 * Returns true only when both operations succeed.
 */
bool a6ec_set_csp_mode(
    int slave,
    int8_t *mode_readback
);

/*
 * Read the CiA-402 Error Code object:
 *
 *      0x603F:00
 *
 * Returns true when the SDO read succeeds.
 */
bool a6ec_read_error_code(
    int slave,
    uint16_t *error_code
);


/*
 * Convenience accessors over the existing A6-EC simulator PDO mapping.
 */
void a6ec_read_feedback(
    int slave,
    A6ECPDOFeedback *feedback
);


void a6ec_read_command(
    int slave,
    A6ECPDOCommand *command
);


void a6ec_write_command(
    int slave,
    const A6ECPDOCommand *command
);


/*
 * Exact position conversions extracted from main.c.
 *
 * Current assumption:
 *      131072 units / rev
 *      1:1 electronic gearing
 *
 * Real gearbox ratio, joint sign and zero offset are deliberately NOT added in
 * this modularization step.
 */
int32_t a6ec_joint_rad_to_position_units(
    double q_rad
);


double a6ec_position_units_to_joint_rad(
    int32_t drive_units
);


#endif /* A6EC_DRIVE_H */
