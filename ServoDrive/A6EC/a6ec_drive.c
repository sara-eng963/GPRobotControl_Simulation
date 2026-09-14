#include "a6ec_drive.h"

#include "a6ec_registers.h"

#include "../../EtherCATComm/ethercat_master.h"

#include <math.h>
#include <stddef.h>


/* ============================================================================
 *  CSP MODE CONFIGURATION
 * ============================================================================
 *
 * Same 0x6060 write/readback behavior used by the tested main.c.
 * ============================================================================
 */

bool a6ec_set_csp_mode(
    int slave,
    int8_t *mode_readback
)
{
    int8_t mode =
        A6EC_MODE_CSP;


    int write_wkc =
        ethercat_master_sdo_write(
            slave,
            A6EC_OD_MODES_OF_OPERATION,
            A6EC_SUBINDEX_0,
            sizeof(mode),
            &mode
        );


    if (
        write_wkc <= 0
    )
    {
        return false;
    }


    int8_t readback =
        0;


    size_t readback_size =
        sizeof(readback);


    int read_wkc =
        ethercat_master_sdo_read(
            slave,
            A6EC_OD_MODES_OF_OPERATION,
            A6EC_SUBINDEX_0,
            &readback_size,
            &readback
        );


    if (
        read_wkc <= 0
    )
    {
        return false;
    }


    if (
        mode_readback != NULL
    )
    {
        *mode_readback =
            readback;
    }


    return true;
}


/* ============================================================================
 *  CYCLIC PDO ACCESS
 * ============================================================================
 */

void a6ec_read_feedback(
    int slave,
    A6ECPDOFeedback *feedback
)
{
    if (
        feedback == NULL
    )
    {
        return;
    }


    const uint8_t *inputs =
        ethercat_master_slave_inputs(
            slave
        );


    a6ec_pdo_read_feedback(
        inputs,
        feedback
    );
}


void a6ec_read_command(
    int slave,
    A6ECPDOCommand *command
)
{
    if (
        command == NULL
    )
    {
        return;
    }


    const uint8_t *outputs =
        ethercat_master_slave_outputs(
            slave
        );


    a6ec_pdo_read_command(
        outputs,
        command
    );
}


void a6ec_write_command(
    int slave,
    const A6ECPDOCommand *command
)
{
    if (
        command == NULL
    )
    {
        return;
    }


    uint8_t *outputs =
        ethercat_master_slave_outputs(
            slave
        );


    a6ec_pdo_write_command(
        outputs,
        command
    );
}


/* ============================================================================
 *  POSITION REFERENCE CONVERSION
 * ============================================================================
 *
 * Direct extraction of:
 *
 *      joint_rad_to_drive_units()
 *      drive_units_to_joint_rad()
 *
 * from the tested main.c.
 * ============================================================================
 */

int32_t a6ec_joint_rad_to_position_units(
    double q_rad
)
{
    const double two_pi =
        6.28318530717958647692;


    return
        (int32_t)llround(
            q_rad *
            A6EC_POSITION_UNITS_PER_REV /
            two_pi
        );
}


double a6ec_position_units_to_joint_rad(
    int32_t drive_units
)
{
    const double two_pi =
        6.28318530717958647692;


    return
        ((double)drive_units) *
        two_pi /
        A6EC_POSITION_UNITS_PER_REV;
}
