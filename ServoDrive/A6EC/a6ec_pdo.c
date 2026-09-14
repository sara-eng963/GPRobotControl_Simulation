#include "a6ec_pdo.h"

#include "../../EtherCATComm/ethercat_master.h"

#include <stddef.h>


void a6ec_pdo_read_feedback(
    const uint8_t *inputs,
    A6ECPDOFeedback *feedback
)
{
    if (
        inputs == NULL ||
        feedback == NULL
    )
    {
        return;
    }


    feedback->statusword =
        ethercat_pdo_read_u16(
            inputs +
            A6EC_TPDO_STATUSWORD_OFFSET
        );


    feedback->actualPosition =
        ethercat_pdo_read_i32(
            inputs +
            A6EC_TPDO_ACTUAL_POSITION_OFFSET
        );
}


void a6ec_pdo_read_command(
    const uint8_t *outputs,
    A6ECPDOCommand *command
)
{
    if (
        outputs == NULL ||
        command == NULL
    )
    {
        return;
    }


    command->controlword =
        ethercat_pdo_read_u16(
            outputs +
            A6EC_RPDO_CONTROLWORD_OFFSET
        );


    command->targetPosition =
        ethercat_pdo_read_i32(
            outputs +
            A6EC_RPDO_TARGET_POSITION_OFFSET
        );
}


void a6ec_pdo_write_command(
    uint8_t *outputs,
    const A6ECPDOCommand *command
)
{
    if (
        outputs == NULL ||
        command == NULL
    )
    {
        return;
    }


    ethercat_pdo_write_u16(
        outputs +
        A6EC_RPDO_CONTROLWORD_OFFSET,
        command->controlword
    );


    ethercat_pdo_write_i32(
        outputs +
        A6EC_RPDO_TARGET_POSITION_OFFSET,
        command->targetPosition
    );
}
