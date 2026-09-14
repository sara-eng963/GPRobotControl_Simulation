#ifndef A6EC_PDO_H
#define A6EC_PDO_H

#include <stdint.h>

/*
 * ============================================================================
 *  A6-EC PDO LAYOUT
 * ============================================================================
 *
 * IMPORTANT:
 *
 * These byte offsets are EXACTLY the offsets used by the already tested
 * KickCAT A6-EC simulator setup.
 *
 * They have NOT been changed or generalized.
 *
 * Current simulator mapping:
 *
 *      Master -> drive
 *          outputs + 0 : Controlword
 *          outputs + 2 : Target Position
 *
 *      Drive -> master
 *          inputs  + 2 : Statusword
 *          inputs  + 4 : Position Actual Value
 *
 * These offsets must still be verified against the real A6-EC ESI / PDO
 * configuration before real-hardware deployment.
 * ============================================================================
 */

#define A6EC_RPDO_CONTROLWORD_OFFSET          0U
#define A6EC_RPDO_TARGET_POSITION_OFFSET      2U

#define A6EC_TPDO_STATUSWORD_OFFSET            2U
#define A6EC_TPDO_ACTUAL_POSITION_OFFSET       4U


typedef struct
{
    uint16_t controlword;
    int32_t targetPosition;

} A6ECPDOCommand;


typedef struct
{
    uint16_t statusword;
    int32_t actualPosition;

} A6ECPDOFeedback;


/*
 * Read the same two input-PDO values that main.c already used.
 */
void a6ec_pdo_read_feedback(
    const uint8_t *inputs,
    A6ECPDOFeedback *feedback
);


/*
 * Read back the current output-PDO command image.
 *
 * This is used only for diagnostics/printing, matching the old main.c code
 * that directly read outputs + 0 and outputs + 2.
 */
void a6ec_pdo_read_command(
    const uint8_t *outputs,
    A6ECPDOCommand *command
);


/*
 * Write the same two output-PDO values that main.c already used.
 */
void a6ec_pdo_write_command(
    uint8_t *outputs,
    const A6ECPDOCommand *command
);


#endif /* A6EC_PDO_H */
