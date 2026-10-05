#ifndef AVATAR_M_PDO_H
#define AVATAR_M_PDO_H

#include "../../CANComm/can_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t actual_position;
    uint16_t statusword;

} AvatarMFeedback;

bool avatar_m_build_rpdo4(
    uint8_t node_id,
    int32_t target_position,
    CanFrame *frame
);

bool avatar_m_parse_rpdo4(
    uint8_t node_id,
    const CanFrame *frame,
    int32_t *target_position
);

bool avatar_m_build_tpdo4(
    uint8_t node_id,
    int32_t actual_position,
    uint16_t statusword,
    CanFrame *frame
);

bool avatar_m_parse_tpdo4(
    uint8_t node_id,
    const CanFrame *frame,
    AvatarMFeedback *feedback
);

void avatar_m_build_sync(
    CanFrame *frame
);

#ifdef __cplusplus
}
#endif

#endif /* AVATAR_M_PDO_H */
