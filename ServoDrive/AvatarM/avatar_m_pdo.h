#ifndef AVATAR_M_PDO_H
#define AVATAR_M_PDO_H

#include "../../CANComm/can_types.h"

#include <stdbool.h>
#include <stdint.h>

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

bool avatar_m_parse_tpdo4(
    uint8_t node_id,
    const CanFrame *frame,
    AvatarMFeedback *feedback
);

void avatar_m_build_sync(
    CanFrame *frame
);

#endif /* AVATAR_M_PDO_H */