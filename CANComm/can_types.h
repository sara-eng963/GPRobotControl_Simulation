#ifndef CAN_TYPES_H
#define CAN_TYPES_H

#include <stdint.h>

#define CAN_CLASSIC_MAX_DATA_BYTES 8U
#define CAN_STANDARD_ID_MAX        0x7FFU

typedef struct
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_CLASSIC_MAX_DATA_BYTES];

} CanFrame;

#endif /* CAN_TYPES_H */