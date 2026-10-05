#ifndef CANOPEN_IDS_H
#define CANOPEN_IDS_H

#include <stdbool.h>
#include <stdint.h>

#define CANOPEN_NODE_ID_MIN 1U
#define CANOPEN_NODE_ID_MAX 127U

/* Those IDs come directly from the M-series default CANopen mapping. */

#define CANOPEN_COBID_NMT       0x000U
#define CANOPEN_COBID_SYNC      0x080U

#define CANOPEN_TPDO1_BASE      0x180U
#define CANOPEN_RPDO1_BASE      0x200U

#define CANOPEN_TPDO2_BASE      0x280U
#define CANOPEN_RPDO2_BASE      0x300U

#define CANOPEN_TPDO3_BASE      0x380U
#define CANOPEN_RPDO3_BASE      0x400U

#define CANOPEN_TPDO4_BASE      0x480U
#define CANOPEN_RPDO4_BASE      0x500U

#define CANOPEN_SDO_TX_BASE     0x580U
#define CANOPEN_SDO_RX_BASE     0x600U

#define CANOPEN_HEARTBEAT_BASE  0x700U

static inline bool canopen_node_id_valid(uint8_t node_id)
{
    return node_id >= CANOPEN_NODE_ID_MIN &&
           node_id <= CANOPEN_NODE_ID_MAX;
}

static inline uint16_t canopen_rpdo4_id(uint8_t node_id)
{
    return (uint16_t)(CANOPEN_RPDO4_BASE + node_id);
}

static inline uint16_t canopen_tpdo4_id(uint8_t node_id)
{
    return (uint16_t)(CANOPEN_TPDO4_BASE + node_id);
}

static inline uint16_t canopen_sdo_rx_id(uint8_t node_id)
{
    return (uint16_t)(CANOPEN_SDO_RX_BASE + node_id);
}

static inline uint16_t canopen_sdo_tx_id(uint8_t node_id)
{
    return (uint16_t)(CANOPEN_SDO_TX_BASE + node_id);
}

static inline uint16_t canopen_heartbeat_id(uint8_t node_id)
{
    return (uint16_t)(CANOPEN_HEARTBEAT_BASE + node_id);
}

#endif /* CANOPEN_IDS_H */