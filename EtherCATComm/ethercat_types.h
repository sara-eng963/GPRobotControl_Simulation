#ifndef ETHERCAT_TYPES_H
#define ETHERCAT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * ============================================================================
 *  GENERIC ETHERCAT TYPES
 * ============================================================================
 *
 * These types contain only the bus-level information that already exists in
 * the tested main.c.
 *
 * No CiA-402 logic.
 * No A6-EC register knowledge.
 * No trajectory/control logic.
 * ============================================================================
 */

typedef struct
{
    const char *interfaceName;
    int expectedSlaveCount;
    uint32_t cycleTimeNs;

} EtherCATMasterConfig;

typedef struct
{
    uint32_t vendorId;
    uint32_t productCode;
    uint32_t revision;
    uint32_t serialNumber;

} EtherCATSlaveIdentity;

typedef struct
{
    int slave;

    /*
     * Raw EtherCAT Application Layer state.
     */
    uint16_t state;

    /*
     * EtherCAT AL Status Code reported by the slave.
     *
     * Example:
     *      0x0000 = No error
     */
    uint16_t alStatusCode;

    /*
     * SOEM lost-slave flag.
     */
    bool lost;

} EtherCATSlaveStatus;

typedef enum
{
    ETHERCAT_RECOVERY_ACTION_NONE = 0,

    ETHERCAT_RECOVERY_ACTION_ACK_ERROR,

    ETHERCAT_RECOVERY_ACTION_REQUEST_OPERATIONAL,

    ETHERCAT_RECOVERY_ACTION_RECONFIGURE,

    ETHERCAT_RECOVERY_ACTION_RECOVER_LOST,

    ETHERCAT_RECOVERY_ACTION_FAILED

} EtherCATRecoveryAction;

typedef struct
{
    int slaveCount;
    int mappedBytes;

    int outputBytes;
    int inputBytes;

    bool dcFound;

    int expectedWkc;

} EtherCATBusInfo;


#endif /* ETHERCAT_TYPES_H */
