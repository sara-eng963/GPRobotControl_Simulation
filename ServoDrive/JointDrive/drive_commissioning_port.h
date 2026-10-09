#ifndef DRIVE_COMMISSIONING_PORT_H
#define DRIVE_COMMISSIONING_PORT_H

#include "joint_drive_port.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Nonblocking commissioning interface for the robot's six drive axes.
 *
 * Unlike JointDrivePort, this interface owns drive configuration:
 * node identity, network transitions, heartbeat producer/consumer settings,
 * operating mode selection, and CiA-402 status/controlword SDO transactions.
 * This is a local function-pointer interface, NOT a cross-core IPC ABI.
 *
 * One nonblocking operation may be outstanding at a time. The caller must
 * query completion and clear the result before starting another operation.
 */
typedef enum {
    DRIVE_NETWORK_RESET_COMMUNICATION,
    DRIVE_NETWORK_START
} DriveNetworkCommand;

typedef enum {
    DRIVE_NODE_BOOTUP,
    DRIVE_NODE_OPERATIONAL
} DriveNodeHeartbeatState;

typedef enum {
    DRIVE_COMMISSION_READ_IDENTITY,         /* argument: subindex (1,2) */
    DRIVE_COMMISSION_SET_HEARTBEAT_PERIOD,  /* argument: period ms */
    DRIVE_COMMISSION_SET_HEARTBEAT_CONSUMER,
    DRIVE_COMMISSION_READ_HEARTBEAT_CONSUMER,
    DRIVE_COMMISSION_SET_INTERPOLATION_MODE,
    DRIVE_COMMISSION_READ_MODE_DISPLAY,
    DRIVE_COMMISSION_READ_STATUSWORD,
    DRIVE_COMMISSION_WRITE_CONTROLWORD,    /* argument: 16-bit controlword */
    DRIVE_COMMISSION_READ_ACTUAL_POSITION
} DriveCommissionOperation;

typedef enum {
    DRIVE_OPERATION_IDLE,
    DRIVE_OPERATION_PENDING,
    DRIVE_OPERATION_COMPLETE,
    DRIVE_OPERATION_ABORT,
    DRIVE_OPERATION_TIMEOUT,
    DRIVE_OPERATION_TRANSPORT_ERROR
} DriveOperationStatus;

typedef struct {
    DriveOperationStatus status;
    bool has_response;
    bool is_read;
    bool is_write_ok;
    uint8_t data_size;
    uint32_t value;
    uint32_t abort_code;
} DriveOperationResult;

typedef struct {
    void *context;

    bool (*is_initialized)(void *context);
    bool (*is_configured)(void *context);
    void (*reset_runtime)(void *context, uint32_t heartbeat_timeout_ms);
    bool (*send_network_command)(void *context, DriveNetworkCommand command);
    bool (*all_heartbeat_state)(void *context, DriveNodeHeartbeatState state);

    bool (*begin_operation)(void *context, DriveCommissionOperation operation,
                            size_t axis, uint32_t argument, uint32_t now_ms);
    DriveOperationResult (*operation_result)(void *context);
    void (*clear_operation)(void *context);

    /* Expected values are properties of the installed drive family. */
    uint32_t expected_vendor_id;
    uint32_t expected_product_code;
    uint32_t expected_heartbeat_consumer;
    uint8_t expected_mode_display;
} DriveCommissioningPort;

static inline bool drive_commissioning_port_valid(
    const DriveCommissioningPort *port)
{
    return port != NULL && port->context != NULL &&
           port->is_initialized != NULL &&
           port->is_configured != NULL &&
           port->reset_runtime != NULL &&
           port->send_network_command != NULL &&
           port->all_heartbeat_state != NULL &&
           port->begin_operation != NULL &&
           port->operation_result != NULL &&
           port->clear_operation != NULL;
}

static inline bool drive_commissioning_port_begin(
    const DriveCommissioningPort *port, DriveCommissionOperation operation,
    size_t axis, uint32_t argument, uint32_t now_ms)
{
    return drive_commissioning_port_valid(port) && axis < JOINT_DRIVE_AXES &&
           port->begin_operation(port->context, operation, axis, argument, now_ms);
}

static inline DriveOperationResult drive_commissioning_port_result(
    const DriveCommissioningPort *port)
{
    DriveOperationResult result = {0};
    result.status = DRIVE_OPERATION_TRANSPORT_ERROR;
    if (drive_commissioning_port_valid(port))
        result = port->operation_result(port->context);
    return result;
}

static inline void drive_commissioning_port_clear(
    const DriveCommissioningPort *port)
{
    if (drive_commissioning_port_valid(port))
        port->clear_operation(port->context);
}

static inline bool drive_commissioning_port_network_command(
    const DriveCommissioningPort *port, DriveNetworkCommand command)
{
    return drive_commissioning_port_valid(port) &&
           port->send_network_command(port->context, command);
}

static inline bool drive_commissioning_port_all_heartbeat_state(
    const DriveCommissioningPort *port, DriveNodeHeartbeatState state)
{
    return drive_commissioning_port_valid(port) &&
           port->all_heartbeat_state(port->context, state);
}

static inline void drive_commissioning_port_reset_runtime(
    const DriveCommissioningPort *port, uint32_t heartbeat_timeout_ms)
{
    if (drive_commissioning_port_valid(port))
        port->reset_runtime(port->context, heartbeat_timeout_ms);
}

#endif /* DRIVE_COMMISSIONING_PORT_H */
