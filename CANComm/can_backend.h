#ifndef CAN_BACKEND_H
#define CAN_BACKEND_H

#include "can_types.h"

#include <stdbool.h>
#include <stddef.h>


/*
 * Result of a CAN backend operation.
 */
typedef enum
{
    CAN_BACKEND_OK = 0,

    /*
     * No frame is currently available.
     * This is not an error.
     */
    CAN_BACKEND_WOULD_BLOCK,

    CAN_BACKEND_ERROR

} CanBackendResult;


/*
 * Backend function signatures.
 *
 * context is backend-specific state.
 *
 * Examples:
 *
 *     SIL Kit:
 *         participant/controller/receive queue
 *
 *     STM32:
 *         FDCAN peripheral + RX queue
 *
 *     SocketCAN:
 *         Linux socket descriptor
 */
typedef CanBackendResult (*CanBackendSendFunction)(
    void *context,
    const CanFrame *frame
);


typedef CanBackendResult (*CanBackendReceiveFunction)(
    void *context,
    CanFrame *frame
);


typedef void (*CanBackendCloseFunction)(
    void *context
);


/*
 * Generic CAN transport interface.
 *
 * Higher-level code only sees this structure.
 * It does not know which CAN implementation is underneath.
 */
typedef struct
{
    void *context;

    CanBackendSendFunction send;
    CanBackendReceiveFunction receive;
    CanBackendCloseFunction close;

} CanBackend;


/*
 * Validate that a backend provides the minimum required operations.
 */
static inline bool can_backend_valid(
    const CanBackend *backend
)
{
    return
        backend != NULL &&
        backend->send != NULL &&
        backend->receive != NULL;
}


/*
 * Send one CAN frame through the selected backend.
 */
static inline CanBackendResult can_backend_send(
    CanBackend *backend,
    const CanFrame *frame
)
{
    if (
        !can_backend_valid(backend) ||
        frame == NULL
    )
    {
        return CAN_BACKEND_ERROR;
    }

    return backend->send(
        backend->context,
        frame
    );
}


/*
 * Receive one CAN frame.
 *
 * CAN_BACKEND_WOULD_BLOCK means that no frame
 * is currently available.
 */
static inline CanBackendResult can_backend_receive(
    CanBackend *backend,
    CanFrame *frame
)
{
    if (
        !can_backend_valid(backend) ||
        frame == NULL
    )
    {
        return CAN_BACKEND_ERROR;
    }

    return backend->receive(
        backend->context,
        frame
    );
}


/*
 * Shut down the backend.
 */
static inline void can_backend_close(
    CanBackend *backend
)
{
    if (
        backend == NULL ||
        backend->close == NULL
    )
    {
        return;
    }

    backend->close(
        backend->context
    );
}

#endif /* CAN_BACKEND_H */