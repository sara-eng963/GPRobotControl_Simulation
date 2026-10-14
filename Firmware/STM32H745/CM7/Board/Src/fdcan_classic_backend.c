/*
 * STM32H745 CM7 CANopen physical transport using official STM32CubeH7 HAL1.
 * Not an FDCAN initializer: CubeMX must first generate clocks/pins/Message RAM.
 * Absent HAL_FDCAN_MODULE_ENABLED, deliberately compiles to no adapter.
 */
#include "main.h"

#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "fdcan_classic_backend.h"
#include <string.h>

/* HAL1 uses DLC codes, not arbitrary byte counts. Classic CAN's valid
 * values are 0..8, but keep explicit official HAL constants here. */
static const uint32_t k_dlc[CAN_CLASSIC_MAX_DATA_BYTES + 1U] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
};

static bool fdcan_running(const GpFdcanClassicBackend *state)
{
    if (state == NULL || !state->started || state->handle == NULL ||
        HAL_FDCAN_GetState(state->handle) != HAL_FDCAN_STATE_BUSY)
        return false;

    FDCAN_ProtocolStatusTypeDef status = {0};
    return HAL_FDCAN_GetProtocolStatus(state->handle, &status) == HAL_OK &&
           status.BusOff == 0U;
}

static CanBackendResult fdcan_send(void *context, const CanFrame *frame)
{
    GpFdcanClassicBackend *state = (GpFdcanClassicBackend *)context;
    if (!fdcan_running(state) || frame == NULL ||
        frame->id > CAN_STANDARD_ID_MAX ||
        frame->dlc > CAN_CLASSIC_MAX_DATA_BYTES)
        return CAN_BACKEND_ERROR;

    if (HAL_FDCAN_GetTxFifoFreeLevel(state->handle) == 0U)
        return CAN_BACKEND_WOULD_BLOCK;

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = frame->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = k_dlc[frame->dlc];
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    /* HAL requires a non-null buffer even for a DLC=0 heartbeat/SYNC. */
    return HAL_FDCAN_AddMessageToTxFifoQ(
        state->handle, &header, frame->data) == HAL_OK
        ? CAN_BACKEND_OK : CAN_BACKEND_ERROR;
}

static CanBackendResult fdcan_receive(void *context, CanFrame *frame)
{
    GpFdcanClassicBackend *state = (GpFdcanClassicBackend *)context;
    if (!fdcan_running(state) || frame == NULL)
        return CAN_BACKEND_ERROR;

    if (HAL_FDCAN_GetRxFifoFillLevel(state->handle, FDCAN_RX_FIFO0) == 0U)
        return CAN_BACKEND_WOULD_BLOCK;

    FDCAN_RxHeaderTypeDef header = {0};
    /* Capacity 64 protects the HAL copy even if a malformed FD frame arrives
     * despite the Classic CAN configuration. Never pass it to CANopen. */
    uint8_t payload[64] = {0};
    if (HAL_FDCAN_GetRxMessage(
            state->handle, FDCAN_RX_FIFO0, &header, payload) != HAL_OK)
        return CAN_BACKEND_ERROR;

    if (header.IdType != FDCAN_STANDARD_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME ||
        header.FDFormat != FDCAN_CLASSIC_CAN ||
        header.Identifier > CAN_STANDARD_ID_MAX ||
        header.DataLength > FDCAN_DLC_BYTES_8)
        return CAN_BACKEND_ERROR;

    CanFrame received = {0};
    received.id = (uint16_t)header.Identifier;
    received.dlc = (uint8_t)header.DataLength;
    memcpy(received.data, payload, received.dlc);
    *frame = received;
    return CAN_BACKEND_OK;
}

static void fdcan_close(void *context)
{
    GpFdcanClassicBackend *state = (GpFdcanClassicBackend *)context;
    if (state == NULL || !state->started || state->handle == NULL)
        return;

    (void)HAL_FDCAN_Stop(state->handle);
    state->started = false;
}

/* Never change CAN bit timing here. The generated HAL_FDCAN_Init owns it. */
bool gp_fdcan_classic_backend_open(GpFdcanClassicBackend *state,
                                   CanBackend *interface,
                                   FDCAN_HandleTypeDef *fdcan)
{
    if (state == NULL || interface == NULL || fdcan == NULL ||
        HAL_FDCAN_GetState(fdcan) != HAL_FDCAN_STATE_READY ||
        fdcan->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
        fdcan->Init.Mode != FDCAN_MODE_NORMAL ||
        fdcan->Init.TxFifoQueueMode != FDCAN_TX_FIFO_OPERATION ||
        fdcan->Init.TxFifoQueueElmtsNbr < 16U ||
        fdcan->Init.RxFifo0ElmtsNbr < 16U ||
        fdcan->Init.TxElmtSize != FDCAN_DATA_BYTES_8 ||
        fdcan->Init.RxFifo0ElmtSize != FDCAN_DATA_BYTES_8)
        return false;

    /* Accept 11-bit CANopen traffic; reject all extended/remote traffic.
     * Global filter is an official ST HAL1 API, not custom register writes.
     * A later bounded acceptance-filter set can reduce unrelated bus load. */
    if (HAL_FDCAN_ConfigGlobalFilter(
            fdcan,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
        return false;

    if (HAL_FDCAN_Start(fdcan) != HAL_OK)
        return false;

    state->handle = fdcan;
    state->started = true;
    interface->context = state;
    interface->send = fdcan_send;
    interface->receive = fdcan_receive;
    interface->close = fdcan_close;
    return true;
}

#endif /* HAL_FDCAN_MODULE_ENABLED */
