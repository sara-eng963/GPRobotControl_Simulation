#!/usr/bin/env bash
# Host-only HAL1 adapter fault-path test. Official HAL headers/firmware build
# are the authoritative integration gate once CubeMX enables FDCAN.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DIR="$(mktemp -d)"
trap 'rm -rf "$DIR"' EXIT
cat > "$DIR/main.h" <<'MOCK_H'
#ifndef MOCK_STM32_MAIN_H
#define MOCK_STM32_MAIN_H
#include <stdint.h>
#include <stdbool.h>
#define HAL_FDCAN_MODULE_ENABLED 1

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum {
  HAL_FDCAN_STATE_READY = 1, HAL_FDCAN_STATE_BUSY = 2,
  HAL_FDCAN_STATE_ERROR = 3
} HAL_FDCAN_StateTypeDef;
typedef struct {
    uint32_t FrameFormat, Mode, TxFifoQueueMode;
    uint32_t TxFifoQueueElmtsNbr, RxFifo0ElmtsNbr;
    uint32_t TxElmtSize, RxFifo0ElmtSize;
} FDCAN_InitTypeDef;
typedef struct {
    FDCAN_InitTypeDef Init;
    HAL_FDCAN_StateTypeDef State;
} FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat;
} FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;

#define FDCAN_FRAME_CLASSIC 0x10U
#define FDCAN_MODE_NORMAL 0x11U
#define FDCAN_TX_FIFO_OPERATION 0x12U
#define FDCAN_DATA_BYTES_8 0x13U
#define FDCAN_STANDARD_ID 0x14U
#define FDCAN_DATA_FRAME 0x15U
#define FDCAN_ESI_ACTIVE 0x16U
#define FDCAN_BRS_OFF 0x17U
#define FDCAN_CLASSIC_CAN 0x18U
#define FDCAN_NO_TX_EVENTS 0x19U
#define FDCAN_RX_FIFO0 0x1AU
#define FDCAN_ACCEPT_IN_RX_FIFO0 0x1BU
#define FDCAN_REJECT 0x1CU
#define FDCAN_REJECT_REMOTE 0x1DU
#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U

HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
#endif
MOCK_H

cat > "$DIR/test.c" <<'MOCK_TEST'
#include "fdcan_classic_backend.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t free_slots, available_rx, bus_off;
static int started, stopped, global_filter_calls, tx_calls;
static FDCAN_TxHeaderTypeDef last_tx;
static FDCAN_RxHeaderTypeDef next_rx;
static uint8_t rx_data[64];

HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(const FDCAN_HandleTypeDef *h)
{ return h->State; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *s)
{ (void)h; s->BusOff=bus_off; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    (void)h;
    assert(a==FDCAN_ACCEPT_IN_RX_FIFO0);
    assert(b==FDCAN_REJECT);
    assert(c==FDCAN_REJECT_REMOTE && d==FDCAN_REJECT_REMOTE);
    global_filter_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{ started++; h->State=HAL_FDCAN_STATE_BUSY; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h)
{ stopped++; h->State=HAL_FDCAN_STATE_READY; return HAL_OK; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h)
{ (void)h; return free_slots; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *h, const FDCAN_TxHeaderTypeDef *t, const uint8_t *d)
{
    (void)h;
    assert(d != NULL);
    last_tx = *t;
    tx_calls++;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; assert(fifo == FDCAN_RX_FIFO0); return available_rx; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *h, uint32_t fifo, FDCAN_RxHeaderTypeDef *r, uint8_t *d)
{
    (void)h;
    assert(fifo == FDCAN_RX_FIFO0);
    assert(available_rx != 0U);
    available_rx--;
    *r=next_rx;
    memcpy(d,rx_data,sizeof(rx_data));
    return HAL_OK;
}

int main(void)
{
    FDCAN_HandleTypeDef h={0};
    h.State=HAL_FDCAN_STATE_READY;
    h.Init.FrameFormat=FDCAN_FRAME_CLASSIC;
    h.Init.Mode=FDCAN_MODE_NORMAL;
    h.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;
    h.Init.TxFifoQueueElmtsNbr=16U;
    h.Init.RxFifo0ElmtsNbr=16U;
    h.Init.TxElmtSize=FDCAN_DATA_BYTES_8;
    h.Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8;
    GpFdcanClassicBackend state={0};
    CanBackend bus={0};

    assert(!gp_fdcan_classic_backend_open(NULL,&bus,&h));
    h.Init.TxFifoQueueElmtsNbr=6U;
    assert(!gp_fdcan_classic_backend_open(&state,&bus,&h));
    h.Init.TxFifoQueueElmtsNbr=16U;
    assert(gp_fdcan_classic_backend_open(&state,&bus,&h));
    assert(started==1 && global_filter_calls==1);
    assert(can_backend_valid(&bus));

    CanFrame f={0};
    f.id=0x801U; f.dlc=0U;
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_ERROR);
    f.id=0x123U; f.dlc=9U;
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_ERROR);
    f.dlc=8U;
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_WOULD_BLOCK);
    free_slots=16U;
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_OK);
    assert(tx_calls==1 && last_tx.Identifier==0x123U);
    assert(last_tx.IdType==FDCAN_STANDARD_ID);
    assert(last_tx.FDFormat==FDCAN_CLASSIC_CAN);
    assert(last_tx.DataLength==FDCAN_DLC_BYTES_8);

    assert(can_backend_receive(&bus,&f)==CAN_BACKEND_WOULD_BLOCK);
    next_rx.Identifier=0x345U;
    next_rx.IdType=FDCAN_STANDARD_ID;
    next_rx.RxFrameType=FDCAN_DATA_FRAME;
    next_rx.FDFormat=FDCAN_CLASSIC_CAN;
    next_rx.DataLength=FDCAN_DLC_BYTES_4;
    rx_data[0]=1; rx_data[1]=2; rx_data[2]=3; rx_data[3]=4;
    available_rx=1U;
    assert(can_backend_receive(&bus,&f)==CAN_BACKEND_OK);
    assert(f.id==0x345U && f.dlc==4U && f.data[3]==4U);

    next_rx.FDFormat=0xFFFFFFFFU;
    available_rx=1U;
    assert(can_backend_receive(&bus,&f)==CAN_BACKEND_ERROR);
    bus_off=1U;
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_ERROR);
    assert(can_backend_receive(&bus,&f)==CAN_BACKEND_ERROR);
    bus_off=0U;
    can_backend_close(&bus);
    assert(stopped==1 && !state.started);
    assert(can_backend_send(&bus,&f)==CAN_BACKEND_ERROR);

    puts("PASS: HAL1 Classic CAN backend mock frame and failure-path tests");
    return 0;
}
MOCK_TEST

gcc -std=c11 -O0 -Wall -Wextra -Werror \
    -I"$DIR" \
    -I"$ROOT/Firmware/STM32H745/CM7/Board/Inc" \
    -I"$ROOT/CANComm" \
    "$ROOT/Firmware/STM32H745/CM7/Board/Src/fdcan_classic_backend.c" \
    "$DIR/test.c" -o "$DIR/fdcan_test"
"$DIR/fdcan_test"
echo "NOTE: mocked HAL1, not the STM32CubeMX-generated ST driver or real CAN."
