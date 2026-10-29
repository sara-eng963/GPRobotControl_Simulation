#ifndef HANDGUIDING_FT_SERIAL_TRANSPORT_H
#define HANDGUIDING_FT_SERIAL_TRANSPORT_H

/* P05: generic USART2/RS-485 receive boundary. This unit does not know a
 * vendor wire format, own an HAL handle, configure a baud rate or authorize
 * motion. All clock values use the SAME CM7 monotonic microsecond clock.
 * ISR: ONE producer; FreeRTOS task: ONE consumer; never poll from an ISR.
 * The caller must bound a DMA callback to FT_SERIAL_ISR_MAX_BYTES. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>
#include "ft_sensor.h"

#define FT_SERIAL_RING_SIZE 512U
#define FT_SERIAL_ISR_MAX_BYTES 64U
#define FT_SERIAL_POLL_MAX_BYTES 256U

typedef struct {
    uint8_t octet;
    uint32_t observed_us; /* timestamp at ISR/DMA handoff, NOT sensor ADC time */
} FtSerialRxByte;

typedef struct {
    double force_N[3];
    double torque_Nm[3];
    uint32_t sequence; /* vendor sample sequence, not inferred from a timestamp */
} FtSerialDecoded;

typedef enum {
    FT_SERIAL_MORE = 0,
    FT_SERIAL_FRAME = 1,
    FT_SERIAL_CORRUPT = 2
} FtSerialDecodeResult;

typedef struct {
    /* Uncommissioned until a vendor-specification-backed decoder is provided.
     * A mock decoder is allowed ONLY in a host/simulation build. */
    void (*reset)(void *ctx);
    FtSerialDecodeResult (*feed)(void *ctx, uint8_t byte,
                                 FtSerialDecoded *decoded);
    bool (*partial)(const void *ctx);
    void *ctx; /* exclusively task-owned */
    bool commissioned; /* MUST be false for an unidentified physical sensor */
} FtSerialProtocol;

typedef enum {
    FT_SERIAL_STATUS_UNCOMMISSIONED = 0,
    FT_SERIAL_STATUS_DISCONNECTED,
    FT_SERIAL_STATUS_WAITING,
    FT_SERIAL_STATUS_STREAMING,
    FT_SERIAL_STATUS_FAULT
} FtSerialStatus;

typedef struct {
    uint32_t bytes_processed;
    uint32_t frames_accepted;
    uint32_t malformed_frames;
    uint32_t duplicate_or_old_sequences;
    uint32_t stale_frames;
    uint32_t partial_timeouts;
    uint32_t link_timeouts;
    uint32_t dma_overflow_events;
    uint32_t uart_error_events;
    uint32_t max_ring_depth;
    uint32_t last_sequence;
    uint32_t last_rx_us;
    bool ever_received;
    bool incident_latched; /* diagnostics; does not reauthorize motion */
} FtSerialDiagnostics;

typedef struct {
    FtSerialRxByte ring[FT_SERIAL_RING_SIZE];
    atomic_uint_least32_t write_cursor; /* producer-owned */
    atomic_uint_least32_t read_cursor;  /* consumer-owned */
    atomic_uint_least32_t overflow_events;
    atomic_uint_least32_t uart_errors;
    uint32_t overflow_seen; /* task-only */
    uint32_t uart_seen;     /* task-only */
    uint32_t link_timeout_us;
    uint32_t fragment_timeout_us;
    uint32_t max_sample_age_us;
    FtSerialProtocol protocol;
    FtSensorSample last_sample;
    uint32_t last_sequence;
    uint32_t last_byte_us;
    uint32_t partial_start_us;
    bool have_sequence;
    bool seen_byte;
    bool partial_started;
    bool link_timed_out;
    FtSerialStatus status;
    FtSerialDiagnostics diagnostics;
} FtSerialPort;

/* timeouts must be 1..INT32_MAX-1 microseconds; no resource allocation. */
bool ft_serial_init(FtSerialPort *port, const FtSerialProtocol *protocol,
                    uint32_t link_timeout_us, uint32_t fragment_timeout_us,
                    uint32_t max_sample_age_us);
/* ISR-only; all bytes in the batch receive the ISR handoff timestamp.
 * Overflow is whole-batch reject and latches an asynchronous fault event. */
bool ft_serial_isr_receive(FtSerialPort *port, const uint8_t *data,
                           size_t len, uint32_t isr_observed_us);
void ft_serial_isr_uart_error(FtSerialPort *port);
/* Task-only: processes at most min(budget, 256) octets, never blocks.
 * At a theoretical 1 Mbaud, 8N1 produces 200 bytes/2ms. A dedicated 1ms
 * service with budget >=128 gives 28% throughput margin; a single 2ms
 * service with budget >=256 gives the same margin. Neither is WCET proof. */
size_t ft_serial_poll(FtSerialPort *port, uint32_t now_us, size_t budget);
/* Task-only: returns a copy; no pointer to a mutable DMA buffer escapes. */
bool ft_serial_latest(FtSerialPort *port, uint32_t now_us,
                      FtSensorSample *sample);
const FtSerialDiagnostics *ft_serial_diagnostics(const FtSerialPort *port);

/* Adapter for existing HandGuidingDryRunConfig.read_sample signature.
 * The Supervisor/task must call update with its CM7-local time BEFORE tick.
 * This remains observation-only and does not access any CAN/drive API. */
typedef struct {
    FtSerialPort *port;
    uint32_t now_us;
    bool updated;
} FtSerialDryRunSampler;
void ft_serial_sampler_init(FtSerialDryRunSampler *sampler, FtSerialPort *port);
size_t ft_serial_sampler_update(FtSerialDryRunSampler *sampler,
                                uint32_t now_us, size_t byte_budget);
bool ft_serial_sampler_read(void *context, FtSensorSample *out);

#endif
