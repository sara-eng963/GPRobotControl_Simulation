#ifndef W25Q_NOR_MODEL_H
#define W25Q_NOR_MODEL_H
/* Host-only model extracted from Batch 2. Literal protocol values, optional
 * storage trace annotations, external memory for full-address protocol tests.
 * Never compile this simulator into MCU/control firmware. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SECTOR 4096U
#define PAGE 256U
#define SLOT_BYTES 16384U
#define IMAGE_BYTES (2U * SLOT_BYTES)
#define MAX_EVENTS 512U
typedef enum {
    DISCOVERY, HEADER_ERASE, ERASE_READBACK, PAYLOAD_ERASE, PAYLOAD_PROGRAM,
    PAYLOAD_VERIFY, HEADER_PROGRAM, HEADER_VERIFY, MARKER_PROGRAM,
    MARKER_VERIFY, FINAL_PAYLOAD_VERIFY, STAGE_COUNT
} Stage;
extern const char *const stage_names[STAGE_COUNT];
typedef enum {
    NO_FAULT, CUT_BEFORE, CUT_DURING, CUT_AFTER, IGNORE_WRITE,
    TRUNCATE_PROGRAM, SILENT_CORRUPTION, READ_ERROR,
    CORRUPT_PAYLOAD_AT_MARKER, CORRUPT_HEADER_AT_MARKER
} Fault;
typedef struct { uint8_t op; uint32_t at; size_t size; Stage stage; } Event;
typedef struct {
    uint8_t bytes[IMAGE_BYTES];
    uint8_t *external_bytes;
    size_t memory_size;
    bool enforce_target, reject_wren, stuck_wip, ignored_clears_wel, keep_wel;
    bool suppress_event_trace; /* false when zero-initialized: preserve test traces */
    bool reset_enabled;
    uint32_t marker_offset, late_corrupt_header_offset, jedec_id, op_calls[256], unknown_commands;
    uint32_t protect_begin, protect_end, busy_rejections;
    uint64_t read_allowed_us, write_allowed_us, reset_until_us;
    uint64_t program_latency_us, erase_latency_us, clock_step_us;
    uint64_t status_latency_us;
    size_t reset_partial;
    bool powered, wel, updating, header_started, marker_started;
    uint64_t now, busy_until;
    uint8_t pending_op, pending_data[PAGE];
    uint32_t pending_at, target_base;
    size_t pending_size;
    Event events[MAX_EVENTS];
    unsigned event_count, fail_event, erase_count;
    Fault fault;
    size_t partial;
    bool fired;
} Model;

void w25_model_init(Model *m, uint8_t *external_bytes, size_t memory_size);
void w25_model_power_cycle(Model *m);
bool w25_model_command(void *ctx, uint8_t op, uint32_t at,
                       const uint8_t *tx, size_t txsz, uint8_t *rx, size_t rxsz);
uint64_t w25_model_time(void *ctx);
void w25_model_idle(void *ctx);
#endif
