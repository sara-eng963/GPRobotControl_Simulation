#ifndef SOEM_BACKEND_H
#define SOEM_BACKEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * ============================================================================
 *  SOEM BACKEND
 * ============================================================================
 *
 * This is the SOEM-specific layer extracted from the already tested main.c.
 *
 * Everything above this file can talk in terms of EtherCAT operations without
 * directly touching ecx_contextt, IOmap, htoes(), etohl(), etc.
 *
 * No CiA-402 state-machine decisions live here.
 * No A6-EC register constants live here.
 * ============================================================================
 */

bool soem_backend_open(
    const char *interface_name
);

void soem_backend_close(void);


int soem_backend_scan(void);


const char *soem_backend_slave_name(
    int slave
);

bool soem_backend_slave_identity(
    int slave,
    uint32_t *vendor_id,
    uint32_t *product_code,
    uint32_t *revision,
    uint32_t *serial_number
);


int soem_backend_map_pdos(void);


int soem_backend_output_bytes(void);

int soem_backend_input_bytes(void);


int soem_backend_sdo_write(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t size,
    const void *data
);


int soem_backend_sdo_read(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t *size,
    void *data
);


bool soem_backend_has_error(void);

const char *soem_backend_pop_error_string(void);


bool soem_backend_configure_dc(void);


bool soem_backend_slave_has_dc(
    int slave
);


void soem_backend_sync0(
    int slave,
    bool enable,
    uint32_t cycle_time_ns,
    int32_t shift_ns
);


uint16_t soem_backend_statecheck(
    int slave,
    uint16_t requested_state,
    int timeout
);


int soem_backend_read_states(void);


uint16_t soem_backend_slave_state(
    int slave
);


void soem_backend_set_group_state(
    uint16_t state
);


int soem_backend_write_group_state(void);


const char *soem_backend_state_name(
    uint16_t state
);


int soem_backend_send_processdata(void);


int soem_backend_receive_processdata(void);


int soem_backend_expected_wkc(void);


uint8_t *soem_backend_slave_outputs(
    int slave
);


uint8_t *soem_backend_slave_inputs(
    int slave
);


/*
 * Exact byte-order helpers moved out of main.c.
 *
 * These still use SOEM's EtherCAT endian conversion helpers, exactly as the
 * tested implementation did.
 */
void soem_backend_write_u16(
    uint8_t *p,
    uint16_t value
);


void soem_backend_write_i32(
    uint8_t *p,
    int32_t value
);


uint16_t soem_backend_read_u16(
    const uint8_t *p
);


int32_t soem_backend_read_i32(
    const uint8_t *p
);


#endif /* SOEM_BACKEND_H */
