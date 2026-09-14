#include "soem_backend.h"

#include "soem/soem.h"

#include <string.h>

/*
 * ============================================================================
 *  SOEM STATE
 * ============================================================================
 *
 * These are the same two pieces of global EtherCAT state that previously lived
 * in main.c:
 *
 *      static ecx_contextt soem_context;
 *      static uint8_t IOmap[4096];
 * ============================================================================
 */

static ecx_contextt soem_context;

static uint8_t IOmap[4096];


/* ============================================================================
 *  OPEN / CLOSE
 * ============================================================================
 */

bool soem_backend_open(
    const char *interface_name
)
{
    memset(
        &soem_context,
        0,
        sizeof(soem_context)
    );

    memset(
        IOmap,
        0,
        sizeof(IOmap)
    );

    return
        ecx_init(
            &soem_context,
            interface_name
        ) != 0;
}


void soem_backend_close(void)
{
    ecx_close(
        &soem_context
    );
}


/* ============================================================================
 *  DISCOVERY
 * ============================================================================
 */

int soem_backend_scan(void)
{
    return
        ecx_config_init(
            &soem_context
        );
}


const char *soem_backend_slave_name(
    int slave
)
{
    return
        soem_context
            .slavelist[slave]
            .name;
}


/* ============================================================================
 *  PDO MAPPING
 * ============================================================================
 */

int soem_backend_map_pdos(void)
{
    return
        ecx_config_map_group(
            &soem_context,
            IOmap,
            0
        );
}


int soem_backend_output_bytes(void)
{
    return
        soem_context
            .grouplist[0]
            .Obytes;
}


int soem_backend_input_bytes(void)
{
    return
        soem_context
            .grouplist[0]
            .Ibytes;
}


/* ============================================================================
 *  SDO ACCESS
 * ============================================================================
 *
 * Same SOEM calls and timeout used by the tested main.c.
 * ============================================================================
 */

int soem_backend_sdo_write(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t size,
    const void *data
)
{
    return
        ecx_SDOwrite(
            &soem_context,
            slave,
            index,
            subindex,
            FALSE,
            (int)size,
            (void *)data,
            EC_TIMEOUTRXM
        );
}


int soem_backend_sdo_read(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t *size,
    void *data
)
{
    if (
        size == NULL
    )
    {
        return 0;
    }

    int soem_size =
        (int)(*size);

    int wkc =
        ecx_SDOread(
            &soem_context,
            slave,
            index,
            subindex,
            FALSE,
            &soem_size,
            data,
            EC_TIMEOUTRXM
        );

    *size =
        (size_t)soem_size;

    return wkc;
}


bool soem_backend_has_error(void)
{
    return
        soem_context.ecaterror != 0;
}


const char *soem_backend_pop_error_string(void)
{
    return
        ecx_elist2string(
            &soem_context
        );
}


/* ============================================================================
 *  DISTRIBUTED CLOCKS
 * ============================================================================
 */

bool soem_backend_configure_dc(void)
{
    return
        ecx_configdc(
            &soem_context
        ) != 0;
}


bool soem_backend_slave_has_dc(
    int slave
)
{
    return
        soem_context
            .slavelist[slave]
            .hasdc != 0;
}


void soem_backend_sync0(
    int slave,
    bool enable,
    uint32_t cycle_time_ns,
    int32_t shift_ns
)
{
    ecx_dcsync0(
        &soem_context,
        slave,
        enable ? TRUE : FALSE,
        cycle_time_ns,
        shift_ns
    );
}


/* ============================================================================
 *  ETHERCAT COMMUNICATION STATE
 * ============================================================================
 */

uint16_t soem_backend_statecheck(
    int slave,
    uint16_t requested_state,
    int timeout
)
{
    return
        ecx_statecheck(
            &soem_context,
            slave,
            requested_state,
            timeout
        );
}


int soem_backend_read_states(void)
{
    return
        ecx_readstate(
            &soem_context
        );
}


uint16_t soem_backend_slave_state(
    int slave
)
{
    return
        soem_context
            .slavelist[slave]
            .state;
}


void soem_backend_set_group_state(
    uint16_t state
)
{
    soem_context
        .slavelist[0]
        .state =
            state;
}


int soem_backend_write_group_state(void)
{
    return
        ecx_writestate(
            &soem_context,
            0
        );
}


const char *soem_backend_state_name(
    uint16_t state
)
{
    switch (state)
    {
        case EC_STATE_INIT:
            return "INIT";

        case EC_STATE_PRE_OP:
            return "PRE-OP";

        case EC_STATE_SAFE_OP:
            return "SAFE-OP";

        case EC_STATE_OPERATIONAL:
            return "OPERATIONAL";

        case EC_STATE_SAFE_OP + EC_STATE_ERROR:
            return "SAFE-OP + ERROR";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================================
 *  CYCLIC PROCESS DATA
 * ============================================================================
 */

int soem_backend_send_processdata(void)
{
    return
        ecx_send_processdata(
            &soem_context
        );
}


int soem_backend_receive_processdata(void)
{
    return
        ecx_receive_processdata(
            &soem_context,
            EC_TIMEOUTRET
        );
}


int soem_backend_expected_wkc(void)
{
    return
        (
            soem_context
                .grouplist[0]
                .outputsWKC * 2
        )
        +
        soem_context
            .grouplist[0]
            .inputsWKC;
}


uint8_t *soem_backend_slave_outputs(
    int slave
)
{
    return
        soem_context
            .slavelist[slave]
            .outputs;
}


uint8_t *soem_backend_slave_inputs(
    int slave
)
{
    return
        soem_context
            .slavelist[slave]
            .inputs;
}


/* ============================================================================
 *  PDO BYTE-ORDER HELPERS
 * ============================================================================
 *
 * Exact logic moved from the tested main.c.
 * ============================================================================
 */

void soem_backend_write_u16(
    uint8_t *p,
    uint16_t value
)
{
    uint16_t ethercat_value =
        htoes(value);

    memcpy(
        p,
        &ethercat_value,
        sizeof(ethercat_value)
    );
}


void soem_backend_write_i32(
    uint8_t *p,
    int32_t value
)
{
    uint32_t ethercat_value =
        htoel(
            (uint32_t)value
        );

    memcpy(
        p,
        &ethercat_value,
        sizeof(ethercat_value)
    );
}


uint16_t soem_backend_read_u16(
    const uint8_t *p
)
{
    uint16_t value;

    memcpy(
        &value,
        p,
        sizeof(value)
    );

    return
        etohs(value);
}


int32_t soem_backend_read_i32(
    const uint8_t *p
)
{
    uint32_t value;

    memcpy(
        &value,
        p,
        sizeof(value)
    );

    value =
        etohl(value);

    return
        (int32_t)value;
}
