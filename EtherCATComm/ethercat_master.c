#include "ethercat_master.h"

#include "SOEM/soem_backend.h"

#include <stdio.h>
#include <string.h>

/*
 * SOEM EtherCAT state values.
 *
 * They remain implementation details of the SOEM-backed communication layer;
 * main.c no longer needs to include soem/soem.h just to inspect EtherCAT state.
 */
#include "soem/soem.h"


/* ============================================================================
 *  MODULE STATE
 * ============================================================================
 */

static EtherCATMasterConfig master_config;

static EtherCATBusInfo bus_info;

static bool configured =
    false;


/* ============================================================================
 *  INITIALIZATION
 * ============================================================================
 */

bool ethercat_master_init(
    const EtherCATMasterConfig *config
)
{
    if (
        config == NULL ||
        config->interfaceName == NULL ||
        config->expectedSlaveCount <= 0 ||
        config->cycleTimeNs == 0
    )
    {
        return false;
    }


    master_config =
        *config;


    memset(
        &bus_info,
        0,
        sizeof(bus_info)
    );


    configured =
        true;


    return true;
}


bool ethercat_master_open(void)
{
    if (!configured)
    {
        return false;
    }


    printf(
        "Opening SOEM on %s...\n",
        master_config.interfaceName
    );


    if (
        !soem_backend_open(
            master_config.interfaceName
        )
    )
    {
        printf(
            "ERROR: Could not open %s\n",
            master_config.interfaceName
        );

        return false;
    }


    printf(
        "SOEM initialized successfully\n"
    );


    return true;
}


/* ============================================================================
 *  DISCOVERY
 * ============================================================================
 */

int ethercat_master_scan(void)
{
    if (!configured)
    {
        return 0;
    }


    printf(
        "\nScanning EtherCAT bus...\n"
    );


    bus_info.slaveCount =
        soem_backend_scan();


    if (
        bus_info.slaveCount <= 0
    )
    {
        printf(
            "ERROR: No EtherCAT slaves found\n"
        );

        return bus_info.slaveCount;
    }


    printf(
        "%d EtherCAT slave(s) found\n",
        bus_info.slaveCount
    );


    for (int slave = 1;
     slave <= bus_info.slaveCount;
     slave++)
{
    printf(
        "Slave %d: %s\n",
        slave,
        soem_backend_slave_name(
            slave
        )
    );


    EtherCATSlaveIdentity identity;


    if (
        ethercat_master_slave_identity(
            slave,
            &identity
        )
    )
    {
        printf(
            "  Vendor ID    : 0x%08lX\n"
            "  Product Code : 0x%08lX\n"
            "  Revision     : 0x%08lX\n"
            "  Serial Number: 0x%08lX\n",
            (unsigned long)identity.vendorId,
            (unsigned long)identity.productCode,
            (unsigned long)identity.revision,
            (unsigned long)identity.serialNumber
        );
    }
}


    if (
        bus_info.slaveCount !=
        master_config.expectedSlaveCount
    )
    {
        printf(
            "\nERROR: Expected %d slaves but found %d\n",
            master_config.expectedSlaveCount,
            bus_info.slaveCount
        );
    }


    return
        bus_info.slaveCount;
}


const EtherCATBusInfo *ethercat_master_info(void)
{
    return
        &bus_info;
}


const char *ethercat_master_slave_name(
    int slave
)
{
    return
        soem_backend_slave_name(
            slave
        );
}
bool ethercat_master_slave_identity(
    int slave,
    EtherCATSlaveIdentity *identity
)
{
    if (
        identity == NULL ||
        slave <= 0 ||
        slave > bus_info.slaveCount
    )
    {
        return false;
    }


    return
        soem_backend_slave_identity(
            slave,
            &identity->vendorId,
            &identity->productCode,
            &identity->revision,
            &identity->serialNumber
        );
}


/* ============================================================================
 *  PDO MAPPING
 * ============================================================================
 */

int ethercat_master_map_pdos(void)
{
    printf(
        "\nMapping PDOs...\n"
    );


    bus_info.mappedBytes =
        soem_backend_map_pdos();


    bus_info.outputBytes =
        soem_backend_output_bytes();


    bus_info.inputBytes =
        soem_backend_input_bytes();


    printf(
        "Mapped bytes: %d\n",
        bus_info.mappedBytes
    );


    printf(
        "Outputs: %d bytes | Inputs: %d bytes\n",
        bus_info.outputBytes,
        bus_info.inputBytes
    );


    return
        bus_info.mappedBytes;
}


/* ============================================================================
 *  SDO ACCESS
 * ============================================================================
 */

int ethercat_master_sdo_write(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t size,
    const void *data
)
{
    return
        soem_backend_sdo_write(
            slave,
            index,
            subindex,
            size,
            data
        );
}


int ethercat_master_sdo_read(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t *size,
    void *data
)
{
    return
        soem_backend_sdo_read(
            slave,
            index,
            subindex,
            size,
            data
        );
}


bool ethercat_master_has_error(void)
{
    return
        soem_backend_has_error();
}


const char *ethercat_master_pop_error_string(void)
{
    return
        soem_backend_pop_error_string();
}


/* ============================================================================
 *  DISTRIBUTED CLOCKS
 * ============================================================================
 */

bool ethercat_master_configure_distributed_clocks(void)
{
    printf(
        "\nConfiguring Distributed Clocks...\n"
    );


    bus_info.dcFound =
        soem_backend_configure_dc();


    printf(
        "DC-capable bus: %s\n",
        bus_info.dcFound ? "YES" : "NO"
    );


    for (int slave = 1;
         slave <= bus_info.slaveCount;
         slave++)
    {
        if (
            soem_backend_slave_has_dc(
                slave
            )
        )
        {
            printf(
                "Slave %d: DC supported -> requesting 1 ms SYNC0\n",
                slave
            );


            soem_backend_sync0(
                slave,
                true,
                master_config.cycleTimeNs,
                0
            );
        }
        else
        {
            printf(
                "Slave %d: no DC support\n",
                slave
            );
        }
    }


    return
        bus_info.dcFound;
}


/* ============================================================================
 *  SAFE-OP / OPERATIONAL
 * ============================================================================
 */

bool ethercat_master_wait_for_safe_op(void)
{
    printf(
        "\nWaiting for SAFE-OP...\n"
    );


    soem_backend_statecheck(
        0,
        EC_STATE_SAFE_OP,
        EC_TIMEOUTSTATE * 4
    );


    soem_backend_read_states();


    bool all_safe_op =
        true;


    for (int slave = 1;
         slave <= bus_info.slaveCount;
         slave++)
    {
        uint16_t state =
            soem_backend_slave_state(
                slave
            );


        printf(
            "Slave %d: %s\n",
            slave,
            soem_backend_state_name(
                state
            )
        );


        if (
            state != EC_STATE_SAFE_OP
        )
        {
            all_safe_op =
                false;
        }
    }


    return
        all_safe_op;
}


int ethercat_master_exchange(void)
{
    soem_backend_send_processdata();

    return
        soem_backend_receive_processdata();
}


bool ethercat_master_request_operational(void)
{
    printf(
        "\nRequesting OPERATIONAL...\n"
    );


    soem_backend_set_group_state(
        EC_STATE_OPERATIONAL
    );


    soem_backend_write_group_state();


    for (int attempt = 0;
         attempt < 50;
         attempt++)
    {
        soem_backend_send_processdata();

        soem_backend_receive_processdata();


        soem_backend_statecheck(
            0,
            EC_STATE_OPERATIONAL,
            EC_TIMEOUTSTATE / 10
        );


        if (
            soem_backend_slave_state(0) ==
            EC_STATE_OPERATIONAL
        )
        {
            break;
        }
    }


    soem_backend_read_states();


    bool all_operational =
        true;


    for (int slave = 1;
         slave <= bus_info.slaveCount;
         slave++)
    {
        uint16_t state =
            soem_backend_slave_state(
                slave
            );


        printf(
            "Slave %d final state: %s (0x%02X)\n",
            slave,
            soem_backend_state_name(
                state
            ),
            state
        );


        if (
            state !=
            EC_STATE_OPERATIONAL
        )
        {
            all_operational =
                false;
        }
    }


    bus_info.expectedWkc =
        soem_backend_expected_wkc();


    printf(
        "\nExpected WKC = %d\n",
        bus_info.expectedWkc
    );


    return
        all_operational;
}


/* ============================================================================
 *  CYCLIC ACCESS
 * ============================================================================
 */

int ethercat_master_expected_wkc(void)
{
    return
        bus_info.expectedWkc;
}


uint8_t *ethercat_master_slave_outputs(
    int slave
)
{
    return
        soem_backend_slave_outputs(
            slave
        );
}


uint8_t *ethercat_master_slave_inputs(
    int slave
)
{
    return
        soem_backend_slave_inputs(
            slave
        );
}


uint16_t ethercat_master_slave_state(
    int slave
)
{
    return
        soem_backend_slave_state(
            slave
        );
}


const char *ethercat_master_state_name(
    uint16_t state
)
{
    return
        soem_backend_state_name(
            state
        );
}


/* ============================================================================
 *  CLEAN ETHERCAT SHUTDOWN
 * ============================================================================
 */

void ethercat_master_close(void)
{
    for (int slave = 1;
         slave <= bus_info.slaveCount;
         slave++)
    {
        if (
            soem_backend_slave_has_dc(
                slave
            )
        )
        {
            soem_backend_sync0(
                slave,
                false,
                master_config.cycleTimeNs,
                0
            );
        }
    }


    soem_backend_close();
}


/* ============================================================================
 *  PDO BYTE-ORDER HELPERS
 * ============================================================================
 */

void ethercat_pdo_write_u16(
    uint8_t *p,
    uint16_t value
)
{
    soem_backend_write_u16(
        p,
        value
    );
}


void ethercat_pdo_write_i32(
    uint8_t *p,
    int32_t value
)
{
    soem_backend_write_i32(
        p,
        value
    );
}


uint16_t ethercat_pdo_read_u16(
    const uint8_t *p
)
{
    return
        soem_backend_read_u16(
            p
        );
}


int32_t ethercat_pdo_read_i32(
    const uint8_t *p
)
{
    return
        soem_backend_read_i32(
            p
        );
}
