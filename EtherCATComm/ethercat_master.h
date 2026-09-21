#ifndef ETHERCAT_MASTER_H
#define ETHERCAT_MASTER_H

#include "ethercat_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * ============================================================================
 *  ETHERCAT MASTER PUBLIC API
 * ============================================================================
 *
 * This is the public EtherCAT-facing layer extracted from the tested main.c.
 *
 * It intentionally does NOT contain:
 *
 *      - CiA-402 enable/disable decisions
 *      - A6-EC object/register definitions
 *      - trajectory planning
 *      - HMI logic
 *
 * The SOEM-specific implementation is hidden below this layer.
 * ============================================================================
 */


/* Store the same configuration that previously lived as #defines in main.c. */
bool ethercat_master_init(
    const EtherCATMasterConfig *config
);


/* Open the requested network interface. */
bool ethercat_master_open(void);


/* Scan/discover slaves. Returns discovered count. */
int ethercat_master_scan(void);


/* Current bus information collected by the wrapper. */
const EtherCATBusInfo *ethercat_master_info(void);


/* Human-readable name reported by one slave. */
const char *ethercat_master_slave_name(
    int slave
);

bool ethercat_master_slave_identity(
    int slave,
    EtherCATSlaveIdentity *identity
);


/* Map PDO process data into the existing SOEM IOmap. */
int ethercat_master_map_pdos(void);


/*
 * Generic SDO access.
 *
 * These use the same complete-access flag and SOEM mailbox timeout that the
 * tested main.c used.
 */
int ethercat_master_sdo_write(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t size,
    const void *data
);


int ethercat_master_sdo_read(
    int slave,
    uint16_t index,
    uint8_t subindex,
    size_t *size,
    void *data
);


bool ethercat_master_has_error(void);

const char *ethercat_master_pop_error_string(void);


/* Configure DC and request the configured SYNC0 cycle on every DC-capable slave. */
bool ethercat_master_configure_distributed_clocks(void);


/* Wait for all slaves to reach EtherCAT SAFE-OP. */
bool ethercat_master_wait_for_safe_op(void);


/*
 * Send/receive one initial process-data frame, matching the tested startup
 * sequence before OPERATIONAL is requested.
 */
int ethercat_master_exchange(void);


/*
 * Request EtherCAT OPERATIONAL using the same 50-attempt startup loop already
 * tested in main.c.
 */
bool ethercat_master_request_operational(void);


/* Expected EtherCAT Working Counter for group 0. */
int ethercat_master_expected_wkc(void);


/* Raw PDO regions for one slave. */
uint8_t *ethercat_master_slave_outputs(
    int slave
);


uint8_t *ethercat_master_slave_inputs(
    int slave
);


/* Raw EtherCAT communication state for one slave. */
uint16_t ethercat_master_slave_state(
    int slave
);

bool ethercat_master_slave_status(
    int slave,
    EtherCATSlaveStatus *status
);


void ethercat_master_refresh_slave_states(void);

void ethercat_master_print_slave_diagnostics(void);

const char *ethercat_master_al_status_name(
    uint16_t alStatusCode
);


const char *ethercat_master_state_name(
    uint16_t state
);

bool ethercat_master_all_slaves_operational(
    int slaveCount,
    int *failedSlave
);

/*
 * Disable SYNC0 on DC-capable slaves and close SOEM.
 *
 * This is the same EtherCAT-side shutdown that was previously in main.c.
 * CiA-402 drive disabling still happens outside this module.
 */
void ethercat_master_close(void);


/* ============================================================================
 *  PDO BYTE-ORDER HELPERS
 * ============================================================================
 *
 * These are the exact helpers moved from main.c, still backed by SOEM's
 * htoes()/htoes()/etohs()/etohl() conversion helpers.
 * ============================================================================
 */

void ethercat_pdo_write_u16(
    uint8_t *p,
    uint16_t value
);


void ethercat_pdo_write_i32(
    uint8_t *p,
    int32_t value
);


uint16_t ethercat_pdo_read_u16(
    const uint8_t *p
);


int32_t ethercat_pdo_read_i32(
    const uint8_t *p
);

EtherCATRecoveryAction ethercat_master_recovery_step(
    int slave
);

#endif /* ETHERCAT_MASTER_H */
