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
 * Bus transport only.  No CiA-402 policy, A6-EC object knowledge, trajectory
 * planning or HMI logic belongs here.
 * ============================================================================
 */

bool ethercat_master_init(
    const EtherCATMasterConfig *config
);

bool ethercat_master_open(void);

int ethercat_master_scan(void);

const EtherCATBusInfo *ethercat_master_info(void);

const char *ethercat_master_slave_name(
    int slave
);

bool ethercat_master_slave_identity(
    int slave,
    EtherCATSlaveIdentity *identity
);

int ethercat_master_map_pdos(void);

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

bool ethercat_master_configure_distributed_clocks(void);

bool ethercat_master_wait_for_safe_op(void);

/*
 * Perform exactly one process-data exchange.
 *
 * Every caller uses this same API so the master can keep one authoritative
 * last-WKC/exchange counter for diagnostics, regardless of the active state.
 */
int ethercat_master_exchange(void);

bool ethercat_master_request_operational(void);

/* Expected Working Counter for group 0. */
int ethercat_master_expected_wkc(void);

/* Result of the most recent exchange performed through this master. */
int ethercat_master_last_wkc(void);

/* Monotonic count of process-data exchanges performed by the master. */
uint64_t ethercat_master_exchange_count(void);

uint8_t *ethercat_master_slave_outputs(
    int slave
);

uint8_t *ethercat_master_slave_inputs(
    int slave
);

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

void ethercat_master_close(void);

/* ============================================================================
 *  PDO BYTE-ORDER HELPERS
 * ============================================================================ */

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
