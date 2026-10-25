#ifndef APPLICATION_VALIDATION_SERVICE_H
#define APPLICATION_VALIDATION_SERVICE_H
#include <stdbool.h>
#include "validation_storage_bridge.h"
#include "trajectory_storage_service.h"

/* M4-only durable storage worker. Missing providers leave worker_ready false;
 * the separate IPC/HMI service must continue running. Call once at startup. */
bool application_validation_service_start(void);

/* Implement BOTH providers against the SAME physical committed artifact.
 * Strong board definitions are optional; no fake default implementation. */
bool board_m4_supply_validation_backend(PathValidationStorage *out);
bool board_m4_supply_trajectory_reader(CommittedTrajectoryReader *out);
#endif
