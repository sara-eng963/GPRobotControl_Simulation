#ifndef APPLICATION_VALIDATION_SERVICE_H
#define APPLICATION_VALIDATION_SERVICE_H
#include <stdbool.h>
/* One M4-only worker for durable validation storage operations. Returns false
 * when board NOR storage is absent; the HMI IPC task must still run. */
bool application_validation_service_start(void);
#endif
