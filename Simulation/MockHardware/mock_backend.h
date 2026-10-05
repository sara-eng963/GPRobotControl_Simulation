#ifndef MOCK_BACKEND_H
#define MOCK_BACKEND_H
#include <stdbool.h>
#include <stdint.h>
/* PC plant boundary: no state completion or validation results are synthesized. */
void mock_backend_reset(void);
void mock_backend_set_joint(unsigned joint, double radians);
double mock_backend_get_joint(unsigned joint);
void mock_backend_hold(void);
void mock_backend_set_fault(bool active);
void mock_backend_set_communication_failure(bool active);
bool mock_backend_ready(void);
#endif
