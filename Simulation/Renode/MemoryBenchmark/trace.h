/* Test-image-only bounded instrumentation. No allocation, I/O or RTOS calls.
 * Included at the end of a generated FreeRTOSConfig.h; never in production. */
void membench_create(void *, void *, unsigned, const char *, unsigned);
void membench_switch(void *);
void membench_delete(void *);
void membench_malloc(void *);
void membench_free(void);
#define traceTASK_SWITCHED_IN() membench_switch(pxCurrentTCB)
#define traceTASK_DELETE(p) membench_delete(p)
#define traceMALLOC(p, n) membench_malloc(p)
#define traceFREE(p, n) membench_free()
