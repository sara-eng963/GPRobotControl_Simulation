#ifndef ALIGNMENT_MESSAGE_BUFFER_H
#define ALIGNMENT_MESSAGE_BUFFER_H
/* Compile-only type seam. Real IPC ABI is verified in ARM and A02 checks. */
typedef void *MessageBufferHandle_t;
typedef struct { unsigned unused; } StaticMessageBuffer_t;
#endif
