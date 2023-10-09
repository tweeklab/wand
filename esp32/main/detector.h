#ifndef __DETECTOR_H
#define __DETECTOR_H

#include "freertos/message_buffer.h"

#define STATUS_DATA_BUFLEN 2048

#ifdef __cplusplus
extern "C" {
#endif

void start_detector_tasks(MessageBufferHandle_t msgbuf);
void enable_status_stream(void);
void disable_status_stream(void);

#ifdef __cplusplus
};
#endif

#endif // __DETECTOR_H