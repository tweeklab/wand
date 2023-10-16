#ifndef __DETECTOR_H
#define __DETECTOR_H

#define POINT_DATA_BUFLEN 2048

#ifdef __cplusplus
extern "C" {
#endif

void start_detector_tasks(void);
void set_detector_point_stream_buffer(MessageBufferHandle_t);
void set_detector_video_stream_buffer(MessageBufferHandle_t);

#ifdef __cplusplus
};
#endif

#endif // __DETECTOR_H