/* Telechips camera_api.h의 최소 가짜 선언 (PC 시험용 스텁). 실제 보드에서는 SDK 헤더 사용. */

#ifndef AIG_STUB_CAMERA_API_H
#define AIG_STUB_CAMERA_API_H

#include <stdint.h>

#define CAMERA_DEV_NAME_0 "/dev/video0"
#define CAMERA_DEV_NAME_2 "/dev/video2"

typedef enum { CAMERA_STATUS_IDLE = 0, CAMERA_STATUS_OPENED, CAMERA_STATUS_STREAMING } cam_status_t;
typedef enum { CAMERA_FORMAT_RGB32 = 0, CAMERA_FORMAT_RGB24 } cam_format_t;
typedef void *CameraHandle;

int32_t CameraCreate(CameraHandle *pHandle);
int32_t CameraDestroy(CameraHandle handle);
int32_t CameraOpenDevice(CameraHandle handle, char *inputDevName);
int32_t CameraCloseDevice(CameraHandle handle);
int32_t CameraSetConfig(CameraHandle handle, uint32_t width, uint32_t height);
int32_t CameraGetBuffer(CameraHandle handle, uint8_t **virtualAddr, uint64_t *baseOffset);
int32_t CameraReleaseBuffer(CameraHandle handle);
cam_status_t CameraGetStatus(CameraHandle handle);

#endif /* AIG_STUB_CAMERA_API_H */
