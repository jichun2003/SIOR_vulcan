/* Telechips scaler_api.h의 최소 가짜 선언 (PC 시험용 스텁). 실제 보드에서는 SDK 헤더 사용. */

#ifndef AIG_STUB_SCALER_API_H
#define AIG_STUB_SCALER_API_H

#include <stdint.h>

#define SCALER_DEV_NAME_0 "/dev/scaler1"

typedef enum { SCALER_INDEX_0 = 0, SCALER_INDEX_1, SCALER_INDEX_MAX } scaler_index_t;

typedef enum {
    SCALER_FORMAT_ARGB8888 = 12,
    SCALER_FORMAT_RGB888   = 14
} scaler_format_t;

typedef struct {
    uint64_t pmap;
    uint32_t width;
    uint32_t height;
    scaler_format_t format;
} scaler_size_info_t;

typedef void *ScalerHandle;

int32_t ScalerCreate(ScalerHandle *pHandle);
int32_t ScalerDestroy(ScalerHandle handle);
int32_t ScalerOpenDevice(ScalerHandle handle, const char *scalerDevName, scaler_index_t scalerIndex);
int32_t ScalerCloseDevice(ScalerHandle handle, scaler_index_t scalerIndex);
int32_t ScalerResize(ScalerHandle handle, scaler_index_t scalerIndex,
                     scaler_size_info_t scalerSrc, scaler_size_info_t scalerDes);
int32_t ScalerCropResize(ScalerHandle handle, scaler_index_t scalerIndex,
                         scaler_size_info_t scalerSrc, scaler_size_info_t scalerDes,
                         uint32_t cropXMin, uint32_t cropYMin, uint32_t cropXMax, uint32_t cropYMax);
int32_t ScalerPoll(ScalerHandle handle, scaler_index_t scalerIndex);
int32_t ScalerGetStatus(ScalerHandle handle, scaler_index_t scalerIndex);

#endif /* AIG_STUB_SCALER_API_H */
