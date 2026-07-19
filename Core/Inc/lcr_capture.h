#ifndef LCR_CAPTURE_H
#define LCR_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define LCR_CAPTURE_SAMPLE_COUNT 1024U

typedef enum
{
  LCR_CAPTURE_IDLE = 0,
  LCR_CAPTURE_RUNNING,
  LCR_CAPTURE_COMPLETE,
  LCR_CAPTURE_ERROR
} LCR_CaptureState;

typedef enum
{
  LCR_CAPTURE_VOLTAGE_CHANNEL = 0,
  LCR_CAPTURE_CURRENT_CHANNEL
} LCR_CaptureChannel;

typedef struct
{
  uint16_t first;
  uint16_t last;
  uint16_t minimum;
  uint16_t maximum;
  uint16_t mean;
  uint16_t peak_to_peak;
  uint32_t near_rail_count;
} LCR_CaptureChannelSummary;

typedef struct
{
  uint32_t sample_count;
  LCR_CaptureChannelSummary voltage;
  LCR_CaptureChannelSummary current;
} LCR_CaptureSummary;

HAL_StatusTypeDef LCR_CaptureInit(void);
HAL_StatusTypeDef LCR_CaptureStart(void);
HAL_StatusTypeDef LCR_CaptureProcess(LCR_CaptureSummary *summary);
HAL_StatusTypeDef LCR_CaptureAbort(void);
LCR_CaptureState LCR_CaptureGetState(void);
uint32_t LCR_CaptureGetLastError(void);
bool LCR_CaptureGetSample(
  LCR_CaptureChannel channel,
  uint32_t index,
  uint16_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* LCR_CAPTURE_H */
