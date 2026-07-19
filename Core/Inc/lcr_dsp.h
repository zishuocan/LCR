#ifndef LCR_DSP_H
#define LCR_DSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define LCR_DSP_MAX_HARMONIC 5U

typedef struct
{
  int32_t real_milli_counts;
  int32_t imaginary_milli_counts;
  uint32_t magnitude_milli_counts;
  int32_t phase_millidegrees;
} LCR_DspBin;

typedef struct
{
  LCR_DspBin bin[LCR_DSP_MAX_HARMONIC + 1U];
  uint32_t harmonic_ratio_millipercent[LCR_DSP_MAX_HARMONIC + 1U];
  uint32_t thd_millipercent;
  bool fundamental_valid;
} LCR_DspChannelResult;

typedef struct
{
  LCR_DspChannelResult voltage;
  LCR_DspChannelResult current;
  int32_t voltage_minus_current_millidegrees;
} LCR_DspCaptureResult;

HAL_StatusTypeDef LCR_DspAnalyzeCapture(
  uint32_t samples_per_period,
  LCR_DspCaptureResult *result);

#ifdef __cplusplus
}
#endif

#endif /* LCR_DSP_H */
