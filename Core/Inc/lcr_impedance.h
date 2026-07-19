#ifndef LCR_IMPEDANCE_H
#define LCR_IMPEDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_dsp.h"
#include "lcr_range.h"
#include "stm32g4xx_hal.h"

typedef struct
{
  int32_t resistance_milliohms;
  int32_t reactance_milliohms;
  uint32_t magnitude_milliohms;
  int32_t phase_millidegrees;
  bool valid;
} LCR_ImpedanceResult;

HAL_StatusTypeDef LCR_ImpedanceCalculateRaw(
  const LCR_DspCaptureResult *dsp_result,
  uint32_t frequency_hz,
  const LCR_FeedbackNetwork *feedback,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain,
  LCR_ImpedanceResult *result);

#ifdef __cplusplus
}
#endif

#endif /* LCR_IMPEDANCE_H */
