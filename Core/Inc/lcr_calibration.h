#ifndef LCR_CALIBRATION_H
#define LCR_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lcr_impedance.h"
#include "lcr_range.h"
#include "stm32g4xx_hal.h"

HAL_StatusTypeDef LCR_CalibrationApply(
  const LCR_ImpedanceResult *raw_result,
  uint32_t frequency_hz,
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain,
  LCR_ImpedanceResult *calibrated_result);

#ifdef __cplusplus
}
#endif

#endif /* LCR_CALIBRATION_H */
