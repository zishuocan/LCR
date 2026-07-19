#include "lcr_calibration.h"

#include <math.h>
#include <stddef.h>

#define LCR_CALIBRATION_PI 3.14159265358979323846f

typedef struct
{
  uint32_t frequency_hz;
  LCR_FeedbackRange feedback_range;
  LCR_PgaGain voltage_gain;
  LCR_PgaGain current_gain;
  float factor_real;
  float factor_imaginary;
} LCR_CalibrationPoint;

/*
 * Provisional reference: 93.7 kOhm measured with a DMM.
 * Mean of eight valid captures: 95301.076 - j482.744 Ohm at 10 kHz.
 * Replace this point with a commercial LCR reference before final acceptance.
 */
static const LCR_CalibrationPoint lcr_calibration_points[] =
{
  {
    10000U,
    LCR_FEEDBACK_10K_OHM,
    LCR_PGA_GAIN_1X,
    LCR_PGA_GAIN_8X,
    0.9831746f,
    0.0049802f
  }
};

static int32_t LCR_CalibrationRoundToInt32(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

HAL_StatusTypeDef LCR_CalibrationApply(
  const LCR_ImpedanceResult *raw_result,
  uint32_t frequency_hz,
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain,
  LCR_ImpedanceResult *calibrated_result)
{
  const LCR_CalibrationPoint *point = NULL;
  float raw_real;
  float raw_imaginary;
  float calibrated_real;
  float calibrated_imaginary;
  float calibrated_magnitude;
  float calibrated_phase_degrees;
  uint32_t index;

  if ((raw_result == NULL) || (calibrated_result == NULL) ||
      !raw_result->valid)
  {
    return HAL_ERROR;
  }

  for (index = 0U;
       index < (sizeof(lcr_calibration_points) /
                sizeof(lcr_calibration_points[0]));
       ++index)
  {
    const LCR_CalibrationPoint *candidate = &lcr_calibration_points[index];

    if ((candidate->frequency_hz == frequency_hz) &&
        (candidate->feedback_range == feedback_range) &&
        (candidate->voltage_gain == voltage_gain) &&
        (candidate->current_gain == current_gain))
    {
      point = candidate;
      break;
    }
  }

  if (point == NULL)
  {
    return HAL_ERROR;
  }

  raw_real = (float)raw_result->resistance_milliohms / 1000.0f;
  raw_imaginary = (float)raw_result->reactance_milliohms / 1000.0f;
  calibrated_real =
    (point->factor_real * raw_real) -
    (point->factor_imaginary * raw_imaginary);
  calibrated_imaginary =
    (point->factor_real * raw_imaginary) +
    (point->factor_imaginary * raw_real);
  calibrated_magnitude = sqrtf(
    (calibrated_real * calibrated_real) +
    (calibrated_imaginary * calibrated_imaginary));
  calibrated_phase_degrees =
    atan2f(calibrated_imaginary, calibrated_real) *
    (180.0f / LCR_CALIBRATION_PI);

  calibrated_result->resistance_milliohms =
    LCR_CalibrationRoundToInt32(calibrated_real * 1000.0f);
  calibrated_result->reactance_milliohms =
    LCR_CalibrationRoundToInt32(calibrated_imaginary * 1000.0f);
  calibrated_result->magnitude_milliohms = (uint32_t)
    LCR_CalibrationRoundToInt32(calibrated_magnitude * 1000.0f);
  calibrated_result->phase_millidegrees =
    LCR_CalibrationRoundToInt32(calibrated_phase_degrees * 1000.0f);
  calibrated_result->valid = true;
  return HAL_OK;
}
