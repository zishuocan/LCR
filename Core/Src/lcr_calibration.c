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
  int32_t short_resistance_milliohms;
  int32_t short_reactance_milliohms;
  float factor_real;
  float factor_imaginary;
  bool ready;
} LCR_CalibrationPoint;

/*
 * Provisional 93.7 kOhm references measured with a DMM.  The 100 kOhm / 1x
 * point is the feedback-specific coefficient recorded for that hardware
 * combination.  The 10 kOhm / 8x point remains a distinct
 * profile; neither coefficient may be substituted for the other range.
 * Replace both with commercial-LCR references before final acceptance.
 */
static const LCR_CalibrationPoint lcr_calibration_points[] =
{
  {
    10000U,
    LCR_FEEDBACK_100K_OHM,
    LCR_PGA_GAIN_1X,
    LCR_PGA_GAIN_1X,
    0,
    0,
    1.088892f,
    -0.103684f,
    true
  },
  {
    10000U,
    LCR_FEEDBACK_10K_OHM,
    LCR_PGA_GAIN_1X,
    LCR_PGA_GAIN_8X,
    0,
    0,
    0.9831746f,
    0.0049802f,
    true
  }
};

static int32_t LCR_CalibrationRoundToInt32(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static HAL_StatusTypeDef LCR_CalibrationApplyComplexFactor(
  const LCR_ImpedanceResult *raw_result,
  int32_t short_resistance_milliohms,
  int32_t short_reactance_milliohms,
  float factor_real,
  float factor_imaginary,
  LCR_ImpedanceResult *calibrated_result)
{
  const float raw_real =
    (float)(raw_result->resistance_milliohms -
            short_resistance_milliohms) / 1000.0f;
  const float raw_imaginary =
    (float)(raw_result->reactance_milliohms -
            short_reactance_milliohms) / 1000.0f;
  const float calibrated_real =
    (factor_real * raw_real) - (factor_imaginary * raw_imaginary);
  const float calibrated_imaginary =
    (factor_real * raw_imaginary) + (factor_imaginary * raw_real);
  const float calibrated_magnitude = sqrtf(
    (calibrated_real * calibrated_real) +
    (calibrated_imaginary * calibrated_imaginary));
  const float calibrated_phase_degrees =
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

HAL_StatusTypeDef LCR_CalibrationApply(
  const LCR_ImpedanceResult *raw_result,
  uint32_t frequency_hz,
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain,
  LCR_ImpedanceResult *calibrated_result)
{
  const LCR_CalibrationPoint *point = NULL;
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

    if (candidate->ready &&
        (candidate->frequency_hz == frequency_hz) &&
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

  return LCR_CalibrationApplyComplexFactor(
    raw_result,
    point->short_resistance_milliohms,
    point->short_reactance_milliohms,
    point->factor_real,
    point->factor_imaginary,
    calibrated_result);
}
