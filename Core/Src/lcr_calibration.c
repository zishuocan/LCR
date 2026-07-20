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

typedef struct
{
  uint32_t raw_capacitance_picofarads;
  uint32_t reference_capacitance_picofarads;
} LCR_CapacitanceCalibrationPoint;

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

/*
 * Provisional 10 kHz series-mode capacitance transfer curve, 2026-07-20.
 * References are the component nominal markings supplied during development,
 * not yet commercial-LCR traceable values.  Keep the measured raw points and
 * their references separate so the same interpolation can be retained when
 * the references are replaced by same-frequency commercial-LCR readings.
 *
 * This curve is intentionally capacitance-specific and is only selected for
 * a strongly capacitive raw impedance.  It therefore cannot alter resistor or
 * inductor results.  Linear interpolation preserves monotonicity and avoids a
 * single global multiplier that would move alternating range/gain errors in
 * the wrong direction.
 */
static const LCR_CapacitanceCalibrationPoint
  lcr_capacitance_calibration_points[] =
{
  {    888U,    1000U},
  {  10371U,   10000U},
  {  88512U,  100000U},
  { 228651U,  220000U},
  { 448817U,  470000U},
  { 871175U, 1000000U},
  {4223000U, 4700000U}
};

static int32_t LCR_CalibrationRoundToInt32(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static HAL_StatusTypeDef LCR_CalibrationApplyComplexFactor(
  const LCR_ImpedanceResult *raw_result,
  float factor_real,
  float factor_imaginary,
  LCR_ImpedanceResult *calibrated_result)
{
  const float raw_real =
    (float)raw_result->resistance_milliohms / 1000.0f;
  const float raw_imaginary =
    (float)raw_result->reactance_milliohms / 1000.0f;
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

static HAL_StatusTypeDef LCR_CalibrationApplyCapacitanceCurve(
  const LCR_ImpedanceResult *raw_result,
  uint32_t frequency_hz,
  LCR_ImpedanceResult *calibrated_result)
{
  const uint32_t point_count =
    sizeof(lcr_capacitance_calibration_points) /
    sizeof(lcr_capacitance_calibration_points[0]);
  double raw_capacitance_picofarads;
  double reference_capacitance_picofarads;
  double scale;
  uint32_t index;

  if ((frequency_hz != 10000U) ||
      (raw_result->reactance_milliohms >= 0) ||
      (raw_result->phase_millidegrees > -45000L) ||
      (point_count < 2U))
  {
    return HAL_ERROR;
  }

  raw_capacitance_picofarads = 1.0e15 /
    (2.0 * (double)LCR_CALIBRATION_PI * (double)frequency_hz *
     (double)(-(int64_t)raw_result->reactance_milliohms));
  if (raw_capacitance_picofarads <= 0.0)
  {
    return HAL_ERROR;
  }

  if (raw_capacitance_picofarads <=
      (double)lcr_capacitance_calibration_points[0]
        .raw_capacitance_picofarads)
  {
    reference_capacitance_picofarads = raw_capacitance_picofarads *
      (double)lcr_capacitance_calibration_points[0]
        .reference_capacitance_picofarads /
      (double)lcr_capacitance_calibration_points[0]
        .raw_capacitance_picofarads;
  }
  else if (raw_capacitance_picofarads >=
           (double)lcr_capacitance_calibration_points[point_count - 1U]
             .raw_capacitance_picofarads)
  {
    reference_capacitance_picofarads = raw_capacitance_picofarads *
      (double)lcr_capacitance_calibration_points[point_count - 1U]
        .reference_capacitance_picofarads /
      (double)lcr_capacitance_calibration_points[point_count - 1U]
        .raw_capacitance_picofarads;
  }
  else
  {
    reference_capacitance_picofarads = 0.0;
    for (index = 0U; index < (point_count - 1U); ++index)
    {
      const LCR_CapacitanceCalibrationPoint *lower =
        &lcr_capacitance_calibration_points[index];
      const LCR_CapacitanceCalibrationPoint *upper =
        &lcr_capacitance_calibration_points[index + 1U];

      if (raw_capacitance_picofarads <=
          (double)upper->raw_capacitance_picofarads)
      {
        const double position =
          (raw_capacitance_picofarads -
           (double)lower->raw_capacitance_picofarads) /
          ((double)upper->raw_capacitance_picofarads -
           (double)lower->raw_capacitance_picofarads);

        reference_capacitance_picofarads =
          (double)lower->reference_capacitance_picofarads +
          position *
          ((double)upper->reference_capacitance_picofarads -
           (double)lower->reference_capacitance_picofarads);
        break;
      }
    }
  }

  if (reference_capacitance_picofarads <= 0.0)
  {
    return HAL_ERROR;
  }

  /* Z is inversely proportional to C; scale Z so its derived C is reference. */
  scale = raw_capacitance_picofarads /
          reference_capacitance_picofarads;
  return LCR_CalibrationApplyComplexFactor(
    raw_result,
    (float)scale,
    0.0f,
    calibrated_result);
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

  if (LCR_CalibrationApplyCapacitanceCurve(
        raw_result,
        frequency_hz,
        calibrated_result) == HAL_OK)
  {
    return HAL_OK;
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

  return LCR_CalibrationApplyComplexFactor(
    raw_result,
    point->factor_real,
    point->factor_imaginary,
    calibrated_result);
}
