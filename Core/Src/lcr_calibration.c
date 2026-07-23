#include "lcr_calibration.h"

#include <math.h>
#include <stddef.h>

#define LCR_CALIBRATION_PI 3.14159265358979323846f
#define LCR_INDUCTANCE_CALIBRATION_FREQUENCY_HZ 25000U
#define LCR_INDUCTANCE_CALIBRATION_MIN_PHASE_MDEG 10000L
#define LCR_INDUCTANCE_CALIBRATION_SLOPE 1.04507631028907
#define LCR_INDUCTANCE_CALIBRATION_OFFSET_NH 6362.20767440943

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
  },
  /*
   * Provisional 47 uH nominal reference at 50 kHz, series mode.
   * Raw average: 3.024 + j11.690 ohm (37.210 uH).
   * Target keeps measured Rs and sets X=2*pi*50k*47uH.
   * Valid only for this exact feedback/PGA profile; replace after short
   * compensation and same-frequency commercial-LCR characterization.
   */
  {
    50000U,
    LCR_FEEDBACK_100_OHM,
    LCR_PGA_GAIN_16X,
    LCR_PGA_GAIN_2X,
    0,
    0,
    1.2465861f,
    0.06378755f,
    true
  },
  /*
   * Provisional 100 uH nominal reference at 50 kHz, series mode.
   * Raw average: 5.268 + j26.832 ohm (85.409 uH).
   * Exact-profile only; keep separate from the 47 uH V16x point.
   */
  {
    50000U,
    LCR_FEEDBACK_100_OHM,
    LCR_PGA_GAIN_8X,
    LCR_PGA_GAIN_2X,
    0,
    0,
    1.1644973f,
    0.03229620f,
    true
  },
  /*
   * Provisional 22 uH nominal reference at 50 kHz, series mode.
   * Raw average: 2.273 + j5.343 ohm (17.007 uH).
   * Exact-profile only; this V32x path is distinct from V8x and V16x.
   */
  {
    50000U,
    LCR_FEEDBACK_100_OHM,
    LCR_PGA_GAIN_32X,
    LCR_PGA_GAIN_2X,
    0,
    0,
    1.2485754f,
    0.10574807f,
    true
  },
  /* Reserved profile: populate Zshort and K from a 50 kHz short/resistor run. */
  {
    50000U,
    LCR_FEEDBACK_100_OHM,
    LCR_PGA_GAIN_64X,
    LCR_PGA_GAIN_2X,
    0,
    0,
    1.0f,
    0.0f,
    false
  },
  /* Fifth profile for a low-inductance DUT whose I2x channel clips. */
  {
    50000U,
    LCR_FEEDBACK_100_OHM,
    LCR_PGA_GAIN_64X,
    LCR_PGA_GAIN_1X,
    0,
    0,
    1.0f,
    0.0f,
    false
  }
};

/*
 * A completed 50 kHz profile is an affine complex calibration:
 *
 *   Zcorrected = (Zraw - Zshort) * K
 *
 * Zshort comes from a DUT-terminal four-wire short.  K comes from a known
 * precision resistor measured at the same frequency and hardware profile:
 *
 *   K = Rreference / (Zresistor_raw - Zshort)
 *
 * The three ready 50 kHz entries above retain the earlier inductor-derived
 * factors with a zero placeholder short only to preserve current bring-up
 * behaviour.  Replace each pair with real short/resistor data before claiming
 * unknown-impedance accuracy.  The two V64x entries deliberately remain
 * unavailable until that data exists.
 */

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

/*
 * Provisional two-point series-inductance calibration at 25 kHz.
 * The user currently treats the nominal markings as the references:
 *
 *   raw 38.885 uH -> reference 47 uH
 *   raw 89.599 uH -> reference 100 uH
 *
 * The fitted transfer is Lcal = 1.04507631028907 * Lraw + 6.362207674 uH.
 * It is deliberately restricted to clearly inductive impedance at the same
 * frequency and cannot alter resistor, capacitor, 1 kHz, or 10 kHz results.
 * Replace these nominal references with same-frequency commercial-LCR values
 * before final accuracy acceptance.
 */
static HAL_StatusTypeDef LCR_CalibrationApplyInductanceTransfer(
  const LCR_ImpedanceResult *raw_result,
  uint32_t frequency_hz,
  LCR_ImpedanceResult *calibrated_result)
{
  double raw_inductance_nanohenries;
  double calibrated_inductance_nanohenries;
  double calibrated_reactance_milliohms;
  double resistance_milliohms;
  double magnitude_milliohms;
  double phase_degrees;

  if ((frequency_hz != LCR_INDUCTANCE_CALIBRATION_FREQUENCY_HZ) ||
      (raw_result->reactance_milliohms <= 0) ||
      (raw_result->phase_millidegrees <
       LCR_INDUCTANCE_CALIBRATION_MIN_PHASE_MDEG))
  {
    return HAL_ERROR;
  }

  raw_inductance_nanohenries =
    ((double)raw_result->reactance_milliohms * 1.0e6) /
    (2.0 * (double)LCR_CALIBRATION_PI * (double)frequency_hz);
  calibrated_inductance_nanohenries =
    (LCR_INDUCTANCE_CALIBRATION_SLOPE * raw_inductance_nanohenries) +
    LCR_INDUCTANCE_CALIBRATION_OFFSET_NH;
  if (calibrated_inductance_nanohenries <= 0.0)
  {
    return HAL_ERROR;
  }

  calibrated_reactance_milliohms =
    (2.0 * (double)LCR_CALIBRATION_PI * (double)frequency_hz *
     calibrated_inductance_nanohenries) / 1.0e6;
  resistance_milliohms = (double)raw_result->resistance_milliohms;
  magnitude_milliohms = sqrt(
    (resistance_milliohms * resistance_milliohms) +
    (calibrated_reactance_milliohms * calibrated_reactance_milliohms));
  phase_degrees = atan2(
    calibrated_reactance_milliohms,
    resistance_milliohms) * (180.0 / (double)LCR_CALIBRATION_PI);

  calibrated_result->resistance_milliohms =
    raw_result->resistance_milliohms;
  calibrated_result->reactance_milliohms =
    LCR_CalibrationRoundToInt32((float)calibrated_reactance_milliohms);
  calibrated_result->magnitude_milliohms = (uint32_t)
    LCR_CalibrationRoundToInt32((float)magnitude_milliohms);
  calibrated_result->phase_millidegrees =
    LCR_CalibrationRoundToInt32((float)(phase_degrees * 1000.0));
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

  if (LCR_CalibrationApplyInductanceTransfer(
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

bool LCR_CalibrationIsInductorProfileSupported(
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain)
{
  if (feedback_range != LCR_FEEDBACK_100_OHM)
  {
    return false;
  }

  if (current_gain == LCR_PGA_GAIN_2X)
  {
    return (voltage_gain == LCR_PGA_GAIN_8X) ||
           (voltage_gain == LCR_PGA_GAIN_16X) ||
           (voltage_gain == LCR_PGA_GAIN_32X) ||
           (voltage_gain == LCR_PGA_GAIN_64X);
  }

  return (current_gain == LCR_PGA_GAIN_1X) &&
         (voltage_gain == LCR_PGA_GAIN_64X);
}
