#include "lcr_impedance.h"

#include <math.h>
#include <stddef.h>

#define LCR_IMPEDANCE_PI 3.14159265358979323846f

static int32_t LCR_ImpedanceRoundToInt32(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

HAL_StatusTypeDef LCR_ImpedanceCalculateRaw(
  const LCR_DspCaptureResult *dsp_result,
  uint32_t frequency_hz,
  const LCR_FeedbackNetwork *feedback,
  LCR_PgaGain voltage_gain,
  LCR_PgaGain current_gain,
  LCR_ImpedanceResult *result)
{
  float voltage_real;
  float voltage_imaginary;
  float current_real;
  float current_imaginary;
  float current_power;
  float ratio_real;
  float ratio_imaginary;
  float feedback_capacitance_f;
  float omega_rc;
  float feedback_denominator;
  float feedback_real;
  float feedback_imaginary;
  float gain_ratio;
  float impedance_real;
  float impedance_imaginary;
  float impedance_magnitude;
  float impedance_phase_degrees;
  uint32_t voltage_gain_value;
  uint32_t current_gain_value;

  if ((dsp_result == NULL) || (feedback == NULL) || (result == NULL) ||
      (frequency_hz == 0U) || !dsp_result->current.fundamental_valid)
  {
    return HAL_ERROR;
  }

  voltage_gain_value = LCR_GetPgaGainValue(voltage_gain);
  current_gain_value = LCR_GetPgaGainValue(current_gain);
  if ((voltage_gain_value == 0U) || (current_gain_value == 0U) ||
      (feedback->resistance_ohms == 0U))
  {
    return HAL_ERROR;
  }

  voltage_real = (float)dsp_result->voltage.bin[1].real_milli_counts;
  voltage_imaginary =
    (float)dsp_result->voltage.bin[1].imaginary_milli_counts;
  current_real = (float)dsp_result->current.bin[1].real_milli_counts;
  current_imaginary =
    (float)dsp_result->current.bin[1].imaginary_milli_counts;
  current_power =
    (current_real * current_real) +
    (current_imaginary * current_imaginary);
  if (current_power <= 0.0f)
  {
    return HAL_ERROR;
  }

  /* Complex division V/I. Both inputs use milli-counts, so scale cancels. */
  ratio_real =
    ((voltage_real * current_real) +
     (voltage_imaginary * current_imaginary)) /
    current_power;
  ratio_imaginary =
    ((voltage_imaginary * current_real) -
     (voltage_real * current_imaginary)) /
    current_power;

  feedback_capacitance_f =
    (float)feedback->capacitance_tenths_pf * 1.0e-13f;
  omega_rc =
    2.0f * LCR_IMPEDANCE_PI * (float)frequency_hz *
    (float)feedback->resistance_ohms * feedback_capacitance_f;
  feedback_denominator = 1.0f + (omega_rc * omega_rc);
  feedback_real =
    (float)feedback->resistance_ohms / feedback_denominator;
  feedback_imaginary = -
    ((float)feedback->resistance_ohms * omega_rc) /
    feedback_denominator;

  gain_ratio = (float)current_gain_value / (float)voltage_gain_value;

  /*
   * I_AMP = -Gi * I_DUT * Zf and V_AMP = Gv * V_DUT, therefore:
   * Z_DUT = -(Gi/Gv) * Zf * (V_AMP/I_AMP).
   */
  impedance_real = -gain_ratio *
    ((feedback_real * ratio_real) -
     (feedback_imaginary * ratio_imaginary));
  impedance_imaginary = -gain_ratio *
    ((feedback_real * ratio_imaginary) +
     (feedback_imaginary * ratio_real));
  impedance_magnitude = sqrtf(
    (impedance_real * impedance_real) +
    (impedance_imaginary * impedance_imaginary));
  impedance_phase_degrees =
    atan2f(impedance_imaginary, impedance_real) *
    (180.0f / LCR_IMPEDANCE_PI);

  result->resistance_milliohms =
    LCR_ImpedanceRoundToInt32(impedance_real * 1000.0f);
  result->reactance_milliohms =
    LCR_ImpedanceRoundToInt32(impedance_imaginary * 1000.0f);
  result->magnitude_milliohms = (uint32_t)
    LCR_ImpedanceRoundToInt32(impedance_magnitude * 1000.0f);
  result->phase_millidegrees =
    LCR_ImpedanceRoundToInt32(impedance_phase_degrees * 1000.0f);
  result->valid = true;
  return HAL_OK;
}
