#include "lcr_dsp.h"

#include <math.h>
#include <stddef.h>

#include "lcr_capture.h"

#define LCR_DSP_Q15_SCALE                    32767.0f
#define LCR_DSP_FULL_CIRCLE_MILLIDEGREES     360000L
#define LCR_DSP_HALF_CIRCLE_MILLIDEGREES     180000L
#define LCR_DSP_MIN_FUNDAMENTAL_MILLI_COUNTS 5000U
#define LCR_DSP_PI                           3.14159265358979323846f
#define LCR_DSP_TABLE_SIZE                   64U

static const int16_t lcr_dsp_sine_q15[LCR_DSP_TABLE_SIZE] =
{
       0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
   23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
   32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
   23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
       0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
  -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
  -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
  -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212
};

static int32_t LCR_DspRoundToInt32(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t LCR_DspNormalizePhase(int32_t phase_millidegrees)
{
  while (phase_millidegrees > LCR_DSP_HALF_CIRCLE_MILLIDEGREES)
  {
    phase_millidegrees -= LCR_DSP_FULL_CIRCLE_MILLIDEGREES;
  }
  while (phase_millidegrees <= -LCR_DSP_HALF_CIRCLE_MILLIDEGREES)
  {
    phase_millidegrees += LCR_DSP_FULL_CIRCLE_MILLIDEGREES;
  }
  return phase_millidegrees;
}

static HAL_StatusTypeDef LCR_DspAnalyzeChannel(
  LCR_CaptureChannel channel,
  uint32_t samples_per_period,
  LCR_DspChannelResult *result)
{
  float harmonic_power = 0.0f;
  uint32_t harmonic;

  for (harmonic = 1U; harmonic <= LCR_DSP_MAX_HARMONIC; ++harmonic)
  {
    int64_t real_accumulator = 0LL;
    int64_t imaginary_accumulator = 0LL;
    uint32_t index;
    float real_counts;
    float imaginary_counts;
    float magnitude_counts;
    float phase_degrees;

    for (index = 0U; index < LCR_CAPTURE_SAMPLE_COUNT; ++index)
    {
      uint16_t sample;
      const uint32_t table_step =
        LCR_DSP_TABLE_SIZE / samples_per_period;
      const uint32_t phase_index =
        (harmonic * (index % samples_per_period) * table_step) %
        LCR_DSP_TABLE_SIZE;
      const uint32_t cosine_index =
        (phase_index + (LCR_DSP_TABLE_SIZE / 4U)) % LCR_DSP_TABLE_SIZE;

      if (!LCR_CaptureGetSample(channel, index, &sample))
      {
        return HAL_ERROR;
      }

      real_accumulator +=
        (int64_t)sample * (int64_t)lcr_dsp_sine_q15[cosine_index];
      imaginary_accumulator -=
        (int64_t)sample * (int64_t)lcr_dsp_sine_q15[phase_index];
    }

    real_counts =
      (2.0f * (float)real_accumulator) /
      ((float)LCR_CAPTURE_SAMPLE_COUNT * LCR_DSP_Q15_SCALE);
    imaginary_counts =
      (2.0f * (float)imaginary_accumulator) /
      ((float)LCR_CAPTURE_SAMPLE_COUNT * LCR_DSP_Q15_SCALE);
    magnitude_counts = sqrtf(
      (real_counts * real_counts) +
      (imaginary_counts * imaginary_counts));
    phase_degrees = atan2f(imaginary_counts, real_counts) *
                    (180.0f / LCR_DSP_PI);

    result->bin[harmonic].real_milli_counts =
      LCR_DspRoundToInt32(real_counts * 1000.0f);
    result->bin[harmonic].imaginary_milli_counts =
      LCR_DspRoundToInt32(imaginary_counts * 1000.0f);
    result->bin[harmonic].magnitude_milli_counts =
      (uint32_t)LCR_DspRoundToInt32(magnitude_counts * 1000.0f);
    result->bin[harmonic].phase_millidegrees =
      LCR_DspNormalizePhase(LCR_DspRoundToInt32(phase_degrees * 1000.0f));
  }

  result->fundamental_valid =
    result->bin[1].magnitude_milli_counts >=
    LCR_DSP_MIN_FUNDAMENTAL_MILLI_COUNTS;
  result->harmonic_ratio_millipercent[0] = 0U;
  result->harmonic_ratio_millipercent[1] = 100000U;

  for (harmonic = 2U; harmonic <= LCR_DSP_MAX_HARMONIC; ++harmonic)
  {
    const float magnitude =
      (float)result->bin[harmonic].magnitude_milli_counts;

    harmonic_power += magnitude * magnitude;
    if (result->fundamental_valid)
    {
      result->harmonic_ratio_millipercent[harmonic] = (uint32_t)
        LCR_DspRoundToInt32(
          (magnitude * 100000.0f) /
          (float)result->bin[1].magnitude_milli_counts);
    }
    else
    {
      result->harmonic_ratio_millipercent[harmonic] = 0U;
    }
  }

  if (result->fundamental_valid)
  {
    result->thd_millipercent = (uint32_t)LCR_DspRoundToInt32(
      (sqrtf(harmonic_power) * 100000.0f) /
      (float)result->bin[1].magnitude_milli_counts);
  }
  else
  {
    result->thd_millipercent = 0U;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LCR_DspAnalyzeCapture(
  uint32_t samples_per_period,
  LCR_DspCaptureResult *result)
{
  HAL_StatusTypeDef status;

  if ((result == NULL) ||
      ((samples_per_period != 16U) &&
       (samples_per_period != 32U) &&
       (samples_per_period != LCR_DSP_TABLE_SIZE)) ||
      ((LCR_CAPTURE_SAMPLE_COUNT % samples_per_period) != 0U))
  {
    return HAL_ERROR;
  }

  status = LCR_DspAnalyzeChannel(
    LCR_CAPTURE_VOLTAGE_CHANNEL,
    samples_per_period,
    &result->voltage);
  if (status == HAL_OK)
  {
    status = LCR_DspAnalyzeChannel(
      LCR_CAPTURE_CURRENT_CHANNEL,
      samples_per_period,
      &result->current);
  }
  if (status == HAL_OK)
  {
    result->voltage_minus_current_millidegrees = LCR_DspNormalizePhase(
      result->voltage.bin[1].phase_millidegrees -
      result->current.bin[1].phase_millidegrees);
  }

  return status;
}
