#include "lcr_excitation.h"

#include "dac.h"
#include "tim.h"

#define LCR_TIM6_MAX_TICKS 65536ULL

/* One signed Q15 sine period. The generated DAC table is centered at midscale. */
static const int16_t lcr_sine_q15[LCR_EXCITATION_TABLE_SIZE] =
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

/* DMA reads half words and writes words to the DAC; keep the source word aligned. */
static uint16_t lcr_dac_waveform[LCR_EXCITATION_TABLE_SIZE]
  __attribute__((aligned(4)));

static bool lcr_initialized;
static bool lcr_running;
static uint32_t lcr_requested_frequency_hz = LCR_EXCITATION_DEFAULT_FREQUENCY_HZ;
static uint32_t lcr_actual_frequency_hz;
static uint32_t lcr_samples_per_period = LCR_EXCITATION_TABLE_SIZE;
static uint16_t lcr_amplitude = LCR_EXCITATION_DEFAULT_AMPLITUDE;

static uint32_t LCR_TIM6_GetInputClockHz(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    timer_clock_hz *= 2U;
  }

  return timer_clock_hz;
}

static void LCR_ExcitationBuildWaveform(void)
{
  uint32_t index;
  const uint32_t table_step =
    LCR_EXCITATION_TABLE_SIZE / lcr_samples_per_period;

  for (index = 0U; index < lcr_samples_per_period; ++index)
  {
    int32_t scaled =
      (int32_t)lcr_sine_q15[index * table_step] *
      (int32_t)lcr_amplitude;

    /* Round symmetrically before converting the Q15 product to DAC counts. */
    scaled += (scaled >= 0) ? 16383 : -16383;
    scaled /= 32767;
    lcr_dac_waveform[index] = (uint16_t)((int32_t)LCR_EXCITATION_DAC_MIDPOINT + scaled);
  }
}

static HAL_StatusTypeDef LCR_ExcitationSelectTableSize(
  uint32_t frequency_hz,
  uint32_t *samples_per_period)
{
  if ((frequency_hz == 0U) || (samples_per_period == NULL))
  {
    return HAL_ERROR;
  }

  if (((uint64_t)frequency_hz * LCR_EXCITATION_TABLE_SIZE) <=
      LCR_EXCITATION_MAX_TRIGGER_RATE_HZ)
  {
    *samples_per_period = LCR_EXCITATION_TABLE_SIZE;
  }
  else if (((uint64_t)frequency_hz * LCR_EXCITATION_MEDIUM_TABLE_SIZE) <=
           LCR_EXCITATION_MAX_TRIGGER_RATE_HZ)
  {
    *samples_per_period = LCR_EXCITATION_MEDIUM_TABLE_SIZE;
  }
  else if (((uint64_t)frequency_hz * LCR_EXCITATION_HIGH_TABLE_SIZE) <=
           LCR_EXCITATION_MAX_TRIGGER_RATE_HZ)
  {
    *samples_per_period = LCR_EXCITATION_HIGH_TABLE_SIZE;
  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef LCR_ExcitationCalculateTimer(
  uint32_t frequency_hz,
  uint32_t samples_per_period,
  uint32_t *period,
  uint32_t *actual_frequency_hz)
{
  const uint32_t timer_clock_hz = LCR_TIM6_GetInputClockHz();
  const uint64_t sample_rate_hz =
    (uint64_t)frequency_hz * (uint64_t)samples_per_period;
  uint64_t timer_ticks;

  if ((frequency_hz == 0U) || (samples_per_period == 0U) ||
      (sample_rate_hz > LCR_EXCITATION_MAX_TRIGGER_RATE_HZ) ||
      (period == NULL) ||
      (actual_frequency_hz == NULL) || (timer_clock_hz == 0U))
  {
    return HAL_ERROR;
  }

  timer_ticks = ((uint64_t)timer_clock_hz + (sample_rate_hz / 2ULL)) /
                sample_rate_hz;
  if ((timer_ticks == 0ULL) || (timer_ticks > LCR_TIM6_MAX_TICKS))
  {
    return HAL_ERROR;
  }

  *period = (uint32_t)(timer_ticks - 1ULL);
  *actual_frequency_hz = (uint32_t)
    ((uint64_t)timer_clock_hz /
     (timer_ticks * (uint64_t)samples_per_period));
  return HAL_OK;
}

static HAL_StatusTypeDef LCR_ExcitationApplyTimer(uint32_t period)
{
  __HAL_TIM_SET_PRESCALER(&htim6, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim6, period);
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  return HAL_TIM_GenerateEvent(&htim6, TIM_EVENTSOURCE_UPDATE);
}

static HAL_StatusTypeDef LCR_ExcitationStopRunning(void)
{
  const HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop(&htim6);
  const HAL_StatusTypeDef dac_status = HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

  lcr_running = false;
  return (timer_status != HAL_OK) ? timer_status : dac_status;
}

HAL_StatusTypeDef LCR_ExcitationInit(void)
{
  uint32_t period;
  uint32_t actual_frequency_hz;
  uint32_t samples_per_period;
  HAL_StatusTypeDef status;

  lcr_initialized = false;
  lcr_running = false;
  lcr_requested_frequency_hz = LCR_EXCITATION_DEFAULT_FREQUENCY_HZ;
  lcr_samples_per_period = LCR_EXCITATION_TABLE_SIZE;
  lcr_amplitude = LCR_EXCITATION_DEFAULT_AMPLITUDE;

  status = LCR_ExcitationSelectTableSize(
    lcr_requested_frequency_hz, &samples_per_period);
  if (status == HAL_OK)
  {
    status = LCR_ExcitationCalculateTimer(
      lcr_requested_frequency_hz,
      samples_per_period,
      &period,
      &actual_frequency_hz);
  }
  if (status != HAL_OK)
  {
    return status;
  }

  lcr_samples_per_period = samples_per_period;
  LCR_ExcitationBuildWaveform();
  status = LCR_ExcitationApplyTimer(period);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_DAC_SetValue(
    &hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, LCR_EXCITATION_DAC_MIDPOINT);
  if (status != HAL_OK)
  {
    return status;
  }

  lcr_actual_frequency_hz = actual_frequency_hz;
  lcr_initialized = true;
  return HAL_OK;
}

HAL_StatusTypeDef LCR_SetFrequency(uint32_t frequency_hz)
{
  uint32_t period;
  uint32_t actual_frequency_hz;
  uint32_t samples_per_period;
  const bool restart = lcr_running;
  HAL_StatusTypeDef status;

  if (!lcr_initialized)
  {
    return HAL_ERROR;
  }

  status = LCR_ExcitationSelectTableSize(
    frequency_hz, &samples_per_period);
  if (status == HAL_OK)
  {
    status = LCR_ExcitationCalculateTimer(
      frequency_hz,
      samples_per_period,
      &period,
      &actual_frequency_hz);
  }
  if (status != HAL_OK)
  {
    return status;
  }

  if (restart)
  {
    status = LCR_ExcitationStopRunning();
    if (status != HAL_OK)
    {
      return status;
    }
  }

  status = LCR_ExcitationApplyTimer(period);
  if (status != HAL_OK)
  {
    return status;
  }
  if (samples_per_period != lcr_samples_per_period)
  {
    lcr_samples_per_period = samples_per_period;
    LCR_ExcitationBuildWaveform();
  }
  lcr_requested_frequency_hz = frequency_hz;
  lcr_actual_frequency_hz = actual_frequency_hz;

  return restart ? LCR_ExcitationStart() : HAL_OK;
}

HAL_StatusTypeDef LCR_SetAmplitude(uint16_t amplitude)
{
  const bool restart = lcr_running;
  HAL_StatusTypeDef status;

  if (!lcr_initialized || (amplitude > LCR_EXCITATION_MAX_AMPLITUDE))
  {
    return HAL_ERROR;
  }

  if (restart)
  {
    status = LCR_ExcitationStopRunning();
    if (status != HAL_OK)
    {
      return status;
    }
  }

  lcr_amplitude = amplitude;
  LCR_ExcitationBuildWaveform();
  return restart ? LCR_ExcitationStart() : HAL_OK;
}

HAL_StatusTypeDef LCR_ExcitationStart(void)
{
  HAL_StatusTypeDef status;

  if (!lcr_initialized)
  {
    return HAL_ERROR;
  }
  if (lcr_running)
  {
    return HAL_OK;
  }

  status = HAL_DAC_SetValue(
    &hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, LCR_EXCITATION_DAC_MIDPOINT);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_DAC_Start_DMA(
    &hdac1,
    DAC_CHANNEL_1,
    (const uint32_t *)(const void *)lcr_dac_waveform,
    lcr_samples_per_period,
    DAC_ALIGN_12B_R);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_TIM_Base_Start(&htim6);
  if (status != HAL_OK)
  {
    (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    return status;
  }

  lcr_running = true;
  return HAL_OK;
}

HAL_StatusTypeDef LCR_ExcitationStop(void)
{
  if (!lcr_initialized)
  {
    return HAL_ERROR;
  }
  if (!lcr_running)
  {
    return HAL_OK;
  }

  return LCR_ExcitationStopRunning();
}

void LCR_ExcitationGetStatus(LCR_ExcitationStatus *status)
{
  if (status == NULL)
  {
    return;
  }

  status->requested_frequency_hz = lcr_requested_frequency_hz;
  status->actual_frequency_hz = lcr_actual_frequency_hz;
  status->samples_per_period = lcr_samples_per_period;
  status->amplitude = lcr_amplitude;
  status->running = lcr_running;
}
