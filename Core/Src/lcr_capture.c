#include "lcr_capture.h"

#include <stddef.h>

#include "adc.h"

#define LCR_CAPTURE_ADC_MAX_CODE         4095U
#define LCR_CAPTURE_NEAR_RAIL_MARGIN     16U

/*
 * ADC12 common CDR packs the master ADC1 sample into bits 15:0 and the
 * slave ADC2 sample into bits 31:16, as defined by RDATA_MST/RDATA_SLV.
 */
static uint32_t lcr_adc_dual_buffer[LCR_CAPTURE_SAMPLE_COUNT]
  __attribute__((aligned(4)));

static volatile LCR_CaptureState lcr_capture_state = LCR_CAPTURE_IDLE;
static volatile uint32_t lcr_capture_last_error;
static bool lcr_capture_data_valid;

static HAL_StatusTypeDef LCR_CaptureStopAdcs(void)
{
  const HAL_StatusTypeDef master_status =
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
  const HAL_StatusTypeDef slave_status = HAL_ADC_Stop(&hadc2);

  return (master_status != HAL_OK) ? master_status : slave_status;
}

static uint16_t LCR_CaptureGetVoltageSample(uint32_t packed_sample)
{
  return (uint16_t)((packed_sample & ADC_CDR_RDATA_MST_Msk) >>
                    ADC_CDR_RDATA_MST_Pos);
}

static uint16_t LCR_CaptureGetCurrentSample(uint32_t packed_sample)
{
  return (uint16_t)((packed_sample & ADC_CDR_RDATA_SLV_Msk) >>
                    ADC_CDR_RDATA_SLV_Pos);
}

static void LCR_CaptureAnalyzeChannel(
  LCR_CaptureChannelSummary *channel,
  bool voltage_channel)
{
  uint64_t sum = 0ULL;
  uint32_t index;

  channel->minimum = LCR_CAPTURE_ADC_MAX_CODE;
  channel->maximum = 0U;
  channel->near_rail_count = 0U;

  for (index = 0U; index < LCR_CAPTURE_SAMPLE_COUNT; ++index)
  {
    const uint16_t sample = voltage_channel ?
      LCR_CaptureGetVoltageSample(lcr_adc_dual_buffer[index]) :
      LCR_CaptureGetCurrentSample(lcr_adc_dual_buffer[index]);

    if (sample < channel->minimum)
    {
      channel->minimum = sample;
    }
    if (sample > channel->maximum)
    {
      channel->maximum = sample;
    }
    if ((sample <= LCR_CAPTURE_NEAR_RAIL_MARGIN) ||
        (sample >= (LCR_CAPTURE_ADC_MAX_CODE - LCR_CAPTURE_NEAR_RAIL_MARGIN)))
    {
      ++channel->near_rail_count;
    }
    sum += sample;
  }

  channel->first = voltage_channel ?
    LCR_CaptureGetVoltageSample(lcr_adc_dual_buffer[0]) :
    LCR_CaptureGetCurrentSample(lcr_adc_dual_buffer[0]);
  channel->last = voltage_channel ?
    LCR_CaptureGetVoltageSample(lcr_adc_dual_buffer[LCR_CAPTURE_SAMPLE_COUNT - 1U]) :
    LCR_CaptureGetCurrentSample(lcr_adc_dual_buffer[LCR_CAPTURE_SAMPLE_COUNT - 1U]);
  channel->mean = (uint16_t)
    ((sum + (LCR_CAPTURE_SAMPLE_COUNT / 2U)) / LCR_CAPTURE_SAMPLE_COUNT);
  channel->peak_to_peak = channel->maximum - channel->minimum;
}

HAL_StatusTypeDef LCR_CaptureInit(void)
{
  HAL_StatusTypeDef status;

  lcr_capture_state = LCR_CAPTURE_IDLE;
  lcr_capture_last_error = HAL_ADC_ERROR_NONE;
  lcr_capture_data_valid = false;

  status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if (status == HAL_OK)
  {
    status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  }
  if (status != HAL_OK)
  {
    lcr_capture_last_error = hadc1.ErrorCode | hadc2.ErrorCode;
    lcr_capture_state = LCR_CAPTURE_ERROR;
  }

  return status;
}

HAL_StatusTypeDef LCR_CaptureStart(void)
{
  HAL_StatusTypeDef status;

  if (lcr_capture_state != LCR_CAPTURE_IDLE)
  {
    return HAL_BUSY;
  }

  lcr_capture_last_error = HAL_ADC_ERROR_NONE;
  lcr_capture_data_valid = false;
  lcr_capture_state = LCR_CAPTURE_RUNNING;
  status = HAL_ADC_Start(&hadc2);
  if (status == HAL_OK)
  {
    status = HAL_ADCEx_MultiModeStart_DMA(
      &hadc1,
      lcr_adc_dual_buffer,
      LCR_CAPTURE_SAMPLE_COUNT);
  }

  if (status != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc2);
    lcr_capture_last_error = hadc1.ErrorCode | hadc2.ErrorCode;
    lcr_capture_state = LCR_CAPTURE_ERROR;
  }

  return status;
}

HAL_StatusTypeDef LCR_CaptureProcess(LCR_CaptureSummary *summary)
{
  const LCR_CaptureState completed_state = lcr_capture_state;
  HAL_StatusTypeDef stop_status;

  if (summary == NULL)
  {
    return HAL_ERROR;
  }
  if ((completed_state != LCR_CAPTURE_COMPLETE) &&
      (completed_state != LCR_CAPTURE_ERROR))
  {
    return HAL_BUSY;
  }

  stop_status = LCR_CaptureStopAdcs();
  if (completed_state == LCR_CAPTURE_ERROR)
  {
    lcr_capture_state = LCR_CAPTURE_IDLE;
    return HAL_ERROR;
  }
  if (stop_status != HAL_OK)
  {
    lcr_capture_last_error |= hadc1.ErrorCode;
    lcr_capture_state = LCR_CAPTURE_IDLE;
    return stop_status;
  }

  summary->sample_count = LCR_CAPTURE_SAMPLE_COUNT;
  LCR_CaptureAnalyzeChannel(&summary->voltage, true);
  LCR_CaptureAnalyzeChannel(&summary->current, false);
  lcr_capture_data_valid = true;
  lcr_capture_state = LCR_CAPTURE_IDLE;
  return HAL_OK;
}

HAL_StatusTypeDef LCR_CaptureAbort(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (lcr_capture_state != LCR_CAPTURE_IDLE)
  {
    status = LCR_CaptureStopAdcs();
    lcr_capture_state = LCR_CAPTURE_IDLE;
  }

  return status;
}

LCR_CaptureState LCR_CaptureGetState(void)
{
  return lcr_capture_state;
}

uint32_t LCR_CaptureGetLastError(void)
{
  return lcr_capture_last_error;
}

bool LCR_CaptureGetSample(
  LCR_CaptureChannel channel,
  uint32_t index,
  uint16_t *sample)
{
  if (!lcr_capture_data_valid ||
      (index >= LCR_CAPTURE_SAMPLE_COUNT) ||
      (sample == NULL))
  {
    return false;
  }

  if (channel == LCR_CAPTURE_VOLTAGE_CHANNEL)
  {
    *sample = LCR_CaptureGetVoltageSample(lcr_adc_dual_buffer[index]);
  }
  else if (channel == LCR_CAPTURE_CURRENT_CHANNEL)
  {
    *sample = LCR_CaptureGetCurrentSample(lcr_adc_dual_buffer[index]);
  }
  else
  {
    return false;
  }

  return true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) &&
      (lcr_capture_state == LCR_CAPTURE_RUNNING))
  {
    lcr_capture_state = LCR_CAPTURE_COMPLETE;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if ((hadc->Instance == ADC1) &&
      (lcr_capture_state == LCR_CAPTURE_RUNNING))
  {
    lcr_capture_last_error = hadc->ErrorCode;
    lcr_capture_state = LCR_CAPTURE_ERROR;
  }
}
