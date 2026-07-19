#include "lcr_range.h"

#include <stdbool.h>
#include <stddef.h>

#include "main.h"

/* S1 through S4 in the schematic, indexed by ADG1609 A1:A0. */
static const LCR_FeedbackNetwork lcr_feedback_networks[LCR_FEEDBACK_RANGE_COUNT] =
{
  {100000U, 22000U},
  { 10000U,  2200U},
  {  1000U,   220U},
  {   100U,    47U}
};

/*
 * Board diagnostic, 2026-07-19:
 * code00/100 kOhm leaves TIA_OUT near the low rail. Keep the range visible
 * but quarantined, so measurement policy can fall back without lying about
 * the actual feedback network used by impedance and calibration code.
 */
static const bool lcr_feedback_range_available[LCR_FEEDBACK_RANGE_COUNT] =
{
  false,
  true,
  true,
  true
};

static const LCR_FeedbackRange
  lcr_feedback_range_fallback[LCR_FEEDBACK_RANGE_COUNT] =
{
  LCR_FEEDBACK_10K_OHM,
  LCR_FEEDBACK_10K_OHM,
  LCR_FEEDBACK_1K_OHM,
  LCR_FEEDBACK_100_OHM
};

static bool lcr_range_initialized;
static LCR_FeedbackRange lcr_requested_feedback_range =
  LCR_FEEDBACK_100_OHM;
static LCR_FeedbackRange lcr_feedback_range = LCR_FEEDBACK_100_OHM;
static LCR_PgaGain lcr_voltage_gain = LCR_PGA_GAIN_1X;
static LCR_PgaGain lcr_current_gain = LCR_PGA_GAIN_1X;
static bool lcr_feedback_fallback_active;

static void LCR_SetGainAddress(LCR_PgaGain gain)
{
  const uint32_t code = (uint32_t)gain;

  /* These pins are open drain: SET releases to the board's 5 V pull-up. */
  HAL_GPIO_WritePin(
    GAIN_A2_GPIO_Port, GAIN_A2_Pin,
    ((code & 0x4U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(
    GAIN_A1_GPIO_Port, GAIN_A1_Pin,
    ((code & 0x2U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(
    GAIN_A0_GPIO_Port, GAIN_A0_Pin,
    ((code & 0x1U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static HAL_StatusTypeDef LCR_LatchGain(
  LCR_PgaGain gain,
  GPIO_TypeDef *chip_select_port,
  uint16_t chip_select_pin)
{
  if ((uint32_t)gain >= (uint32_t)LCR_PGA_GAIN_COUNT)
  {
    return HAL_ERROR;
  }

  /* Keep the other AD8231 latched while changing the shared address bus. */
  HAL_GPIO_WritePin(VGA_CS1_GPIO_Port, VGA_CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VGA_CS2_GPIO_Port, VGA_CS2_Pin, GPIO_PIN_SET);
  LCR_SetGainAddress(gain);

  /* CS is level-sensitive. One millisecond is a conservative latch pulse. */
  HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);
  return HAL_OK;
}

HAL_StatusTypeDef LCR_RangeInit(void)
{
  HAL_StatusTypeDef status;

  lcr_range_initialized = true;

  /* Release both 5 V pulled-up chip selects before changing the address bus. */
  HAL_GPIO_WritePin(VGA_CS1_GPIO_Port, VGA_CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VGA_CS2_GPIO_Port, VGA_CS2_Pin, GPIO_PIN_SET);

  status = LCR_SetFeedbackRange(LCR_FEEDBACK_100_OHM);
  if (status == HAL_OK)
  {
    status = LCR_SetVoltageGain(LCR_PGA_GAIN_1X);
  }
  if (status == HAL_OK)
  {
    status = LCR_SetCurrentGain(LCR_PGA_GAIN_1X);
  }

  if (status != HAL_OK)
  {
    lcr_range_initialized = false;
  }
  return status;
}

HAL_StatusTypeDef LCR_SetFeedbackRange(LCR_FeedbackRange range)
{
  const uint32_t code = (uint32_t)range;

  if (!lcr_range_initialized || (code >= (uint32_t)LCR_FEEDBACK_RANGE_COUNT))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(
    MUX_A1_GPIO_Port, MUX_A1_Pin,
    ((code & 0x2U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(
    MUX_A0_GPIO_Port, MUX_A0_Pin,
    ((code & 0x1U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcr_requested_feedback_range = range;
  lcr_feedback_range = range;
  lcr_feedback_fallback_active = false;
  return HAL_OK;
}

HAL_StatusTypeDef LCR_SelectFeedbackRange(LCR_FeedbackRange requested_range)
{
  LCR_FeedbackRange actual_range;
  HAL_StatusTypeDef status;

  if (!lcr_range_initialized ||
      ((uint32_t)requested_range >= (uint32_t)LCR_FEEDBACK_RANGE_COUNT))
  {
    return HAL_ERROR;
  }

  actual_range = lcr_feedback_range_available[(uint32_t)requested_range] ?
    requested_range :
    lcr_feedback_range_fallback[(uint32_t)requested_range];
  status = LCR_SetFeedbackRange(actual_range);
  if (status == HAL_OK)
  {
    lcr_requested_feedback_range = requested_range;
    lcr_feedback_fallback_active = actual_range != requested_range;
  }
  return status;
}

HAL_StatusTypeDef LCR_SetVoltageGain(LCR_PgaGain gain)
{
  HAL_StatusTypeDef status;

  if (!lcr_range_initialized)
  {
    return HAL_ERROR;
  }

  /* Schematic U4 (VGA_CS1) measures DUT+SENSED - DUT-SENSED -> V_AMP. */
  status = LCR_LatchGain(gain, VGA_CS1_GPIO_Port, VGA_CS1_Pin);
  if (status == HAL_OK)
  {
    lcr_voltage_gain = gain;
  }
  return status;
}

HAL_StatusTypeDef LCR_SetCurrentGain(LCR_PgaGain gain)
{
  HAL_StatusTypeDef status;

  if (!lcr_range_initialized)
  {
    return HAL_ERROR;
  }

  /* Schematic U5 (VGA_CS2) measures TIA_OUT - DUT-SENSED -> I_AMP. */
  status = LCR_LatchGain(gain, VGA_CS2_GPIO_Port, VGA_CS2_Pin);
  if (status == HAL_OK)
  {
    lcr_current_gain = gain;
  }
  return status;
}

const LCR_FeedbackNetwork *LCR_GetFeedbackNetwork(LCR_FeedbackRange range)
{
  if ((uint32_t)range >= (uint32_t)LCR_FEEDBACK_RANGE_COUNT)
  {
    return NULL;
  }
  return &lcr_feedback_networks[(uint32_t)range];
}

uint32_t LCR_GetPgaGainValue(LCR_PgaGain gain)
{
  if ((uint32_t)gain >= (uint32_t)LCR_PGA_GAIN_COUNT)
  {
    return 0U;
  }
  return 1UL << (uint32_t)gain;
}

bool LCR_IsFeedbackRangeAvailable(LCR_FeedbackRange range)
{
  if ((uint32_t)range >= (uint32_t)LCR_FEEDBACK_RANGE_COUNT)
  {
    return false;
  }
  return lcr_feedback_range_available[(uint32_t)range];
}

void LCR_RangeGetStatus(LCR_RangeStatus *status)
{
  if (status == NULL)
  {
    return;
  }

  status->requested_feedback_range = lcr_requested_feedback_range;
  status->feedback_range = lcr_feedback_range;
  status->voltage_gain = lcr_voltage_gain;
  status->current_gain = lcr_current_gain;
  status->feedback_fallback_active = lcr_feedback_fallback_active;
}
