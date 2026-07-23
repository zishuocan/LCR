/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>

#include "lcr_autorange.h"
#include "lcr_100k_diagnostic.h"
#include "lcr_calibration.h"
#include "lcr_capture.h"
#include "lcr_component.h"
#include "lcr_display.h"
#include "lcr_dsp.h"
#include "lcr_excitation.h"
#include "lcr_impedance.h"
#include "lcr_measurement.h"
#include "lcr_range.h"
#include "lcr_result.h"
#include "lcr_workflow.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCR_CAPTURE_SETTLING_TIME_MS 10U
#define LCR_INTER_CAPTURE_TIME_MS 2U
#define LCR_PRIMARY_FREQUENCY_HZ 10000U
#define LCR_AMBIGUITY_FREQUENCY_HZ 1000U
#define LCR_INDUCTOR_FREQUENCY_HZ 50000U
#define LCR_INDUCTOR_SUSPECT_MIN_PHASE_MDEG 3000L
#define LCR_INDUCTOR_SUSPECT_MIN_X_MOHM 200L
#define LCR_INDUCTOR_SUSPECT_MAX_Z_MOHM 100000UL
#define LCR_VOLTAGE_THD_LIMIT_MILLIPERCENT 10000U
#define LCR_CURRENT_THD_LIMIT_MILLIPERCENT 10000U
#define LCR_CAPACITIVE_THD_MARGIN_MILLIPERCENT 3000U
#define LCR_CAPACITIVE_THD_MAX_MILLIPERCENT 30000U
#define LCR_CAPACITIVE_PHASE_MIN_MDEG 45000L
#define LCR_CAPACITIVE_PHASE_MAX_MDEG 135000L
#define LCR_INDUCTIVE_THD_MARGIN_MILLIPERCENT 3000U
#define LCR_INDUCTIVE_THD_MAX_MILLIPERCENT 30000U
#define LCR_INDUCTIVE_PHASE_MIN_MDEG (-170000L)
#define LCR_INDUCTIVE_PHASE_MAX_MDEG (-90000L)

#define LCR_QUALITY_VOLTAGE_RAIL       (1UL << 0)
#define LCR_QUALITY_CURRENT_RAIL       (1UL << 1)
#define LCR_QUALITY_VOLTAGE_NO_H1      (1UL << 2)
#define LCR_QUALITY_CURRENT_NO_H1      (1UL << 3)
#define LCR_QUALITY_VOLTAGE_THD_HIGH   (1UL << 4)
#define LCR_QUALITY_CURRENT_THD_HIGH   (1UL << 5)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
typedef enum
{
  LCR_SECONDARY_NONE = 0,
  LCR_SECONDARY_AMBIGUITY_1KHZ,
  LCR_SECONDARY_INDUCTOR_50KHZ
} LCR_SecondaryMode;

static volatile bool lcr_button_event = false;
static uint32_t lcr_last_button_tick;
static bool lcr_capture_scheduled;
static uint32_t lcr_capture_due_tick;
static uint32_t lcr_capture_deadline_tick;
static LCR_AutorangeSession lcr_autorange_session;
static LCR_MeasurementSession lcr_measurement_session;
static LCR_SecondaryMode lcr_secondary_mode;
static LCR_ImpedanceResult lcr_primary_impedance;
static uint32_t lcr_primary_frequency_hz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t LCR_EvaluateCaptureQuality(
  const LCR_CaptureSummary *summary,
  const LCR_DspCaptureResult *dsp_result,
  uint32_t *voltage_thd_limit_millipercent,
  uint32_t *current_thd_limit_millipercent);
static void LCR_PrintCaptureQuality(
  uint32_t quality_flags,
  const LCR_CaptureSummary *summary,
  const LCR_DspCaptureResult *dsp_result,
  uint32_t voltage_thd_limit_millipercent,
  uint32_t current_thd_limit_millipercent,
  bool autorange_probe);
static void LCR_PrintAutorangeReasons(uint32_t reason_flags);
static void LCR_PrintAutorangeSelection(
  const LCR_AutorangeEvaluation *evaluation);
static void LCR_PrintComponentResult(
  const LCR_ComponentResult *component,
  const char *source);
static void LCR_PrintUint64(uint64_t value);
static void LCR_ScheduleNextCaptureOrFinish(void);
static uint32_t LCR_GetCaptureTimeoutMs(void);
static void LCR_FailWorkflow(LCR_WorkflowError error);
static void LCR_PrintFinalResultSnapshot(void);
static void LCR_UpdateDisplayFromFinalResult(void);
static void LCR_UpdateDisplayMeasuring(uint32_t frequency_hz);
static bool LCR_IsSuspectedInductor(
  const LCR_ImpedanceResult *impedance);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t LCR_GetCaptureTimeoutMs(void)
{
  LCR_ExcitationStatus excitation_status;
  uint64_t sample_rate_hz;
  uint64_t capture_time_ms;
  uint64_t timeout_ms;

  LCR_ExcitationGetStatus(&excitation_status);
  sample_rate_hz =
    (uint64_t)excitation_status.actual_frequency_hz *
    excitation_status.samples_per_period;
  if (sample_rate_hz == 0ULL)
  {
    return 1000U;
  }

  capture_time_ms =
    ((1000ULL * LCR_CAPTURE_SAMPLE_COUNT) + sample_rate_hz - 1ULL) /
    sample_rate_hz;
  timeout_ms = (capture_time_ms * 4ULL) + 20ULL;
  if (timeout_ms < 50ULL)
  {
    timeout_ms = 50ULL;
  }
  else if (timeout_ms > 1000ULL)
  {
    timeout_ms = 1000ULL;
  }
  return (uint32_t)timeout_ms;
}

static bool LCR_IsSuspectedInductor(
  const LCR_ImpedanceResult *impedance)
{
  return (impedance != NULL) && impedance->valid &&
         (impedance->reactance_milliohms >=
          LCR_INDUCTOR_SUSPECT_MIN_X_MOHM) &&
         (impedance->phase_millidegrees >=
          LCR_INDUCTOR_SUSPECT_MIN_PHASE_MDEG) &&
         (impedance->magnitude_milliohms <=
          LCR_INDUCTOR_SUSPECT_MAX_Z_MOHM);
}

static void LCR_FailWorkflow(LCR_WorkflowError error)
{
  LCR_MeasurementSummary measurement_summary;
  LCR_ExcitationStatus excitation_status;
  LCR_WorkflowStatus workflow_status;
  const uint32_t autorange_probe_count = lcr_autorange_session.probe_count;
  const HAL_StatusTypeDef capture_abort_status = LCR_CaptureAbort();
  const HAL_StatusTypeDef excitation_stop_status = LCR_ExcitationStop();

  LCR_MeasurementGetSummary(&lcr_measurement_session, &measurement_summary);
  LCR_ExcitationGetStatus(&excitation_status);
  LCR_ResultStoreMeasurement(
    LCR_WorkflowIsConfirmation(),
    excitation_status.actual_frequency_hz,
    autorange_probe_count,
    &measurement_summary);
  LCR_ResultFail(error);
  lcr_capture_scheduled = false;
  lcr_capture_deadline_tick = 0U;
  LCR_MeasurementCancel(&lcr_measurement_session);
  LCR_AutorangeCancel(&lcr_autorange_session);
  lcr_secondary_mode = LCR_SECONDARY_NONE;
  BSP_LED_Off(LED_GREEN);
  LCR_WorkflowFail(error);
  LCR_WorkflowGetStatus(&workflow_status);
  printf("[LCR] measurement state=%s; code=%s; capture_abort_HAL=%u; excitation_stop_HAL=%u\r\n",
         LCR_WorkflowStateName(workflow_status.state),
         LCR_WorkflowErrorName(workflow_status.last_error),
         (unsigned int)capture_abort_status,
         (unsigned int)excitation_stop_status);
  printf("[LCR] terminal context retained: completed_attempts=%lu eligible=%lu ineligible=%lu autorange_probes=%lu; any incomplete DMA capture is explicitly excluded\r\n",
         (unsigned long)measurement_summary.attempt_count,
         (unsigned long)measurement_summary.eligible_count,
         (unsigned long)measurement_summary.ineligible_count,
         (unsigned long)autorange_probe_count);
  LCR_PrintFinalResultSnapshot();
  LCR_UpdateDisplayFromFinalResult();
}

static void LCR_PrintFinalResultSnapshot(void)
{
  LCR_FinalResult result;

  LCR_ResultGet(&result);
  printf("[LCR] result snapshot: seq=%lu state=%s error=%s primary=%s secondary=%s component=%s\r\n",
         (unsigned long)result.sequence_id,
         LCR_ResultStateName(result.state),
         LCR_WorkflowErrorName(result.error),
         result.primary.present ? "yes" : "no",
         result.secondary.present ? "yes" : "no",
         result.component_valid ? "yes" : "no");
}

static void LCR_UpdateDisplayFromFinalResult(void)
{
  LCR_FinalResult result;
  HAL_StatusTypeDef display_status;

  if (!LCR_DisplayIsReady())
  {
    return;
  }

  LCR_ResultGet(&result);
  display_status = LCR_DisplayShowResult(&result);
  if (display_status != HAL_OK)
  {
    printf("[LCR] OLED final-screen update failed: HAL=%u\r\n",
           (unsigned int)display_status);
  }
}

static void LCR_UpdateDisplayMeasuring(uint32_t frequency_hz)
{
  HAL_StatusTypeDef display_status;

  if (!LCR_DisplayIsReady())
  {
    return;
  }

  display_status = LCR_DisplayShowMeasuring(frequency_hz);
  if (display_status != HAL_OK)
  {
    printf("[LCR] OLED measuring-screen update failed: HAL=%u\r\n",
           (unsigned int)display_status);
  }
}

static uint32_t LCR_EvaluateCaptureQuality(
  const LCR_CaptureSummary *summary,
  const LCR_DspCaptureResult *dsp_result,
  uint32_t *voltage_thd_limit_millipercent,
  uint32_t *current_thd_limit_millipercent)
{
  uint32_t flags = 0U;
  uint32_t voltage_thd_limit = LCR_VOLTAGE_THD_LIMIT_MILLIPERCENT;
  uint32_t current_thd_limit = LCR_CURRENT_THD_LIMIT_MILLIPERCENT;
  LCR_RangeStatus range_status;

  LCR_RangeGetStatus(&range_status);

  /*
   * A capacitor differentiates the excitation waveform.  Its nth current
   * harmonic is therefore expected to be about n times the corresponding
   * voltage harmonic.  A fixed resistor-oriented I-THD limit would reject a
   * physically consistent capacitive waveform (observed at 10 nF: V THD
   * about 7.5%, I THD about 17.5%).  Derive a bounded allowance only when the
   * measured V/I phase is clearly capacitive; resistive and inductive samples
   * retain the original 10% limit, preserving rejection of bad resistor data.
   */
  if (dsp_result->voltage.fundamental_valid &&
      dsp_result->current.fundamental_valid &&
      (dsp_result->voltage_minus_current_millidegrees >=
       LCR_CAPACITIVE_PHASE_MIN_MDEG) &&
      (dsp_result->voltage_minus_current_millidegrees <=
       LCR_CAPACITIVE_PHASE_MAX_MDEG))
  {
    float expected_harmonic_power = 0.0f;
    uint32_t harmonic;

    for (harmonic = 2U; harmonic <= LCR_DSP_MAX_HARMONIC; ++harmonic)
    {
      const float expected_ratio =
        (float)harmonic *
        (float)dsp_result->voltage.harmonic_ratio_millipercent[harmonic];

      expected_harmonic_power += expected_ratio * expected_ratio;
    }
    {
      const uint32_t expected_thd =
        (uint32_t)(sqrtf(expected_harmonic_power) + 0.5f);
      uint32_t derived_limit = expected_thd +
        LCR_CAPACITIVE_THD_MARGIN_MILLIPERCENT;

      if (derived_limit < LCR_CURRENT_THD_LIMIT_MILLIPERCENT)
      {
        derived_limit = LCR_CURRENT_THD_LIMIT_MILLIPERCENT;
      }
      if (derived_limit > LCR_CAPACITIVE_THD_MAX_MILLIPERCENT)
      {
        derived_limit = LCR_CAPACITIVE_THD_MAX_MILLIPERCENT;
      }
      current_thd_limit = derived_limit;
    }
  }

  /*
   * An inductor differentiates current into terminal voltage.  On the
   * preferred low-impedance 100 ohm/1 kOhm feedback profiles, the nth voltage harmonic is
   * therefore expected to be about n times the corresponding current
   * harmonic.  Apply the symmetric bounded allowance only when the raw
   * bridge phase is clearly inductive; this keeps resistor profiles on the
   * original 10% V-THD limit.
   */
  if (((range_status.feedback_range == LCR_FEEDBACK_100_OHM) ||
       (range_status.feedback_range == LCR_FEEDBACK_1K_OHM)) &&
      dsp_result->voltage.fundamental_valid &&
      dsp_result->current.fundamental_valid &&
      (dsp_result->voltage_minus_current_millidegrees >=
       LCR_INDUCTIVE_PHASE_MIN_MDEG) &&
      (dsp_result->voltage_minus_current_millidegrees <=
       LCR_INDUCTIVE_PHASE_MAX_MDEG))
  {
    float expected_harmonic_power = 0.0f;
    uint32_t harmonic;

    for (harmonic = 2U; harmonic <= LCR_DSP_MAX_HARMONIC; ++harmonic)
    {
      const float expected_ratio =
        (float)harmonic *
        (float)dsp_result->current.harmonic_ratio_millipercent[harmonic];

      expected_harmonic_power += expected_ratio * expected_ratio;
    }
    {
      const uint32_t expected_thd =
        (uint32_t)(sqrtf(expected_harmonic_power) + 0.5f);
      uint32_t derived_limit = expected_thd +
        LCR_INDUCTIVE_THD_MARGIN_MILLIPERCENT;

      if (derived_limit < LCR_VOLTAGE_THD_LIMIT_MILLIPERCENT)
      {
        derived_limit = LCR_VOLTAGE_THD_LIMIT_MILLIPERCENT;
      }
      if (derived_limit > LCR_INDUCTIVE_THD_MAX_MILLIPERCENT)
      {
        derived_limit = LCR_INDUCTIVE_THD_MAX_MILLIPERCENT;
      }
      voltage_thd_limit = derived_limit;
    }
  }

  if (voltage_thd_limit_millipercent != NULL)
  {
    *voltage_thd_limit_millipercent = voltage_thd_limit;
  }

  if (current_thd_limit_millipercent != NULL)
  {
    *current_thd_limit_millipercent = current_thd_limit;
  }

  if (summary->voltage.near_rail_count != 0U)
  {
    flags |= LCR_QUALITY_VOLTAGE_RAIL;
  }
  if (summary->current.near_rail_count != 0U)
  {
    flags |= LCR_QUALITY_CURRENT_RAIL;
  }
  if (!dsp_result->voltage.fundamental_valid)
  {
    flags |= LCR_QUALITY_VOLTAGE_NO_H1;
  }
  if (!dsp_result->current.fundamental_valid)
  {
    flags |= LCR_QUALITY_CURRENT_NO_H1;
  }
  if (dsp_result->voltage.fundamental_valid &&
      (dsp_result->voltage.thd_millipercent >
       voltage_thd_limit))
  {
    flags |= LCR_QUALITY_VOLTAGE_THD_HIGH;
  }
  if (dsp_result->current.fundamental_valid &&
      (dsp_result->current.thd_millipercent >
       current_thd_limit))
  {
    flags |= LCR_QUALITY_CURRENT_THD_HIGH;
  }

  return flags;
}

static void LCR_PrintCaptureQuality(
  uint32_t quality_flags,
  const LCR_CaptureSummary *summary,
  const LCR_DspCaptureResult *dsp_result,
  uint32_t voltage_thd_limit_millipercent,
  uint32_t current_thd_limit_millipercent,
  bool autorange_probe)
{
  if (quality_flags == 0U)
  {
    if (autorange_probe)
    {
      printf("[LCR] sample quality=VALID; retained=yes; eligible_for_average=no; reason=AUTORANGE_PROBE\r\n");
    }
    else
    {
      printf("[LCR] sample quality=VALID; retained=yes; eligible_for_average=yes\r\n");
    }
    if (current_thd_limit_millipercent >
        LCR_CURRENT_THD_LIMIT_MILLIPERCENT)
    {
      printf("[LCR] quality model=CAPACITIVE_HARMONIC; I_THD limit=%lu.%03lu%% derived from V harmonics plus margin\r\n",
             (unsigned long)(current_thd_limit_millipercent / 1000U),
             (unsigned long)(current_thd_limit_millipercent % 1000U));
    }
    if (voltage_thd_limit_millipercent >
        LCR_VOLTAGE_THD_LIMIT_MILLIPERCENT)
    {
      printf("[LCR] quality model=INDUCTIVE_HARMONIC; V_THD limit=%lu.%03lu%% derived from I harmonics plus margin\r\n",
             (unsigned long)(voltage_thd_limit_millipercent / 1000U),
             (unsigned long)(voltage_thd_limit_millipercent % 1000U));
    }
    return;
  }

  printf("[LCR] sample quality=SUSPECT; retained=yes; eligible_for_average=no\r\n");
  printf("[LCR] quality reasons:");
  if ((quality_flags & LCR_QUALITY_VOLTAGE_RAIL) != 0U)
  {
    printf(" V_RAIL(samples=%lu)",
           (unsigned long)summary->voltage.near_rail_count);
  }
  if ((quality_flags & LCR_QUALITY_CURRENT_RAIL) != 0U)
  {
    printf(" I_RAIL(samples=%lu)",
           (unsigned long)summary->current.near_rail_count);
  }
  if ((quality_flags & LCR_QUALITY_VOLTAGE_NO_H1) != 0U)
  {
    printf(" V_H1_INVALID");
  }
  if ((quality_flags & LCR_QUALITY_CURRENT_NO_H1) != 0U)
  {
    printf(" I_H1_INVALID");
  }
  if ((quality_flags & LCR_QUALITY_VOLTAGE_THD_HIGH) != 0U)
  {
    printf(" V_THD=%lu.%03lu%%>%lu.%03lu%%",
           (unsigned long)(dsp_result->voltage.thd_millipercent / 1000U),
           (unsigned long)(dsp_result->voltage.thd_millipercent % 1000U),
           (unsigned long)(voltage_thd_limit_millipercent / 1000U),
           (unsigned long)(voltage_thd_limit_millipercent % 1000U));
  }
  if ((quality_flags & LCR_QUALITY_CURRENT_THD_HIGH) != 0U)
  {
    printf(" I_THD=%lu.%03lu%%>%lu.%03lu%%",
           (unsigned long)(dsp_result->current.thd_millipercent / 1000U),
           (unsigned long)(dsp_result->current.thd_millipercent % 1000U),
           (unsigned long)(current_thd_limit_millipercent / 1000U),
           (unsigned long)(current_thd_limit_millipercent % 1000U));
  }
  printf("\r\n");
}

static void LCR_PrintAutorangeReasons(uint32_t reason_flags)
{
  printf("[LCR] autorange reasons:");
  if ((reason_flags & LCR_AUTORANGE_REASON_V_LOW) != 0U)
  {
    printf(" V_TOO_LOW");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_V_HIGH) != 0U)
  {
    printf(" V_TOO_HIGH");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_I_LOW) != 0U)
  {
    printf(" I_TOO_LOW");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_I_HIGH) != 0U)
  {
    printf(" I_TOO_HIGH");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_V_LIMIT) != 0U)
  {
    printf(" V_RANGE_LIMIT");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_I_LIMIT) != 0U)
  {
    printf(" I_RANGE_LIMIT_OR_OPEN");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_APPLY_FAILED) != 0U)
  {
    printf(" RANGE_APPLY_FAILED");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_MAX_ADJUSTMENT) != 0U)
  {
    printf(" MAX_ADJUSTMENTS");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_V_THD_HIGH) != 0U)
  {
    printf(" V_THD_HIGH");
  }
  if ((reason_flags & LCR_AUTORANGE_REASON_I_THD_HIGH) != 0U)
  {
    printf(" I_THD_HIGH");
  }
  if (reason_flags == 0U)
  {
    printf(" SIGNALS_IN_WINDOW");
  }
  printf("\r\n");
}

static void LCR_PrintAutorangeSelection(
  const LCR_AutorangeEvaluation *evaluation)
{
  const LCR_FeedbackNetwork *previous_feedback = LCR_GetFeedbackNetwork(
    evaluation->previous_range.feedback_range);
  const LCR_FeedbackNetwork *selected_feedback = LCR_GetFeedbackNetwork(
    evaluation->selected_range.feedback_range);

  if ((previous_feedback == NULL) || (selected_feedback == NULL))
  {
    return;
  }

  printf("[LCR] autorange selection: feedback %lu->%lu ohm, V gain %lu->%lu, I gain %lu->%lu, amplitude %u->%u\r\n",
         (unsigned long)previous_feedback->resistance_ohms,
         (unsigned long)selected_feedback->resistance_ohms,
         (unsigned long)LCR_GetPgaGainValue(
           evaluation->previous_range.voltage_gain),
         (unsigned long)LCR_GetPgaGainValue(
           evaluation->selected_range.voltage_gain),
         (unsigned long)LCR_GetPgaGainValue(
           evaluation->previous_range.current_gain),
         (unsigned long)LCR_GetPgaGainValue(
           evaluation->selected_range.current_gain),
         (unsigned int)evaluation->previous_amplitude,
         (unsigned int)evaluation->selected_amplitude);
}

static void LCR_PrintComponentResult(
  const LCR_ComponentResult *component,
  const char *source)
{
  if ((component == NULL) || (source == NULL))
  {
    return;
  }

  if (component->type == LCR_COMPONENT_RESISTOR)
  {
    printf("[LCR] component result: type=R, R=%ld.%03lu ohm, f=%lu Hz, source=%s\r\n",
           (long)(component->series_resistance_milliohms / 1000L),
           (unsigned long)labs(
             component->series_resistance_milliohms % 1000L),
           (unsigned long)component->parameter_frequency_hz,
           source);
  }
  else if (component->type == LCR_COMPONENT_CAPACITOR)
  {
    printf("[LCR] component result: type=C, C=");
    LCR_PrintUint64(component->capacitance_picofarads);
    printf(" pF, ESR=%ld.%03lu ohm, f=%lu Hz, source=%s\r\n",
           (long)(component->series_resistance_milliohms / 1000L),
           (unsigned long)labs(
             component->series_resistance_milliohms % 1000L),
           (unsigned long)component->parameter_frequency_hz,
           source);
  }
  else if (component->type == LCR_COMPONENT_INDUCTOR)
  {
    printf("[LCR] component result: type=L, L=");
    LCR_PrintUint64(component->inductance_nanohenries);
    printf(" nH, series_R=%ld.%03lu ohm, f=%lu Hz, source=%s\r\n",
           (long)(component->series_resistance_milliohms / 1000L),
           (unsigned long)labs(
             component->series_resistance_milliohms % 1000L),
           (unsigned long)component->parameter_frequency_hz,
           source);
  }
  else
  {
    printf("[LCR] component result: type=UNKNOWN, source=%s; ambiguity remains\r\n",
           source);
  }
}

static void LCR_PrintUint64(uint64_t value)
{
  char decimal[21];
  uint32_t index = sizeof(decimal) - 1U;

  decimal[index] = '\0';
  do
  {
    decimal[--index] = (char)('0' + (value % 10ULL));
    value /= 10ULL;
  }
  while ((value != 0ULL) && (index != 0U));
  printf("%s", &decimal[index]);
}

static void LCR_ScheduleNextCaptureOrFinish(void)
{
  if (LCR_MeasurementIsComplete(&lcr_measurement_session))
  {
    LCR_MeasurementSummary measurement_summary;
    const HAL_StatusTypeDef stop_status = LCR_ExcitationStop();
    bool confirmation_started = false;

    LCR_MeasurementGetSummary(
      &lcr_measurement_session,
      &measurement_summary);
    lcr_capture_scheduled = false;
    BSP_LED_Off(LED_GREEN);
    printf("[LCR] measurement summary: attempts=%lu eligible=%lu ineligible=%lu retained=%lu outliers=%lu\r\n",
           (unsigned long)measurement_summary.attempt_count,
           (unsigned long)measurement_summary.eligible_count,
           (unsigned long)measurement_summary.ineligible_count,
           (unsigned long)measurement_summary.retained_count,
           (unsigned long)measurement_summary.outlier_count);
    if (stop_status != HAL_OK)
    {
      printf("[LCR] excitation stop failed at measurement boundary: HAL=%u\r\n",
             (unsigned int)stop_status);
      LCR_FailWorkflow(LCR_WORKFLOW_ERROR_EXCITATION_CONTROL);
      return;
    }
    if (!measurement_summary.average_valid)
    {
      printf("[LCR] average unavailable: fewer than %u valid samples in %u attempts, or too few samples remained after outlier rejection\r\n",
             (unsigned int)LCR_MEASUREMENT_MIN_VALID_COUNT,
             (unsigned int)LCR_MEASUREMENT_MAX_ATTEMPT_COUNT);
      if (lcr_secondary_mode != LCR_SECONDARY_NONE)
      {
        printf("[LCR] multi-frequency confirmation unavailable because secondary average is invalid\r\n");
        LCR_FailWorkflow(LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID);
      }
      else
      {
        LCR_FailWorkflow(LCR_WORKFLOW_ERROR_NO_ELIGIBLE_SAMPLES);
      }
      return;
    }
    else
    {
      LCR_ExcitationStatus excitation_status;
      LCR_ComponentResult component;

      printf("[LCR] average %s Z: R=%ld.%03lu ohm, X=%ld.%03lu ohm, |Z|=%lu.%03lu ohm, phase=%ld.%03lu deg\r\n",
             measurement_summary.average_is_calibrated ?
               "CALIBRATED" : "RAW/UNCALIBRATED",
             (long)(measurement_summary.average.resistance_milliohms / 1000L),
             (unsigned long)labs(
               measurement_summary.average.resistance_milliohms % 1000L),
             (long)(measurement_summary.average.reactance_milliohms / 1000L),
             (unsigned long)labs(
               measurement_summary.average.reactance_milliohms % 1000L),
             (unsigned long)(measurement_summary.average.magnitude_milliohms / 1000U),
             (unsigned long)(measurement_summary.average.magnitude_milliohms % 1000U),
              (long)(measurement_summary.average.phase_millidegrees / 1000L),
              (unsigned long)labs(
                measurement_summary.average.phase_millidegrees % 1000L));
      LCR_ExcitationGetStatus(&excitation_status);
      LCR_ResultStoreMeasurement(
        lcr_secondary_mode != LCR_SECONDARY_NONE,
        excitation_status.actual_frequency_hz,
        lcr_autorange_session.probe_count,
        &measurement_summary);
      if (LCR_ComponentClassify(
            &measurement_summary.average,
            excitation_status.actual_frequency_hz,
            &component) == HAL_OK)
      {
        LCR_PrintComponentResult(
          &component,
          (lcr_secondary_mode != LCR_SECONDARY_NONE) ?
            "secondary-valid-sample-average" :
            "primary-valid-sample-average");

        if (lcr_secondary_mode == LCR_SECONDARY_INDUCTOR_50KHZ)
        {
          if ((component.type != LCR_COMPONENT_INDUCTOR) ||
              !component.parameter_valid ||
              (component.parameter_frequency_hz !=
               LCR_INDUCTOR_FREQUENCY_HZ))
          {
            printf("[LCR] 50 kHz formal measurement did not confirm an inductor\r\n");
            LCR_FailWorkflow(LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID);
            return;
          }

          printf("[LCR] inductor result accepted from 50 kHz formal measurement; 10 kHz data was coarse-detection only\r\n");
          LCR_ResultComplete(&component);
          lcr_secondary_mode = LCR_SECONDARY_NONE;
        }
        else if (lcr_secondary_mode == LCR_SECONDARY_AMBIGUITY_1KHZ)
        {
          LCR_ComponentResult confirmed_component;

          printf("[LCR] multi-frequency pair: primary=%lu Hz, secondary=%lu Hz\r\n",
                 (unsigned long)lcr_primary_frequency_hz,
                 (unsigned long)excitation_status.actual_frequency_hz);
          if (LCR_ComponentConfirm(
                &lcr_primary_impedance,
                lcr_primary_frequency_hz,
                &measurement_summary.average,
                excitation_status.actual_frequency_hz,
                &confirmed_component) == HAL_OK)
          {
            LCR_PrintComponentResult(
              &confirmed_component,
              "two-frequency-consistency-check");
            LCR_ResultComplete(&confirmed_component);
          }
          else
          {
            printf("[LCR] multi-frequency confirmation calculation failed\r\n");
            LCR_FailWorkflow(LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID);
            return;
          }
          lcr_secondary_mode = LCR_SECONDARY_NONE;
        }
        else if ((stop_status == HAL_OK) &&
                 (LCR_IsSuspectedInductor(
                    &measurement_summary.average) ||
                  ((component.type == LCR_COMPONENT_UNKNOWN) &&
                   (excitation_status.actual_frequency_hz !=
                    LCR_AMBIGUITY_FREQUENCY_HZ))))
        {
          HAL_StatusTypeDef confirmation_status;
          uint32_t secondary_frequency_hz;

          lcr_primary_impedance = measurement_summary.average;
          lcr_primary_frequency_hz =
            excitation_status.actual_frequency_hz;
          if (LCR_IsSuspectedInductor(&measurement_summary.average))
          {
            lcr_secondary_mode = LCR_SECONDARY_INDUCTOR_50KHZ;
            secondary_frequency_hz = LCR_INDUCTOR_FREQUENCY_HZ;
            printf("[LCR] 10 kHz coarse result is inductive; starting dedicated 50 kHz formal measurement\r\n");
          }
          else
          {
            lcr_secondary_mode = LCR_SECONDARY_AMBIGUITY_1KHZ;
            secondary_frequency_hz = LCR_AMBIGUITY_FREQUENCY_HZ;
          }
          confirmation_status = LCR_SetFrequency(
            secondary_frequency_hz);
          if (confirmation_status == HAL_OK)
          {
            confirmation_status = LCR_AutorangeBegin(
              &lcr_autorange_session);
          }
          if (confirmation_status == HAL_OK)
          {
            confirmation_status = LCR_ExcitationStart();
          }

          if (confirmation_status == HAL_OK)
          {
            LCR_ExcitationStatus confirmation_excitation;

            LCR_WorkflowBeginConfirmation();
            LCR_ExcitationGetStatus(&confirmation_excitation);
            LCR_UpdateDisplayMeasuring(
              confirmation_excitation.actual_frequency_hz);
            BSP_LED_On(LED_GREEN);
            lcr_capture_due_tick =
              HAL_GetTick() + LCR_CAPTURE_SETTLING_TIME_MS;
            lcr_capture_scheduled = true;
            confirmation_started = true;
            printf("[LCR] starting automatic %lu Hz secondary measurement\r\n",
                   (unsigned long)confirmation_excitation.actual_frequency_hz);
            printf("[LCR] confirmation autorange probe scheduled after %u ms settling\r\n",
                   (unsigned int)LCR_CAPTURE_SETTLING_TIME_MS);
          }
          else
          {
            printf("[LCR] automatic frequency confirmation start failed: HAL=%u\r\n",
                   (unsigned int)confirmation_status);
            LCR_FailWorkflow(LCR_WORKFLOW_ERROR_CONFIRMATION_START);
            return;
          }
        }
        else
        {
          LCR_ResultComplete(&component);
        }
      }
      else
      {
        printf("[LCR] component classification unavailable\r\n");
        LCR_FailWorkflow(
          (lcr_secondary_mode != LCR_SECONDARY_NONE) ?
            LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID :
            LCR_WORKFLOW_ERROR_DFT);
        return;
      }
    }
    if (confirmation_started)
    {
      return;
    }
    LCR_WorkflowComplete();
    {
      LCR_WorkflowStatus workflow_status;

      LCR_WorkflowGetStatus(&workflow_status);
      printf("[LCR] measurement state=%s; code=%s\r\n",
             LCR_WorkflowStateName(workflow_status.state),
             LCR_WorkflowErrorName(workflow_status.last_error));
    }
    LCR_PrintFinalResultSnapshot();
    LCR_UpdateDisplayFromFinalResult();
    printf("[LCR] excitation stopped automatically: HAL=%u\r\n",
           (unsigned int)stop_status);
  }
  else if (LCR_MeasurementIsActive(&lcr_measurement_session))
  {
    lcr_capture_due_tick = HAL_GetTick() + LCR_INTER_CAPTURE_TIME_MS;
    lcr_capture_scheduled = true;
    printf("[LCR] next capture scheduled after %u ms\r\n",
           (unsigned int)LCR_INTER_CAPTURE_TIME_MS);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if (LCR_ExcitationInit() != HAL_OK)
  {
    printf("\r\n[LCR] excitation initialization failed\r\n");
    Error_Handler();
  }
  if (LCR_RangeInit() != HAL_OK)
  {
    printf("\r\n[LCR] range initialization failed\r\n");
    Error_Handler();
  }
  if (LCR_CaptureInit() != HAL_OK)
  {
    printf("\r\n[LCR] ADC calibration failed: error=0x%08lX\r\n",
           (unsigned long)LCR_CaptureGetLastError());
    Error_Handler();
  }
  if (LCR_SelectFeedbackRange(LCR_FEEDBACK_100_OHM) != HAL_OK)
  {
    printf("\r\n[LCR] safe 100 ohm feedback selection failed\r\n");
    Error_Handler();
  }
  if (LCR_SetCurrentGain(LCR_PGA_GAIN_1X) != HAL_OK)
  {
    printf("\r\n[LCR] current PGA 1x selection failed\r\n");
    Error_Handler();
  }
  LCR_WorkflowInit();
  LCR_ResultInit();
  (void)LCR_100kDiagnosticRun(false);
  {
    LCR_DisplayProbeStatus display_probe;

    if (LCR_DisplayProbe(&display_probe) == HAL_OK)
    {
      if (display_probe.detected_address_7bit != 0U)
      {
        const HAL_StatusTypeDef display_init_status = LCR_DisplayInit(
          display_probe.detected_address_7bit);
        HAL_StatusTypeDef display_ready_status = HAL_ERROR;

        if (display_init_status == HAL_OK)
        {
          display_ready_status = LCR_DisplayShowReady();
        }
        printf("[LCR] OLED at 0x%02X: SSD1306 128x64 init HAL=%u, ready screen HAL=%u\r\n",
               (unsigned int)display_probe.detected_address_7bit,
               (unsigned int)display_init_status,
               (unsigned int)display_ready_status);
      }
      else
      {
        printf("[LCR] OLED I2C probe: no response at 0x3C or 0x3D; display remains disabled\r\n");
      }
    }
    else
    {
      printf("[LCR] OLED I2C probe failed; display remains disabled\r\n");
    }
  }

  lcr_last_button_tick = HAL_GetTick() - 200U;
  BSP_LED_Off(LED_GREEN);
  {
    LCR_ExcitationStatus status;
    LCR_RangeStatus range_status;
    const LCR_FeedbackNetwork *feedback;

    LCR_ExcitationGetStatus(&status);
    LCR_RangeGetStatus(&range_status);
    feedback = LCR_GetFeedbackNetwork(range_status.feedback_range);
    printf("\r\n[LCR] stage 3 DFT self-test ready\r\n");
    printf("[LCR] DAC=PA4, points=%lu, requested=%lu Hz, actual=%lu Hz, amplitude=%u counts\r\n",
           (unsigned long)status.samples_per_period,
           (unsigned long)status.requested_frequency_hz,
           (unsigned long)status.actual_frequency_hz,
           (unsigned int)status.amplitude);
    printf("[LCR] feedback=%lu ohm // %lu.%lu pF, V gain=%lu, I gain=%lu\r\n",
           (unsigned long)feedback->resistance_ohms,
           (unsigned long)(feedback->capacitance_tenths_pf / 10U),
           (unsigned long)(feedback->capacitance_tenths_pf % 10U),
           (unsigned long)LCR_GetPgaGainValue(range_status.voltage_gain),
           (unsigned long)LCR_GetPgaGainValue(range_status.current_gain));
    printf("[LCR] ADC mapping: low16=ADC1/PA0/V_AMP, high16=ADC2/PA1/I_AMP\r\n");
    printf("[LCR] autorange H1 target=%lu.%03lu..%lu.%03lu counts; p2p auxiliary low=%u, clip=%u; max adjustments=%u\r\n",
           (unsigned long)(LCR_AUTORANGE_MIN_H1_MILLI_COUNTS / 1000U),
           (unsigned long)(LCR_AUTORANGE_MIN_H1_MILLI_COUNTS % 1000U),
           (unsigned long)(LCR_AUTORANGE_MAX_H1_MILLI_COUNTS / 1000U),
           (unsigned long)(LCR_AUTORANGE_MAX_H1_MILLI_COUNTS % 1000U),
           (unsigned int)LCR_AUTORANGE_MIN_AUX_P2P_COUNTS,
           (unsigned int)LCR_AUTORANGE_MAX_P2P_COUNTS,
           (unsigned int)LCR_AUTORANGE_MAX_ADJUSTMENTS);
    printf("[LCR] general autorange current PGA maximum=%lux; all four feedback profiles remain candidates at every measurement frequency\r\n",
           (unsigned long)LCR_GetPgaGainValue(
             LCR_AUTORANGE_MAX_CURRENT_GAIN));
    printf("[LCR] frequency profiles: 1/10 kHz use 64 points; up to 25 kHz use 32; 50 kHz high-frequency profile uses 16\r\n");
    printf("[LCR] inductor flow: 10 kHz coarse classification, then 50 kHz formal measurement; V/I/R use the common autorange algorithm\r\n");
    printf("[LCR] output is stopped; press PC13 to autorange, then collect %u valid samples in at most %u captures of %u pairs\r\n",
           (unsigned int)LCR_MEASUREMENT_MIN_VALID_COUNT,
           (unsigned int)LCR_MEASUREMENT_MAX_ATTEMPT_COUNT,
           (unsigned int)LCR_CAPTURE_SAMPLE_COUNT);
  }

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (lcr_button_event)
    {
      const uint32_t now = HAL_GetTick();

      lcr_button_event = false;
      if ((uint32_t)(now - lcr_last_button_tick) >= 150U)
      {
        LCR_ExcitationStatus status;
        HAL_StatusTypeDef operation_status;

        lcr_last_button_tick = now;
        LCR_ExcitationGetStatus(&status);
        if (status.running)
        {
          operation_status = LCR_ExcitationStop();
        }
        else
        {
          lcr_secondary_mode = LCR_SECONDARY_NONE;
          LCR_ResultBegin();
          operation_status = LCR_SetFrequency(
            LCR_PRIMARY_FREQUENCY_HZ);
          if (operation_status == HAL_OK)
          {
            operation_status = LCR_AutorangeBegin(&lcr_autorange_session);
          }
          if (operation_status == HAL_OK)
          {
            operation_status = LCR_ExcitationStart();
          }
        }
        LCR_ExcitationGetStatus(&status);

        if (operation_status == HAL_OK)
        {
          if (status.running)
          {
            LCR_MeasurementCancel(&lcr_measurement_session);
            LCR_WorkflowBeginPrimary();
            LCR_UpdateDisplayMeasuring(status.actual_frequency_hz);
            BSP_LED_On(LED_GREEN);
            lcr_capture_due_tick = now + LCR_CAPTURE_SETTLING_TIME_MS;
            lcr_capture_scheduled = true;
            printf("[LCR] excitation started: %lu Hz, amplitude=%u counts\r\n",
                   (unsigned long)status.actual_frequency_hz,
                   (unsigned int)status.amplitude);
            printf("[LCR] autorange started from feedback=100 ohm, V gain=1, I gain=1\r\n");
            printf("[LCR] autorange probe scheduled after %u ms settling\r\n",
                   (unsigned int)LCR_CAPTURE_SETTLING_TIME_MS);
          }
          else
          {
            LCR_MeasurementSummary measurement_summary;

            lcr_capture_scheduled = false;
            (void)LCR_CaptureAbort();
            BSP_LED_Off(LED_GREEN);
            LCR_MeasurementGetSummary(
              &lcr_measurement_session,
              &measurement_summary);
            LCR_MeasurementCancel(&lcr_measurement_session);
            LCR_AutorangeCancel(&lcr_autorange_session);
            lcr_secondary_mode = LCR_SECONDARY_NONE;
            LCR_ResultStoreMeasurement(
              LCR_WorkflowIsConfirmation(),
              status.actual_frequency_hz,
              lcr_autorange_session.probe_count,
              &measurement_summary);
            LCR_ResultAbort();
            LCR_WorkflowAbort();
            printf("[LCR] measurement aborted by user: attempts=%lu eligible=%lu; autorange_probes=%lu\r\n",
                   (unsigned long)measurement_summary.attempt_count,
                   (unsigned long)measurement_summary.eligible_count,
                   (unsigned long)lcr_autorange_session.probe_count);
            printf("[LCR] measurement state=ABORTED; code=NONE\r\n");
            LCR_PrintFinalResultSnapshot();
            LCR_UpdateDisplayFromFinalResult();
            printf("[LCR] excitation stopped\r\n");
          }
        }
        else
        {
          printf("[LCR] excitation start/stop failed: HAL status=%u\r\n",
                 (unsigned int)operation_status);
          LCR_FailWorkflow(LCR_WORKFLOW_ERROR_AUTORANGE);
        }
      }
    }

    if (lcr_capture_scheduled &&
        ((int32_t)(HAL_GetTick() - lcr_capture_due_tick) >= 0))
    {
      const HAL_StatusTypeDef capture_status = LCR_CaptureStart();

      lcr_capture_scheduled = false;
      if (capture_status == HAL_OK)
      {
        const uint32_t capture_timeout_ms = LCR_GetCaptureTimeoutMs();

        lcr_capture_deadline_tick = HAL_GetTick() + capture_timeout_ms;
        if (LCR_AutorangeIsSearching(&lcr_autorange_session))
        {
          printf("[LCR] autorange probe %lu started; timeout=%lu ms\r\n",
                 (unsigned long)(lcr_autorange_session.probe_count + 1U),
                 (unsigned long)capture_timeout_ms);
        }
        else
        {
          LCR_MeasurementSummary measurement_summary;

          LCR_MeasurementGetSummary(
            &lcr_measurement_session,
            &measurement_summary);
          printf("[LCR] dual ADC capture %lu/%u started; timeout=%lu ms\r\n",
                 (unsigned long)(measurement_summary.attempt_count + 1U),
                 (unsigned int)LCR_MEASUREMENT_MAX_ATTEMPT_COUNT,
                 (unsigned long)capture_timeout_ms);
        }
      }
      else
      {
        printf("[LCR] dual ADC capture start failed: HAL=%u, error=0x%08lX\r\n",
               (unsigned int)capture_status,
               (unsigned long)LCR_CaptureGetLastError());
        LCR_FailWorkflow(LCR_WORKFLOW_ERROR_ADC_START);
      }
    }

    if ((LCR_CaptureGetState() == LCR_CAPTURE_RUNNING) &&
        ((int32_t)(HAL_GetTick() - lcr_capture_deadline_tick) >= 0))
    {
      printf("[LCR] dual ADC capture timed out: deadline=%lu ms, error=0x%08lX\r\n",
             (unsigned long)lcr_capture_deadline_tick,
             (unsigned long)LCR_CaptureGetLastError());
      LCR_FailWorkflow(LCR_WORKFLOW_ERROR_ADC_TIMEOUT);
    }

    if ((LCR_CaptureGetState() == LCR_CAPTURE_COMPLETE) ||
        (LCR_CaptureGetState() == LCR_CAPTURE_ERROR))
    {
      LCR_CaptureSummary summary;
      const HAL_StatusTypeDef capture_status = LCR_CaptureProcess(&summary);

      if (capture_status == HAL_OK)
      {
        LCR_DspCaptureResult dsp_result;
        LCR_ImpedanceResult measurement_result = {0};
        const bool autorange_probe =
          LCR_AutorangeIsSearching(&lcr_autorange_session);
        bool dsp_result_available = false;
        bool measurement_result_available = false;
        bool measurement_result_calibrated = false;
        bool measurement_result_eligible = false;
        bool autorange_voltage_thd_ok = false;
        bool autorange_current_thd_ok = false;

        lcr_capture_deadline_tick = 0U;

        printf("[LCR] capture complete: %lu sample pairs\r\n",
               (unsigned long)summary.sample_count);
        printf("[LCR] V ADC1/PA0: first=%u last=%u min=%u max=%u mean=%u p2p=%u rail=%lu\r\n",
               (unsigned int)summary.voltage.first,
               (unsigned int)summary.voltage.last,
               (unsigned int)summary.voltage.minimum,
               (unsigned int)summary.voltage.maximum,
               (unsigned int)summary.voltage.mean,
               (unsigned int)summary.voltage.peak_to_peak,
               (unsigned long)summary.voltage.near_rail_count);
        printf("[LCR] I ADC2/PA1: first=%u last=%u min=%u max=%u mean=%u p2p=%u rail=%lu\r\n",
               (unsigned int)summary.current.first,
               (unsigned int)summary.current.last,
               (unsigned int)summary.current.minimum,
               (unsigned int)summary.current.maximum,
               (unsigned int)summary.current.mean,
               (unsigned int)summary.current.peak_to_peak,
               (unsigned long)summary.current.near_rail_count);

        {
          LCR_ExcitationStatus dsp_excitation_status;

          LCR_ExcitationGetStatus(&dsp_excitation_status);
          if (LCR_DspAnalyzeCapture(
              dsp_excitation_status.samples_per_period,
              &dsp_result) == HAL_OK)
          {
          uint32_t voltage_thd_limit_millipercent;
          uint32_t current_thd_limit_millipercent;
          const uint32_t quality_flags = LCR_EvaluateCaptureQuality(
            &summary,
            &dsp_result,
            &voltage_thd_limit_millipercent,
            &current_thd_limit_millipercent);

          dsp_result_available = true;
          measurement_result_eligible = quality_flags == 0U;
          autorange_voltage_thd_ok =
            (quality_flags & LCR_QUALITY_VOLTAGE_THD_HIGH) == 0U;
          autorange_current_thd_ok =
            (quality_flags & LCR_QUALITY_CURRENT_THD_HIGH) == 0U;

          printf("[LCR] V H1=%lu.%03lu counts phase=%ld.%03lu deg THD2-5=%lu.%03lu%%\r\n",
                 (unsigned long)(dsp_result.voltage.bin[1].magnitude_milli_counts / 1000U),
                 (unsigned long)(dsp_result.voltage.bin[1].magnitude_milli_counts % 1000U),
                 (long)(dsp_result.voltage.bin[1].phase_millidegrees / 1000L),
                 (unsigned long)labs(dsp_result.voltage.bin[1].phase_millidegrees % 1000L),
                 (unsigned long)(dsp_result.voltage.thd_millipercent / 1000U),
                 (unsigned long)(dsp_result.voltage.thd_millipercent % 1000U));
          printf("[LCR] V harmonics: H2=%lu.%03lu%% H3=%lu.%03lu%% H4=%lu.%03lu%% H5=%lu.%03lu%%\r\n",
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[2] / 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[2] % 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[3] / 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[3] % 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[4] / 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[4] % 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[5] / 1000U),
                 (unsigned long)(dsp_result.voltage.harmonic_ratio_millipercent[5] % 1000U));
          LCR_PrintCaptureQuality(
            quality_flags,
            &summary,
            &dsp_result,
            voltage_thd_limit_millipercent,
            current_thd_limit_millipercent,
            autorange_probe);
          if (dsp_result.current.fundamental_valid)
          {
            LCR_ExcitationStatus excitation_status;
            LCR_RangeStatus range_status;
            LCR_ImpedanceResult impedance;
            const LCR_FeedbackNetwork *feedback;

            printf("[LCR] I H1=%lu.%03lu counts phase=%ld.%03lu deg THD2-5=%lu.%03lu%%\r\n",
                   (unsigned long)(dsp_result.current.bin[1].magnitude_milli_counts / 1000U),
                   (unsigned long)(dsp_result.current.bin[1].magnitude_milli_counts % 1000U),
                   (long)(dsp_result.current.bin[1].phase_millidegrees / 1000L),
                   (unsigned long)labs(dsp_result.current.bin[1].phase_millidegrees % 1000L),
                   (unsigned long)(dsp_result.current.thd_millipercent / 1000U),
                   (unsigned long)(dsp_result.current.thd_millipercent % 1000U));
            printf("[LCR] V-I phase=%ld.%03lu deg\r\n",
                   (long)(dsp_result.voltage_minus_current_millidegrees / 1000L),
                   (unsigned long)labs(dsp_result.voltage_minus_current_millidegrees % 1000L));

            LCR_ExcitationGetStatus(&excitation_status);
            LCR_RangeGetStatus(&range_status);
            feedback = LCR_GetFeedbackNetwork(range_status.feedback_range);
            if (LCR_ImpedanceCalculateRaw(
                  &dsp_result,
                  excitation_status.actual_frequency_hz,
                  feedback,
                  range_status.voltage_gain,
                  range_status.current_gain,
                  &impedance) == HAL_OK)
            {
              LCR_ImpedanceResult calibrated_impedance;

              measurement_result = impedance;
              measurement_result_available = true;

              printf("[LCR] raw Z: R=%ld.%03lu ohm, X=%ld.%03lu ohm, |Z|=%lu.%03lu ohm, phase=%ld.%03lu deg\r\n",
                     (long)(impedance.resistance_milliohms / 1000L),
                     (unsigned long)labs(impedance.resistance_milliohms % 1000L),
                     (long)(impedance.reactance_milliohms / 1000L),
                     (unsigned long)labs(impedance.reactance_milliohms % 1000L),
                     (unsigned long)(impedance.magnitude_milliohms / 1000U),
                     (unsigned long)(impedance.magnitude_milliohms % 1000U),
                     (long)(impedance.phase_millidegrees / 1000L),
                     (unsigned long)labs(impedance.phase_millidegrees % 1000L));

              if (LCR_CalibrationApply(
                    &impedance,
                    excitation_status.actual_frequency_hz,
                    range_status.feedback_range,
                    range_status.voltage_gain,
                    range_status.current_gain,
                    &calibrated_impedance) == HAL_OK)
              {
                measurement_result = calibrated_impedance;
                measurement_result_calibrated = true;
                printf("[LCR] calibration=EXACT_PROFILE for current frequency/range/gain\r\n");
                printf("[LCR] calibrated Z: R=%ld.%03lu ohm, X=%ld.%03lu ohm, |Z|=%lu.%03lu ohm, phase=%ld.%03lu deg\r\n",
                       (long)(calibrated_impedance.resistance_milliohms / 1000L),
                       (unsigned long)labs(calibrated_impedance.resistance_milliohms % 1000L),
                       (long)(calibrated_impedance.reactance_milliohms / 1000L),
                       (unsigned long)labs(calibrated_impedance.reactance_milliohms % 1000L),
                       (unsigned long)(calibrated_impedance.magnitude_milliohms / 1000U),
                       (unsigned long)(calibrated_impedance.magnitude_milliohms % 1000U),
                       (long)(calibrated_impedance.phase_millidegrees / 1000L),
                       (unsigned long)labs(calibrated_impedance.phase_millidegrees % 1000L));
              }
              else
              {
                printf("[LCR] calibration=UNAVAILABLE for current frequency/range/gain; result remains RAW/UNCALIBRATED\r\n");
              }
            }
            else
            {
              printf("[LCR] raw impedance calculation failed\r\n");
            }
          }
          else
          {
            printf("[LCR] I fundamental below 5 counts; phase is not valid (connect DUT)\r\n");
          }
          }
          else
          {
            printf("[LCR] DFT analysis failed\r\n");
          }
        }

        if (autorange_probe)
        {
          if (dsp_result_available)
          {
            LCR_AutorangeEvaluation evaluation;
            const HAL_StatusTypeDef autorange_status = LCR_AutorangeEvaluate(
              &lcr_autorange_session,
              &summary,
              &dsp_result,
              autorange_voltage_thd_ok,
              autorange_current_thd_ok,
              &evaluation);

            if (autorange_status == HAL_OK)
            {
              printf("[LCR] autorange probe %lu retained: V p2p=%u, I p2p=%u, averaged=no\r\n",
                     (unsigned long)lcr_autorange_session.probe_count,
                     (unsigned int)summary.voltage.peak_to_peak,
                     (unsigned int)summary.current.peak_to_peak);
              LCR_PrintAutorangeReasons(evaluation.reason_flags);
              LCR_PrintAutorangeSelection(&evaluation);

              if (evaluation.decision == LCR_AUTORANGE_RETRY)
              {
                lcr_capture_due_tick =
                  HAL_GetTick() + LCR_CAPTURE_SETTLING_TIME_MS;
                lcr_capture_scheduled = true;
                printf("[LCR] autorange retry after %u ms settling; adjustment=%lu/%u\r\n",
                       (unsigned int)LCR_CAPTURE_SETTLING_TIME_MS,
                       (unsigned long)lcr_autorange_session.adjustment_count,
                       (unsigned int)LCR_AUTORANGE_MAX_ADJUSTMENTS);
              }
              else if (evaluation.decision == LCR_AUTORANGE_ACCEPTED)
              {
                const LCR_FeedbackNetwork *locked_feedback =
                  LCR_GetFeedbackNetwork(
                    evaluation.selected_range.feedback_range);

                LCR_MeasurementBegin(&lcr_measurement_session);
                LCR_WorkflowBeginAveraging();
                lcr_capture_due_tick =
                  HAL_GetTick() + LCR_CAPTURE_SETTLING_TIME_MS;
                lcr_capture_scheduled = true;
                if (locked_feedback != NULL)
                {
                  printf("[LCR] autorange locked after %lu probes: feedback=%lu ohm, V gain=%lu, I gain=%lu\r\n",
                         (unsigned long)lcr_autorange_session.probe_count,
                         (unsigned long)locked_feedback->resistance_ohms,
                         (unsigned long)LCR_GetPgaGainValue(
                           evaluation.selected_range.voltage_gain),
                         (unsigned long)LCR_GetPgaGainValue(
                           evaluation.selected_range.current_gain));
                }
                printf("[LCR] autorange probes remain in log but are excluded; after %u ms settling, collect at least %u valid fresh samples in at most %u attempts\r\n",
                       (unsigned int)LCR_CAPTURE_SETTLING_TIME_MS,
                       (unsigned int)LCR_MEASUREMENT_MIN_VALID_COUNT,
                       (unsigned int)LCR_MEASUREMENT_MAX_ATTEMPT_COUNT);
              }
              else
              {
                const bool no_dut_or_open =
                  ((evaluation.reason_flags &
                    LCR_AUTORANGE_REASON_I_LOW) != 0U) &&
                  (((evaluation.reason_flags &
                     LCR_AUTORANGE_REASON_I_LIMIT) != 0U) ||
                   ((evaluation.reason_flags &
                     LCR_AUTORANGE_REASON_MAX_ADJUSTMENT) != 0U));

                printf("[LCR] autorange failed after %lu probes and %lu adjustments\r\n",
                       (unsigned long)lcr_autorange_session.probe_count,
                       (unsigned long)lcr_autorange_session.adjustment_count);
                LCR_FailWorkflow(
                  no_dut_or_open ?
                    LCR_WORKFLOW_ERROR_NO_DUT_OR_OPEN :
                    LCR_WORKFLOW_ERROR_AUTORANGE);
              }
            }
            else
            {
              printf("[LCR] autorange evaluation error\r\n");
              LCR_FailWorkflow(LCR_WORKFLOW_ERROR_AUTORANGE);
            }
          }
          else
          {
            printf("[LCR] autorange aborted: DFT unavailable\r\n");
            LCR_FailWorkflow(LCR_WORKFLOW_ERROR_DFT);
          }
        }
        else
        {
          (void)LCR_MeasurementRecord(
            &lcr_measurement_session,
            measurement_result_available ? &measurement_result : NULL,
            measurement_result_eligible && measurement_result_available,
            measurement_result_calibrated);
          LCR_ScheduleNextCaptureOrFinish();
        }
      }
      else
      {
        printf("[LCR] dual ADC capture failed: HAL=%u, error=0x%08lX\r\n",
               (unsigned int)capture_status,
               (unsigned long)LCR_CaptureGetLastError());
        LCR_FailWorkflow(LCR_WORKFLOW_ERROR_ADC_PROCESS);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    lcr_button_event = true;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
