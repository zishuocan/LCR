#include "lcr_100k_diagnostic.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "lcr_calibration.h"
#include "lcr_capture.h"
#include "lcr_dsp.h"
#include "lcr_excitation.h"
#include "lcr_impedance.h"
#include "lcr_range.h"

#define LCR_100K_DIAGNOSTIC_SETTLING_MS          10U
#define LCR_100K_DIAGNOSTIC_INTER_CAPTURE_MS      2U
#define LCR_100K_DIAGNOSTIC_CAPTURE_TIMEOUT_MS  250U
#define LCR_100K_DIAGNOSTIC_MIN_I_P2P_COUNTS     10U
#define LCR_100K_VALIDATION_CAPTURE_COUNT          5U
#define LCR_100K_REFERENCE_MILLIOHMS        93700000L
#define LCR_100K_MAX_ERROR_PERMILLE               50U

static HAL_StatusTypeDef LCR_100kDiagnosticCapture(
  LCR_CaptureSummary *summary,
  LCR_DspCaptureResult *dsp)
{
  const uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status = LCR_CaptureStart();

  if (status != HAL_OK)
  {
    return status;
  }

  while ((LCR_CaptureGetState() == LCR_CAPTURE_RUNNING) &&
         ((uint32_t)(HAL_GetTick() - start_tick) <
          LCR_100K_DIAGNOSTIC_CAPTURE_TIMEOUT_MS))
  {
  }

  if (LCR_CaptureGetState() == LCR_CAPTURE_RUNNING)
  {
    (void)LCR_CaptureAbort();
    return HAL_TIMEOUT;
  }

  status = LCR_CaptureProcess(summary);
  if (status == HAL_OK)
  {
    LCR_ExcitationStatus excitation;

    LCR_ExcitationGetStatus(&excitation);
    status = LCR_DspAnalyzeCapture(excitation.samples_per_period, dsp);
  }
  return status;
}

static uint32_t LCR_100kDiagnosticErrorPermille(int32_t resistance_milliohms)
{
  int64_t difference =
    (int64_t)resistance_milliohms - LCR_100K_REFERENCE_MILLIOHMS;

  if (difference < 0LL)
  {
    difference = -difference;
  }
  return (uint32_t)((difference * 1000LL +
                     (LCR_100K_REFERENCE_MILLIOHMS / 2L)) /
                    LCR_100K_REFERENCE_MILLIOHMS);
}

static void LCR_100kDiagnosticPrintImpedance(
  const char *label,
  const LCR_ImpedanceResult *result)
{
  printf("%s R=%ld.%03lu ohm X=%ld.%03lu ohm",
         label,
         (long)(result->resistance_milliohms / 1000L),
         (unsigned long)labs(result->resistance_milliohms % 1000L),
         (long)(result->reactance_milliohms / 1000L),
         (unsigned long)labs(result->reactance_milliohms % 1000L));
}

HAL_StatusTypeDef LCR_100kDiagnosticRun(bool run_reference_validation)
{
  const LCR_FeedbackNetwork *feedback =
    LCR_GetFeedbackNetwork(LCR_FEEDBACK_100K_OHM);
  LCR_CaptureSummary summary;
  LCR_DspCaptureResult dsp;
  LCR_RangeStatus range;
  int32_t minimum_resistance = INT32_MAX;
  int32_t maximum_resistance = INT32_MIN;
  uint32_t passed_count = 0U;
  uint32_t index;
  bool self_test_passed = false;
  HAL_StatusTypeDef status;

  printf("\r\n[LCR][100K] startup hardware self-test: stop excitation, force feedback=100000 ohm, V gain=1, I gain=1\r\n");
  (void)LCR_CaptureAbort();
  status = LCR_ExcitationStop();
  if (status == HAL_OK)
  {
    status = LCR_SetFrequency(LCR_EXCITATION_DEFAULT_FREQUENCY_HZ);
  }
  if (status == HAL_OK)
  {
    status = LCR_SetAmplitude(LCR_EXCITATION_DEFAULT_AMPLITUDE);
  }
  if (status == HAL_OK)
  {
    status = LCR_SetVoltageGain(LCR_PGA_GAIN_1X);
  }
  if (status == HAL_OK)
  {
    status = LCR_SetCurrentGain(LCR_PGA_GAIN_1X);
  }
  if (status == HAL_OK)
  {
    status = LCR_SelectFeedbackRange(LCR_FEEDBACK_100K_OHM);
  }

  LCR_RangeGetStatus(&range);
  if ((status != HAL_OK) || (feedback == NULL) ||
      (range.requested_feedback_range != LCR_FEEDBACK_100K_OHM) ||
      (range.feedback_range != LCR_FEEDBACK_100K_OHM) ||
      (range.voltage_gain != LCR_PGA_GAIN_1X) ||
      (range.current_gain != LCR_PGA_GAIN_1X))
  {
    printf("[LCR][100K] ERROR: exact 100K hardware mapping failed; no 10K remap will be used\r\n");
    status = HAL_ERROR;
    goto restore_safe_range;
  }

  HAL_Delay(LCR_100K_DIAGNOSTIC_SETTLING_MS);
  status = LCR_ExcitationStart();
  if (status == HAL_OK)
  {
    HAL_Delay(LCR_100K_DIAGNOSTIC_SETTLING_MS);
    status = LCR_100kDiagnosticCapture(&summary, &dsp);
  }
  if (status != HAL_OK)
  {
    printf("[LCR][100K] ERROR: self-test capture failed: HAL=%u capture_error=0x%08lX\r\n",
           (unsigned int)status,
           (unsigned long)LCR_CaptureGetLastError());
    goto restore_safe_range;
  }

  printf("[LCR][100K] self-test response: V p2p=%u, I p2p=%u, I mean=%u, I H1=%lu.%03lu counts, I THD=%lu.%03lu%%\r\n",
         (unsigned int)summary.voltage.peak_to_peak,
         (unsigned int)summary.current.peak_to_peak,
         (unsigned int)summary.current.mean,
         (unsigned long)(dsp.current.bin[1].magnitude_milli_counts / 1000U),
         (unsigned long)(dsp.current.bin[1].magnitude_milli_counts % 1000U),
         (unsigned long)(dsp.current.thd_millipercent / 1000U),
         (unsigned long)(dsp.current.thd_millipercent % 1000U));

  self_test_passed = dsp.current.fundamental_valid &&
    (summary.current.peak_to_peak >=
     LCR_100K_DIAGNOSTIC_MIN_I_P2P_COUNTS) &&
    (summary.current.near_rail_count == 0U);
  if (!self_test_passed)
  {
    printf("[LCR][100K] ERROR: I channel remains fixed/near-DC or invalid in forced 100K mode; fault retained, no fallback to 10K\r\n");
    status = HAL_ERROR;
    goto restore_safe_range;
  }
  printf("[LCR][100K] PASS: normal AC response detected in the physical 100K feedback branch\r\n");

  if (!run_reference_validation)
  {
    status = HAL_OK;
    goto restore_safe_range;
  }

  printf("[LCR][100K] forced 93.7k reference validation: %u consecutive captures\r\n",
         (unsigned int)LCR_100K_VALIDATION_CAPTURE_COUNT);
  for (index = 0U; index < LCR_100K_VALIDATION_CAPTURE_COUNT; ++index)
  {
    LCR_ImpedanceResult raw = {0};
    LCR_ImpedanceResult calibrated = {0};
    uint32_t error_permille = UINT32_MAX;
    bool raw_valid;
    bool calibrated_valid;

    HAL_Delay(LCR_100K_DIAGNOSTIC_INTER_CAPTURE_MS);
    status = LCR_100kDiagnosticCapture(&summary, &dsp);
    if (status != HAL_OK)
    {
      printf("[LCR][100K] validation %lu/%u: capture failed HAL=%u error=0x%08lX\r\n",
             (unsigned long)(index + 1U),
             (unsigned int)LCR_100K_VALIDATION_CAPTURE_COUNT,
             (unsigned int)status,
             (unsigned long)LCR_CaptureGetLastError());
      continue;
    }
    raw_valid = LCR_ImpedanceCalculateRaw(
        &dsp,
        LCR_EXCITATION_DEFAULT_FREQUENCY_HZ,
        feedback,
        LCR_PGA_GAIN_1X,
        LCR_PGA_GAIN_1X,
        &raw) == HAL_OK;
    calibrated_valid = raw_valid &&
      (LCR_CalibrationApply(
        &raw,
        LCR_EXCITATION_DEFAULT_FREQUENCY_HZ,
        LCR_FEEDBACK_100K_OHM,
        LCR_PGA_GAIN_1X,
        LCR_PGA_GAIN_1X,
        &calibrated) == HAL_OK);

    printf("[LCR][100K] validation %lu/%u: V p2p=%u THD=%lu.%03lu%%; I p2p=%u THD=%lu.%03lu%%; ",
           (unsigned long)(index + 1U),
           (unsigned int)LCR_100K_VALIDATION_CAPTURE_COUNT,
           (unsigned int)summary.voltage.peak_to_peak,
           (unsigned long)(dsp.voltage.thd_millipercent / 1000U),
           (unsigned long)(dsp.voltage.thd_millipercent % 1000U),
           (unsigned int)summary.current.peak_to_peak,
           (unsigned long)(dsp.current.thd_millipercent / 1000U),
           (unsigned long)(dsp.current.thd_millipercent % 1000U));
    if (raw_valid)
    {
      LCR_100kDiagnosticPrintImpedance("raw", &raw);
    }
    else
    {
      printf("raw unavailable");
    }
    printf("; ");
    if (calibrated_valid)
    {
      LCR_100kDiagnosticPrintImpedance("cal", &calibrated);
      error_permille = LCR_100kDiagnosticErrorPermille(
        calibrated.resistance_milliohms);
      printf(" error=%lu.%lu%%",
             (unsigned long)(error_permille / 10U),
             (unsigned long)(error_permille % 10U));
      if (calibrated.resistance_milliohms < minimum_resistance)
      {
        minimum_resistance = calibrated.resistance_milliohms;
      }
      if (calibrated.resistance_milliohms > maximum_resistance)
      {
        maximum_resistance = calibrated.resistance_milliohms;
      }
    }
    else
    {
      printf("cal unavailable");
    }
    printf("\r\n");

    if (calibrated_valid &&
        (error_permille <= LCR_100K_MAX_ERROR_PERMILLE))
    {
      ++passed_count;
    }
  }

  if ((passed_count == LCR_100K_VALIDATION_CAPTURE_COUNT) &&
      (((int64_t)maximum_resistance - minimum_resistance) * 1000LL <=
       ((int64_t)LCR_100K_REFERENCE_MILLIOHMS *
        LCR_100K_MAX_ERROR_PERMILLE)))
  {
    printf("[LCR][100K] PASS: 5/5 within 5%% of 93.7k and spread within 5%%; min=%ld.%03lu max=%ld.%03lu ohm\r\n",
           (long)(minimum_resistance / 1000L),
           (unsigned long)labs(minimum_resistance % 1000L),
           (long)(maximum_resistance / 1000L),
           (unsigned long)labs(maximum_resistance % 1000L));
    status = HAL_OK;
  }
  else
  {
    printf("[LCR][100K] ERROR: forced validation failed (%lu/%u captures within 5%%); real 100K fault/result retained, no 10K substitution\r\n",
           (unsigned long)passed_count,
           (unsigned int)LCR_100K_VALIDATION_CAPTURE_COUNT);
    status = HAL_ERROR;
  }

restore_safe_range:
  (void)LCR_CaptureAbort();
  (void)LCR_ExcitationStop();
  (void)LCR_SetVoltageGain(LCR_PGA_GAIN_1X);
  (void)LCR_SetCurrentGain(LCR_PGA_GAIN_1X);
  (void)LCR_SelectFeedbackRange(LCR_FEEDBACK_100_OHM);
  printf("[LCR][100K] diagnostic complete: %s; excitation stopped, safe 100 ohm range restored\r\n",
         (status == HAL_OK) ? "PASS" : "ERROR");
  return status;
}
