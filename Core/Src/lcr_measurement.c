#include "lcr_measurement.h"

#include <math.h>
#include <stddef.h>

#define LCR_MEASUREMENT_PI 3.14159265358979323846f

static int32_t LCR_MeasurementDivideRounded(int64_t value, uint32_t divisor)
{
  const int64_t rounding = (int64_t)(divisor / 2U);

  if (value >= 0)
  {
    return (int32_t)((value + rounding) / (int64_t)divisor);
  }
  return (int32_t)((value - rounding) / (int64_t)divisor);
}

void LCR_MeasurementBegin(LCR_MeasurementSession *session)
{
  if (session == NULL)
  {
    return;
  }

  session->attempt_count = 0U;
  session->eligible_count = 0U;
  session->resistance_sum_milliohms = 0LL;
  session->reactance_sum_milliohms = 0LL;
  session->active = true;
  session->result_source_set = false;
  session->result_is_calibrated = false;
}

void LCR_MeasurementCancel(LCR_MeasurementSession *session)
{
  if (session != NULL)
  {
    session->active = false;
  }
}

HAL_StatusTypeDef LCR_MeasurementRecord(
  LCR_MeasurementSession *session,
  const LCR_ImpedanceResult *result,
  bool eligible,
  bool result_is_calibrated)
{
  if ((session == NULL) || !session->active ||
      (session->attempt_count >= LCR_MEASUREMENT_ATTEMPT_COUNT))
  {
    return HAL_ERROR;
  }

  ++session->attempt_count;
  if (eligible)
  {
    if ((result == NULL) || !result->valid)
    {
      return HAL_ERROR;
    }
    if (session->result_source_set &&
        (session->result_is_calibrated != result_is_calibrated))
    {
      return HAL_ERROR;
    }

    session->result_source_set = true;
    session->result_is_calibrated = result_is_calibrated;
    session->resistance_sum_milliohms += result->resistance_milliohms;
    session->reactance_sum_milliohms += result->reactance_milliohms;
    ++session->eligible_count;
  }

  if (session->attempt_count >= LCR_MEASUREMENT_ATTEMPT_COUNT)
  {
    session->active = false;
  }
  return HAL_OK;
}

bool LCR_MeasurementIsActive(const LCR_MeasurementSession *session)
{
  return (session != NULL) && session->active;
}

bool LCR_MeasurementIsComplete(const LCR_MeasurementSession *session)
{
  return (session != NULL) && !session->active &&
         (session->attempt_count >= LCR_MEASUREMENT_ATTEMPT_COUNT);
}

void LCR_MeasurementGetSummary(
  const LCR_MeasurementSession *session,
  LCR_MeasurementSummary *summary)
{
  float resistance_ohms;
  float reactance_ohms;

  if ((session == NULL) || (summary == NULL))
  {
    return;
  }

  summary->attempt_count = session->attempt_count;
  summary->eligible_count = session->eligible_count;
  summary->ineligible_count =
    session->attempt_count - session->eligible_count;
  summary->average_valid = session->eligible_count != 0U;
  summary->average_is_calibrated = session->result_is_calibrated;
  summary->average.resistance_milliohms = 0;
  summary->average.reactance_milliohms = 0;
  summary->average.magnitude_milliohms = 0U;
  summary->average.phase_millidegrees = 0;
  summary->average.valid = false;

  if (!summary->average_valid)
  {
    return;
  }

  summary->average.resistance_milliohms = LCR_MeasurementDivideRounded(
    session->resistance_sum_milliohms,
    session->eligible_count);
  summary->average.reactance_milliohms = LCR_MeasurementDivideRounded(
    session->reactance_sum_milliohms,
    session->eligible_count);
  resistance_ohms =
    (float)summary->average.resistance_milliohms / 1000.0f;
  reactance_ohms =
    (float)summary->average.reactance_milliohms / 1000.0f;
  summary->average.magnitude_milliohms = (uint32_t)(
    (sqrtf((resistance_ohms * resistance_ohms) +
           (reactance_ohms * reactance_ohms)) * 1000.0f) + 0.5f);
  summary->average.phase_millidegrees = (int32_t)(
    (atan2f(reactance_ohms, resistance_ohms) *
     (180000.0f / LCR_MEASUREMENT_PI)) +
    ((reactance_ohms >= 0.0f) ? 0.5f : -0.5f));
  summary->average.valid = true;
}
