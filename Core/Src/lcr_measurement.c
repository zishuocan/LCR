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

static uint64_t LCR_MeasurementAbsoluteDifference(
  int32_t value,
  int32_t reference)
{
  const int64_t difference = (int64_t)value - (int64_t)reference;

  return (difference < 0LL) ?
    (uint64_t)(-difference) : (uint64_t)difference;
}

static void LCR_MeasurementSortInt32(int32_t *values, uint32_t count)
{
  uint32_t index;

  for (index = 1U; index < count; ++index)
  {
    const int32_t value = values[index];
    uint32_t position = index;

    while ((position != 0U) && (values[position - 1U] > value))
    {
      values[position] = values[position - 1U];
      --position;
    }
    values[position] = value;
  }
}

static void LCR_MeasurementSortUint64(uint64_t *values, uint32_t count)
{
  uint32_t index;

  for (index = 1U; index < count; ++index)
  {
    const uint64_t value = values[index];
    uint32_t position = index;

    while ((position != 0U) && (values[position - 1U] > value))
    {
      values[position] = values[position - 1U];
      --position;
    }
    values[position] = value;
  }
}

static int32_t LCR_MeasurementMedianInt32(int32_t *values, uint32_t count)
{
  LCR_MeasurementSortInt32(values, count);
  if ((count & 1U) != 0U)
  {
    return values[count / 2U];
  }
  return (int32_t)(((int64_t)values[(count / 2U) - 1U] +
                    values[count / 2U]) / 2LL);
}

void LCR_MeasurementBegin(LCR_MeasurementSession *session)
{
  if (session == NULL)
  {
    return;
  }

  session->attempt_count = 0U;
  session->eligible_count = 0U;
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
      (session->attempt_count >= LCR_MEASUREMENT_MAX_ATTEMPT_COUNT))
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
    session->eligible_samples[session->eligible_count] = *result;
    ++session->eligible_count;
  }

  if ((session->eligible_count >= LCR_MEASUREMENT_MIN_VALID_COUNT) ||
      (session->attempt_count >= LCR_MEASUREMENT_MAX_ATTEMPT_COUNT))
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
         ((session->eligible_count >= LCR_MEASUREMENT_MIN_VALID_COUNT) ||
          (session->attempt_count >= LCR_MEASUREMENT_MAX_ATTEMPT_COUNT));
}

void LCR_MeasurementGetSummary(
  const LCR_MeasurementSession *session,
  LCR_MeasurementSummary *summary)
{
  float resistance_ohms;
  float reactance_ohms;
  int32_t resistance_values[LCR_MEASUREMENT_MAX_ATTEMPT_COUNT];
  int32_t reactance_values[LCR_MEASUREMENT_MAX_ATTEMPT_COUNT];
  uint64_t distances[LCR_MEASUREMENT_MAX_ATTEMPT_COUNT];
  int32_t median_resistance;
  int32_t median_reactance;
  uint64_t median_distance;
  uint64_t magnitude_floor;
  uint64_t rejection_threshold;
  int64_t resistance_sum = 0LL;
  int64_t reactance_sum = 0LL;
  uint32_t index;

  if ((session == NULL) || (summary == NULL))
  {
    return;
  }

  summary->attempt_count = session->attempt_count;
  summary->eligible_count = session->eligible_count;
  summary->ineligible_count =
    session->attempt_count - session->eligible_count;
  summary->retained_count = 0U;
  summary->outlier_count = 0U;
  summary->average_valid = false;
  summary->average_is_calibrated = session->result_is_calibrated;
  summary->average.resistance_milliohms = 0;
  summary->average.reactance_milliohms = 0;
  summary->average.magnitude_milliohms = 0U;
  summary->average.phase_millidegrees = 0;
  summary->average.valid = false;

  if (session->eligible_count < LCR_MEASUREMENT_MIN_VALID_COUNT)
  {
    return;
  }

  for (index = 0U; index < session->eligible_count; ++index)
  {
    resistance_values[index] =
      session->eligible_samples[index].resistance_milliohms;
    reactance_values[index] =
      session->eligible_samples[index].reactance_milliohms;
  }
  median_resistance = LCR_MeasurementMedianInt32(
    resistance_values, session->eligible_count);
  median_reactance = LCR_MeasurementMedianInt32(
    reactance_values, session->eligible_count);
  for (index = 0U; index < session->eligible_count; ++index)
  {
    const uint64_t resistance_distance = LCR_MeasurementAbsoluteDifference(
      session->eligible_samples[index].resistance_milliohms,
      median_resistance);
    const uint64_t reactance_distance = LCR_MeasurementAbsoluteDifference(
      session->eligible_samples[index].reactance_milliohms,
      median_reactance);

    distances[index] = (resistance_distance > reactance_distance) ?
      resistance_distance : reactance_distance;
  }
  LCR_MeasurementSortUint64(distances, session->eligible_count);
  median_distance = distances[session->eligible_count / 2U];
  resistance_ohms = (float)median_resistance / 1000.0f;
  reactance_ohms = (float)median_reactance / 1000.0f;
  magnitude_floor = (uint64_t)(
    sqrtf((resistance_ohms * resistance_ohms) +
          (reactance_ohms * reactance_ohms)) * 20.0f);
  rejection_threshold = median_distance * 3ULL;
  if (rejection_threshold < magnitude_floor)
  {
    rejection_threshold = magnitude_floor;
  }

  for (index = 0U; index < session->eligible_count; ++index)
  {
    const uint64_t resistance_distance = LCR_MeasurementAbsoluteDifference(
      session->eligible_samples[index].resistance_milliohms,
      median_resistance);
    const uint64_t reactance_distance = LCR_MeasurementAbsoluteDifference(
      session->eligible_samples[index].reactance_milliohms,
      median_reactance);
    const uint64_t distance =
      (resistance_distance > reactance_distance) ?
        resistance_distance : reactance_distance;

    if (distance <= rejection_threshold)
    {
      resistance_sum +=
        session->eligible_samples[index].resistance_milliohms;
      reactance_sum +=
        session->eligible_samples[index].reactance_milliohms;
      ++summary->retained_count;
    }
  }
  summary->outlier_count =
    session->eligible_count - summary->retained_count;
  if (summary->retained_count < 3U)
  {
    return;
  }

  summary->average.resistance_milliohms = LCR_MeasurementDivideRounded(
    resistance_sum,
    summary->retained_count);
  summary->average.reactance_milliohms = LCR_MeasurementDivideRounded(
    reactance_sum,
    summary->retained_count);
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
  summary->average_valid = true;
}
