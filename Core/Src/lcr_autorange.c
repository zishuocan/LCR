#include "lcr_autorange.h"

#include <limits.h>
#include <stddef.h>

static uint32_t LCR_AutorangeSensitivity(
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain current_gain)
{
  const LCR_FeedbackNetwork *feedback =
    LCR_GetFeedbackNetwork(feedback_range);
  const uint32_t gain = LCR_GetPgaGainValue(current_gain);

  if ((feedback == NULL) || (gain == 0U))
  {
    return 0U;
  }
  return feedback->resistance_ohms * gain;
}

static bool LCR_AutorangeCandidatePreferred(
  LCR_FeedbackRange candidate_range,
  LCR_FeedbackRange selected_range)
{
  const LCR_FeedbackNetwork *candidate =
    LCR_GetFeedbackNetwork(candidate_range);
  const LCR_FeedbackNetwork *selected =
    LCR_GetFeedbackNetwork(selected_range);

  return (candidate != NULL) && (selected != NULL) &&
         (candidate->resistance_ohms > selected->resistance_ohms);
}

static bool LCR_AutorangeFindCurrentProfile(
  const LCR_RangeStatus *status,
  bool increase,
  uint32_t factor,
  LCR_FeedbackRange *selected_range,
  LCR_PgaGain *selected_gain)
{
  const uint32_t current = LCR_AutorangeSensitivity(
    status->feedback_range, status->current_gain);
  uint32_t target;
  uint32_t best = increase ? UINT_MAX : 0U;
  uint32_t fallback = increase ? 0U : UINT_MAX;
  LCR_FeedbackRange best_range = status->feedback_range;
  LCR_PgaGain best_gain = status->current_gain;
  LCR_FeedbackRange fallback_range = status->feedback_range;
  LCR_PgaGain fallback_gain = status->current_gain;
  bool best_valid = false;
  bool fallback_valid = false;
  uint32_t range_index;

  if ((current == 0U) || (factor < 2U) ||
      (selected_range == NULL) || (selected_gain == NULL))
  {
    return false;
  }

  if (increase)
  {
    target = (current > (UINT_MAX / factor)) ? UINT_MAX : current * factor;
  }
  else
  {
    target = current / factor;
  }

  for (range_index = 0U;
       range_index < (uint32_t)LCR_FEEDBACK_RANGE_COUNT;
       ++range_index)
  {
    const LCR_FeedbackRange range = (LCR_FeedbackRange)range_index;
    uint32_t gain_index;

    if (!LCR_IsFeedbackRangeAvailable(range))
    {
      continue;
    }

    for (gain_index = 0U;
         gain_index <= (uint32_t)LCR_AUTORANGE_MAX_CURRENT_GAIN;
         ++gain_index)
    {
      const LCR_PgaGain gain = (LCR_PgaGain)gain_index;
      const uint32_t sensitivity =
        LCR_AutorangeSensitivity(range, gain);

      if (increase && (sensitivity > current))
      {
        if ((sensitivity >= target) &&
            (!best_valid || (sensitivity < best) ||
             ((sensitivity == best) &&
              LCR_AutorangeCandidatePreferred(range, best_range))))
        {
          best = sensitivity;
          best_range = range;
          best_gain = gain;
          best_valid = true;
        }
        if (!fallback_valid || (sensitivity > fallback) ||
            ((sensitivity == fallback) &&
             LCR_AutorangeCandidatePreferred(range, fallback_range)))
        {
          fallback = sensitivity;
          fallback_range = range;
          fallback_gain = gain;
          fallback_valid = true;
        }
      }
      else if (!increase && (sensitivity < current))
      {
        if ((sensitivity <= target) &&
            (!best_valid || (sensitivity > best) ||
             ((sensitivity == best) &&
              LCR_AutorangeCandidatePreferred(range, best_range))))
        {
          best = sensitivity;
          best_range = range;
          best_gain = gain;
          best_valid = true;
        }
        if (!fallback_valid || (sensitivity < fallback) ||
            ((sensitivity == fallback) &&
             LCR_AutorangeCandidatePreferred(range, fallback_range)))
        {
          fallback = sensitivity;
          fallback_range = range;
          fallback_gain = gain;
          fallback_valid = true;
        }
      }
    }
  }

  if (best_valid)
  {
    *selected_range = best_range;
    *selected_gain = best_gain;
    return true;
  }
  if (fallback_valid)
  {
    *selected_range = fallback_range;
    *selected_gain = fallback_gain;
    return true;
  }
  return false;
}

static HAL_StatusTypeDef LCR_AutorangeApplyCurrentProfile(
  const LCR_RangeStatus *previous,
  LCR_FeedbackRange feedback_range,
  LCR_PgaGain current_gain)
{
  HAL_StatusTypeDef status;

  if (previous->feedback_range == feedback_range)
  {
    return LCR_SetCurrentGain(current_gain);
  }

  /* Minimize the transient while the feedback network is being changed. */
  status = LCR_SetCurrentGain(LCR_PGA_GAIN_1X);
  if (status == HAL_OK)
  {
    status = LCR_SelectFeedbackRange(feedback_range);
  }
  if (status == HAL_OK)
  {
    status = LCR_SetCurrentGain(current_gain);
  }
  return status;
}

HAL_StatusTypeDef LCR_AutorangeBegin(LCR_AutorangeSession *session)
{
  HAL_StatusTypeDef status;

  if (session == NULL)
  {
    return HAL_ERROR;
  }

  session->state = LCR_AUTORANGE_IDLE;
  session->probe_count = 0U;
  session->adjustment_count = 0U;
  session->last_reason_flags = 0U;

  /* Always enter a measurement from a known, lowest-sensitivity setup. */
  status = LCR_SetAmplitude(LCR_EXCITATION_DEFAULT_AMPLITUDE);
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
    status = LCR_SelectFeedbackRange(LCR_FEEDBACK_100_OHM);
  }
  if (status != HAL_OK)
  {
    session->state = LCR_AUTORANGE_FAILED;
    session->last_reason_flags = LCR_AUTORANGE_REASON_APPLY_FAILED;
    return status;
  }

  session->state = LCR_AUTORANGE_SEARCHING;
  return HAL_OK;
}

HAL_StatusTypeDef LCR_AutorangeEvaluate(
  LCR_AutorangeSession *session,
  const LCR_CaptureSummary *capture,
  const LCR_DspCaptureResult *dsp,
  LCR_AutorangeEvaluation *evaluation)
{
  LCR_RangeStatus previous;
  LCR_ExcitationStatus excitation;
  LCR_FeedbackRange selected_feedback;
  LCR_PgaGain selected_voltage_gain;
  LCR_PgaGain selected_current_gain;
  uint32_t reasons = 0U;
  bool voltage_high;
  bool voltage_low;
  bool current_high;
  bool current_low;
  bool reduce_amplitude = false;
  uint32_t amplitude_reduction_factor = 2U;
  uint16_t selected_amplitude;
  bool configuration_possible = true;
  HAL_StatusTypeDef status = HAL_OK;

  if ((session == NULL) || (capture == NULL) || (dsp == NULL) ||
      (evaluation == NULL) ||
      (session->state != LCR_AUTORANGE_SEARCHING))
  {
    return HAL_ERROR;
  }

  LCR_RangeGetStatus(&previous);
  LCR_ExcitationGetStatus(&excitation);
  evaluation->previous_range = previous;
  evaluation->selected_range = previous;
  evaluation->previous_amplitude = excitation.amplitude;
  evaluation->selected_amplitude = excitation.amplitude;
  ++session->probe_count;

  voltage_high = (capture->voltage.near_rail_count != 0U) ||
                 (capture->voltage.peak_to_peak >
                  LCR_AUTORANGE_MAX_P2P_COUNTS);
  voltage_low = !voltage_high &&
                (!dsp->voltage.fundamental_valid ||
                 (capture->voltage.peak_to_peak <
                  LCR_AUTORANGE_MIN_P2P_COUNTS));
  current_high = (capture->current.near_rail_count != 0U) ||
                 (capture->current.peak_to_peak >
                  LCR_AUTORANGE_MAX_P2P_COUNTS);
  current_low = !current_high &&
                (!dsp->current.fundamental_valid ||
                 (capture->current.peak_to_peak <
                  LCR_AUTORANGE_MIN_P2P_COUNTS));

  if (voltage_high)
  {
    reasons |= LCR_AUTORANGE_REASON_V_HIGH;
  }
  else if (voltage_low)
  {
    reasons |= LCR_AUTORANGE_REASON_V_LOW;
  }
  if (current_high)
  {
    reasons |= LCR_AUTORANGE_REASON_I_HIGH;
  }
  else if (current_low)
  {
    reasons |= LCR_AUTORANGE_REASON_I_LOW;
  }

  session->last_reason_flags = reasons;
  evaluation->reason_flags = reasons;
  if (reasons == 0U)
  {
    session->state = LCR_AUTORANGE_LOCKED;
    evaluation->decision = LCR_AUTORANGE_ACCEPTED;
    return HAL_OK;
  }

  if (session->adjustment_count >= LCR_AUTORANGE_MAX_ADJUSTMENTS)
  {
    reasons |= LCR_AUTORANGE_REASON_MAX_ADJUSTMENT;
    session->last_reason_flags = reasons;
    session->state = LCR_AUTORANGE_FAILED;
    evaluation->reason_flags = reasons;
    evaluation->decision = LCR_AUTORANGE_REJECTED;
    return HAL_OK;
  }

  selected_feedback = previous.feedback_range;
  selected_voltage_gain = previous.voltage_gain;
  selected_current_gain = previous.current_gain;
  selected_amplitude = excitation.amplitude;

  if (voltage_low)
  {
    const uint32_t step =
      (capture->voltage.peak_to_peak <
       (LCR_AUTORANGE_MIN_P2P_COUNTS / 4U)) ? 2U : 1U;
    const uint32_t selected = (uint32_t)previous.voltage_gain + step;

    if (selected >= (uint32_t)LCR_PGA_GAIN_COUNT)
    {
      reasons |= LCR_AUTORANGE_REASON_V_LIMIT;
      configuration_possible = false;
    }
    else
    {
      selected_voltage_gain = (LCR_PgaGain)selected;
    }
  }
  else if (voltage_high)
  {
    const uint32_t step =
      (capture->voltage.near_rail_count != 0U) ? 2U : 1U;

    if ((uint32_t)previous.voltage_gain < step)
    {
      reduce_amplitude = true;
      if (capture->voltage.near_rail_count != 0U)
      {
        amplitude_reduction_factor = 4U;
      }
    }
    else
    {
      selected_voltage_gain =
        (LCR_PgaGain)((uint32_t)previous.voltage_gain - step);
    }
  }

  if (current_low)
  {
    const uint32_t factor =
      (!dsp->current.fundamental_valid ||
       (capture->current.peak_to_peak <
        (LCR_AUTORANGE_MIN_P2P_COUNTS / 4U))) ? 4U : 2U;

    if (!LCR_AutorangeFindCurrentProfile(
          &previous, true, factor,
          &selected_feedback, &selected_current_gain))
    {
      reasons |= LCR_AUTORANGE_REASON_I_LIMIT;
      configuration_possible = false;
    }
  }
  else if (current_high)
  {
    const uint32_t factor =
      (capture->current.near_rail_count != 0U) ? 4U : 2U;

    if (!LCR_AutorangeFindCurrentProfile(
          &previous, false, factor,
          &selected_feedback, &selected_current_gain))
    {
      reduce_amplitude = true;
      if (capture->current.near_rail_count != 0U)
      {
        amplitude_reduction_factor = 4U;
      }
    }
  }

  if (reduce_amplitude)
  {
    uint32_t reduced =
      (uint32_t)excitation.amplitude / amplitude_reduction_factor;

    if (reduced < LCR_AUTORANGE_MIN_AMPLITUDE_COUNTS)
    {
      reduced = LCR_AUTORANGE_MIN_AMPLITUDE_COUNTS;
    }
    if (reduced >= (uint32_t)excitation.amplitude)
    {
      if (voltage_high)
      {
        reasons |= LCR_AUTORANGE_REASON_V_LIMIT;
      }
      if (current_high)
      {
        reasons |= LCR_AUTORANGE_REASON_I_LIMIT;
      }
      configuration_possible = false;
    }
    else
    {
      selected_amplitude = (uint16_t)reduced;
    }
  }

  if (!configuration_possible)
  {
    session->last_reason_flags = reasons;
    session->state = LCR_AUTORANGE_FAILED;
    evaluation->reason_flags = reasons;
    evaluation->decision = LCR_AUTORANGE_REJECTED;
    return HAL_OK;
  }

  if (selected_amplitude != excitation.amplitude)
  {
    status = LCR_SetAmplitude(selected_amplitude);
  }
  if ((status == HAL_OK) &&
      (selected_voltage_gain != previous.voltage_gain))
  {
    status = LCR_SetVoltageGain(selected_voltage_gain);
  }
  if (status == HAL_OK)
  {
    status = LCR_AutorangeApplyCurrentProfile(
      &previous, selected_feedback, selected_current_gain);
  }
  if (status != HAL_OK)
  {
    reasons |= LCR_AUTORANGE_REASON_APPLY_FAILED;
    session->last_reason_flags = reasons;
    session->state = LCR_AUTORANGE_FAILED;
    evaluation->reason_flags = reasons;
    evaluation->decision = LCR_AUTORANGE_REJECTED;
    LCR_RangeGetStatus(&evaluation->selected_range);
    return HAL_OK;
  }

  ++session->adjustment_count;
  session->last_reason_flags = reasons;
  evaluation->reason_flags = reasons;
  evaluation->decision = LCR_AUTORANGE_RETRY;
  evaluation->selected_amplitude = selected_amplitude;
  LCR_RangeGetStatus(&evaluation->selected_range);
  return HAL_OK;
}

void LCR_AutorangeCancel(LCR_AutorangeSession *session)
{
  if (session != NULL)
  {
    session->state = LCR_AUTORANGE_IDLE;
  }
}

bool LCR_AutorangeIsSearching(const LCR_AutorangeSession *session)
{
  return (session != NULL) &&
         (session->state == LCR_AUTORANGE_SEARCHING);
}
