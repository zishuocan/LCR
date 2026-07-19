#include "lcr_result.h"

#include <stddef.h>
#include <string.h>

static LCR_FinalResult lcr_final_result;

void LCR_ResultInit(void)
{
  memset(&lcr_final_result, 0, sizeof(lcr_final_result));
  lcr_final_result.state = LCR_RESULT_EMPTY;
  lcr_final_result.error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_ResultBegin(void)
{
  uint32_t next_sequence = lcr_final_result.sequence_id + 1U;

  if (next_sequence == 0U)
  {
    next_sequence = 1U;
  }
  memset(&lcr_final_result, 0, sizeof(lcr_final_result));
  lcr_final_result.sequence_id = next_sequence;
  lcr_final_result.state = LCR_RESULT_RUNNING;
  lcr_final_result.error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_ResultStoreMeasurement(
  bool secondary,
  uint32_t frequency_hz,
  uint32_t autorange_probe_count,
  const LCR_MeasurementSummary *measurement)
{
  LCR_ResultMeasurement *destination;

  if (measurement == NULL)
  {
    return;
  }

  destination = secondary ?
    &lcr_final_result.secondary : &lcr_final_result.primary;
  destination->present =
    (measurement->attempt_count != 0U) ||
    (autorange_probe_count != 0U);
  destination->complete =
    measurement->attempt_count >= LCR_MEASUREMENT_ATTEMPT_COUNT;
  destination->frequency_hz = frequency_hz;
  destination->autorange_probe_count = autorange_probe_count;
  destination->measurement = *measurement;
}

void LCR_ResultComplete(const LCR_ComponentResult *component)
{
  if (component == NULL)
  {
    return;
  }

  lcr_final_result.component = *component;
  lcr_final_result.component_valid = true;
  lcr_final_result.state = LCR_RESULT_COMPLETE;
  lcr_final_result.error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_ResultAbort(void)
{
  lcr_final_result.state = LCR_RESULT_ABORTED;
  lcr_final_result.error = LCR_WORKFLOW_ERROR_NONE;
  lcr_final_result.component_valid = false;
}

void LCR_ResultFail(LCR_WorkflowError error)
{
  lcr_final_result.state = LCR_RESULT_ERROR;
  lcr_final_result.error = error;
  lcr_final_result.component_valid = false;
}

void LCR_ResultGet(LCR_FinalResult *result)
{
  if (result != NULL)
  {
    *result = lcr_final_result;
  }
}

const char *LCR_ResultStateName(LCR_ResultState state)
{
  switch (state)
  {
    case LCR_RESULT_EMPTY: return "EMPTY";
    case LCR_RESULT_RUNNING: return "RUNNING";
    case LCR_RESULT_COMPLETE: return "COMPLETE";
    case LCR_RESULT_ABORTED: return "ABORTED";
    case LCR_RESULT_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}
