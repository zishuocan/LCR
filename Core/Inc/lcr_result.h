#ifndef LCR_RESULT_H
#define LCR_RESULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_component.h"
#include "lcr_measurement.h"
#include "lcr_workflow.h"

typedef enum
{
  LCR_RESULT_EMPTY = 0,
  LCR_RESULT_RUNNING,
  LCR_RESULT_COMPLETE,
  LCR_RESULT_ABORTED,
  LCR_RESULT_ERROR
} LCR_ResultState;

typedef struct
{
  bool present;
  bool complete;
  uint32_t frequency_hz;
  uint32_t autorange_probe_count;
  LCR_MeasurementSummary measurement;
} LCR_ResultMeasurement;

typedef struct
{
  uint32_t sequence_id;
  LCR_ResultState state;
  LCR_WorkflowError error;
  LCR_ResultMeasurement primary;
  LCR_ResultMeasurement secondary;
  bool component_valid;
  LCR_ComponentResult component;
} LCR_FinalResult;

void LCR_ResultInit(void);
void LCR_ResultBegin(void);
void LCR_ResultStoreMeasurement(
  bool secondary,
  uint32_t frequency_hz,
  uint32_t autorange_probe_count,
  const LCR_MeasurementSummary *measurement);
void LCR_ResultComplete(const LCR_ComponentResult *component);
void LCR_ResultAbort(void);
void LCR_ResultFail(LCR_WorkflowError error);
void LCR_ResultGet(LCR_FinalResult *result);
const char *LCR_ResultStateName(LCR_ResultState state);

#ifdef __cplusplus
}
#endif

#endif /* LCR_RESULT_H */
