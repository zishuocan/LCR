#ifndef LCR_WORKFLOW_H
#define LCR_WORKFLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum
{
  LCR_WORKFLOW_IDLE = 0,
  LCR_WORKFLOW_AUTORANGE_PRIMARY,
  LCR_WORKFLOW_AVERAGING_PRIMARY,
  LCR_WORKFLOW_AUTORANGE_CONFIRMATION,
  LCR_WORKFLOW_AVERAGING_CONFIRMATION,
  LCR_WORKFLOW_COMPLETE,
  LCR_WORKFLOW_ABORTED,
  LCR_WORKFLOW_ERROR
} LCR_WorkflowState;

typedef enum
{
  LCR_WORKFLOW_ERROR_NONE = 0,
  LCR_WORKFLOW_ERROR_ADC_START,
  LCR_WORKFLOW_ERROR_ADC_TIMEOUT,
  LCR_WORKFLOW_ERROR_ADC_PROCESS,
  LCR_WORKFLOW_ERROR_EXCITATION_CONTROL,
  LCR_WORKFLOW_ERROR_DFT,
  LCR_WORKFLOW_ERROR_AUTORANGE,
  LCR_WORKFLOW_ERROR_NO_DUT_OR_OPEN,
  LCR_WORKFLOW_ERROR_NO_ELIGIBLE_SAMPLES,
  LCR_WORKFLOW_ERROR_CONFIRMATION_START,
  LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID
} LCR_WorkflowError;

typedef struct
{
  LCR_WorkflowState state;
  LCR_WorkflowError last_error;
} LCR_WorkflowStatus;

void LCR_WorkflowInit(void);
void LCR_WorkflowBeginPrimary(void);
void LCR_WorkflowBeginConfirmation(void);
void LCR_WorkflowBeginAveraging(void);
void LCR_WorkflowComplete(void);
void LCR_WorkflowAbort(void);
void LCR_WorkflowFail(LCR_WorkflowError error);
void LCR_WorkflowGetStatus(LCR_WorkflowStatus *status);
bool LCR_WorkflowIsConfirmation(void);
const char *LCR_WorkflowStateName(LCR_WorkflowState state);
const char *LCR_WorkflowErrorName(LCR_WorkflowError error);

#ifdef __cplusplus
}
#endif

#endif /* LCR_WORKFLOW_H */
