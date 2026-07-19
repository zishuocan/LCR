#include "lcr_workflow.h"

#include <stddef.h>

static LCR_WorkflowStatus lcr_workflow_status;

void LCR_WorkflowInit(void)
{
  lcr_workflow_status.state = LCR_WORKFLOW_IDLE;
  lcr_workflow_status.last_error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_WorkflowBeginPrimary(void)
{
  lcr_workflow_status.state = LCR_WORKFLOW_AUTORANGE_PRIMARY;
  lcr_workflow_status.last_error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_WorkflowBeginConfirmation(void)
{
  lcr_workflow_status.state = LCR_WORKFLOW_AUTORANGE_CONFIRMATION;
  lcr_workflow_status.last_error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_WorkflowBeginAveraging(void)
{
  if (lcr_workflow_status.state == LCR_WORKFLOW_AUTORANGE_CONFIRMATION)
  {
    lcr_workflow_status.state = LCR_WORKFLOW_AVERAGING_CONFIRMATION;
  }
  else
  {
    lcr_workflow_status.state = LCR_WORKFLOW_AVERAGING_PRIMARY;
  }
}

void LCR_WorkflowComplete(void)
{
  lcr_workflow_status.state = LCR_WORKFLOW_COMPLETE;
  lcr_workflow_status.last_error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_WorkflowAbort(void)
{
  lcr_workflow_status.state = LCR_WORKFLOW_ABORTED;
  lcr_workflow_status.last_error = LCR_WORKFLOW_ERROR_NONE;
}

void LCR_WorkflowFail(LCR_WorkflowError error)
{
  lcr_workflow_status.state = LCR_WORKFLOW_ERROR;
  lcr_workflow_status.last_error = error;
}

void LCR_WorkflowGetStatus(LCR_WorkflowStatus *status)
{
  if (status != NULL)
  {
    *status = lcr_workflow_status;
  }
}

bool LCR_WorkflowIsConfirmation(void)
{
  return (lcr_workflow_status.state ==
          LCR_WORKFLOW_AUTORANGE_CONFIRMATION) ||
         (lcr_workflow_status.state ==
          LCR_WORKFLOW_AVERAGING_CONFIRMATION);
}

const char *LCR_WorkflowStateName(LCR_WorkflowState state)
{
  switch (state)
  {
    case LCR_WORKFLOW_IDLE: return "IDLE";
    case LCR_WORKFLOW_AUTORANGE_PRIMARY: return "AUTORANGE_PRIMARY";
    case LCR_WORKFLOW_AVERAGING_PRIMARY: return "AVERAGING_PRIMARY";
    case LCR_WORKFLOW_AUTORANGE_CONFIRMATION:
      return "AUTORANGE_CONFIRMATION";
    case LCR_WORKFLOW_AVERAGING_CONFIRMATION:
      return "AVERAGING_CONFIRMATION";
    case LCR_WORKFLOW_COMPLETE: return "COMPLETE";
    case LCR_WORKFLOW_ABORTED: return "ABORTED";
    case LCR_WORKFLOW_ERROR: return "ERROR";
    default: return "UNKNOWN_STATE";
  }
}

const char *LCR_WorkflowErrorName(LCR_WorkflowError error)
{
  switch (error)
  {
    case LCR_WORKFLOW_ERROR_NONE: return "NONE";
    case LCR_WORKFLOW_ERROR_ADC_START: return "ADC_START";
    case LCR_WORKFLOW_ERROR_ADC_TIMEOUT: return "ADC_TIMEOUT";
    case LCR_WORKFLOW_ERROR_ADC_PROCESS: return "ADC_PROCESS";
    case LCR_WORKFLOW_ERROR_EXCITATION_CONTROL:
      return "EXCITATION_CONTROL";
    case LCR_WORKFLOW_ERROR_DFT: return "DFT";
    case LCR_WORKFLOW_ERROR_AUTORANGE: return "AUTORANGE";
    case LCR_WORKFLOW_ERROR_NO_DUT_OR_OPEN: return "NO_DUT_OR_OPEN";
    case LCR_WORKFLOW_ERROR_NO_ELIGIBLE_SAMPLES:
      return "NO_ELIGIBLE_SAMPLES";
    case LCR_WORKFLOW_ERROR_CONFIRMATION_START:
      return "CONFIRMATION_START";
    case LCR_WORKFLOW_ERROR_CONFIRMATION_INVALID:
      return "CONFIRMATION_INVALID";
    default: return "UNKNOWN_ERROR";
  }
}
