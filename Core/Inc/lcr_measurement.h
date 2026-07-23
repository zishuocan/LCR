#ifndef LCR_MEASUREMENT_H
#define LCR_MEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_impedance.h"
#include "stm32g4xx_hal.h"

#define LCR_MEASUREMENT_MIN_VALID_COUNT 5U
#define LCR_MEASUREMENT_MAX_ATTEMPT_COUNT 16U

typedef struct
{
  uint32_t attempt_count;
  uint32_t eligible_count;
  uint32_t ineligible_count;
  uint32_t retained_count;
  uint32_t outlier_count;
  bool average_valid;
  bool average_is_calibrated;
  LCR_ImpedanceResult average;
} LCR_MeasurementSummary;

typedef struct
{
  uint32_t attempt_count;
  uint32_t eligible_count;
  LCR_ImpedanceResult eligible_samples[LCR_MEASUREMENT_MAX_ATTEMPT_COUNT];
  bool active;
  bool result_source_set;
  bool result_is_calibrated;
} LCR_MeasurementSession;

void LCR_MeasurementBegin(LCR_MeasurementSession *session);
void LCR_MeasurementCancel(LCR_MeasurementSession *session);
HAL_StatusTypeDef LCR_MeasurementRecord(
  LCR_MeasurementSession *session,
  const LCR_ImpedanceResult *result,
  bool eligible,
  bool result_is_calibrated);
bool LCR_MeasurementIsActive(const LCR_MeasurementSession *session);
bool LCR_MeasurementIsComplete(const LCR_MeasurementSession *session);
void LCR_MeasurementGetSummary(
  const LCR_MeasurementSession *session,
  LCR_MeasurementSummary *summary);

#ifdef __cplusplus
}
#endif

#endif /* LCR_MEASUREMENT_H */
