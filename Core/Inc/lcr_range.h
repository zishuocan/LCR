#ifndef LCR_RANGE_H
#define LCR_RANGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/* Caller should wait at least this long before capturing after a range change. */
#define LCR_RANGE_SETTLING_TIME_MS 2U

/* Values intentionally equal the ADG1609 A1:A0 address code. */
typedef enum
{
  LCR_FEEDBACK_100K_OHM = 0,
  LCR_FEEDBACK_10K_OHM = 1,
  LCR_FEEDBACK_1K_OHM = 2,
  LCR_FEEDBACK_100_OHM = 3,
  LCR_FEEDBACK_RANGE_COUNT
} LCR_FeedbackRange;

/* Values intentionally equal the AD8231 A2:A1:A0 address code. */
typedef enum
{
  LCR_PGA_GAIN_1X = 0,
  LCR_PGA_GAIN_2X = 1,
  LCR_PGA_GAIN_4X = 2,
  LCR_PGA_GAIN_8X = 3,
  LCR_PGA_GAIN_16X = 4,
  LCR_PGA_GAIN_32X = 5,
  LCR_PGA_GAIN_64X = 6,
  LCR_PGA_GAIN_128X = 7,
  LCR_PGA_GAIN_COUNT
} LCR_PgaGain;

typedef struct
{
  uint32_t resistance_ohms;
  uint32_t capacitance_tenths_pf;
} LCR_FeedbackNetwork;

typedef struct
{
  LCR_FeedbackRange requested_feedback_range;
  LCR_FeedbackRange feedback_range;
  LCR_PgaGain voltage_gain;
  LCR_PgaGain current_gain;
  bool feedback_fallback_active;
} LCR_RangeStatus;

HAL_StatusTypeDef LCR_RangeInit(void);
HAL_StatusTypeDef LCR_SetFeedbackRange(LCR_FeedbackRange range);
HAL_StatusTypeDef LCR_SelectFeedbackRange(LCR_FeedbackRange requested_range);
HAL_StatusTypeDef LCR_SetVoltageGain(LCR_PgaGain gain);
HAL_StatusTypeDef LCR_SetCurrentGain(LCR_PgaGain gain);
const LCR_FeedbackNetwork *LCR_GetFeedbackNetwork(LCR_FeedbackRange range);
uint32_t LCR_GetPgaGainValue(LCR_PgaGain gain);
void LCR_RangeGetStatus(LCR_RangeStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* LCR_RANGE_H */
