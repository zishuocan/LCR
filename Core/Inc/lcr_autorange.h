#ifndef LCR_AUTORANGE_H
#define LCR_AUTORANGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_capture.h"
#include "lcr_dsp.h"
#include "lcr_excitation.h"
#include "lcr_range.h"
#include "stm32g4xx_hal.h"

/* A 20% lower target leaves THD/SNR margin; 80% remains the hard upper limit. */
#define LCR_AUTORANGE_MIN_P2P_COUNTS 820U
#define LCR_AUTORANGE_MAX_P2P_COUNTS 3276U
#define LCR_AUTORANGE_MAX_ADJUSTMENTS 12U
#define LCR_AUTORANGE_MIN_AMPLITUDE_COUNTS 150U
#define LCR_AUTORANGE_MAX_CURRENT_GAIN LCR_PGA_GAIN_128X

#define LCR_AUTORANGE_REASON_V_LOW          (1UL << 0)
#define LCR_AUTORANGE_REASON_V_HIGH         (1UL << 1)
#define LCR_AUTORANGE_REASON_I_LOW          (1UL << 2)
#define LCR_AUTORANGE_REASON_I_HIGH         (1UL << 3)
#define LCR_AUTORANGE_REASON_V_LIMIT        (1UL << 4)
#define LCR_AUTORANGE_REASON_I_LIMIT        (1UL << 5)
#define LCR_AUTORANGE_REASON_APPLY_FAILED   (1UL << 6)
#define LCR_AUTORANGE_REASON_MAX_ADJUSTMENT (1UL << 7)

typedef enum
{
  LCR_AUTORANGE_IDLE = 0,
  LCR_AUTORANGE_SEARCHING,
  LCR_AUTORANGE_LOCKED,
  LCR_AUTORANGE_FAILED
} LCR_AutorangeState;

typedef enum
{
  LCR_AUTORANGE_RETRY = 0,
  LCR_AUTORANGE_ACCEPTED,
  LCR_AUTORANGE_REJECTED
} LCR_AutorangeDecision;

typedef struct
{
  LCR_AutorangeState state;
  uint32_t probe_count;
  uint32_t adjustment_count;
  uint32_t last_reason_flags;
} LCR_AutorangeSession;

typedef struct
{
  LCR_AutorangeDecision decision;
  uint32_t reason_flags;
  uint16_t previous_amplitude;
  uint16_t selected_amplitude;
  LCR_RangeStatus previous_range;
  LCR_RangeStatus selected_range;
} LCR_AutorangeEvaluation;

HAL_StatusTypeDef LCR_AutorangeBegin(LCR_AutorangeSession *session);
HAL_StatusTypeDef LCR_AutorangeEvaluate(
  LCR_AutorangeSession *session,
  const LCR_CaptureSummary *capture,
  const LCR_DspCaptureResult *dsp,
  LCR_AutorangeEvaluation *evaluation);
void LCR_AutorangeCancel(LCR_AutorangeSession *session);
bool LCR_AutorangeIsSearching(const LCR_AutorangeSession *session);

#ifdef __cplusplus
}
#endif

#endif /* LCR_AUTORANGE_H */
