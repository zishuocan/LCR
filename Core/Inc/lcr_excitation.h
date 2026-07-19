#ifndef LCR_EXCITATION_H
#define LCR_EXCITATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#define LCR_EXCITATION_TABLE_SIZE          64U
#define LCR_EXCITATION_MEDIUM_TABLE_SIZE   32U
#define LCR_EXCITATION_HIGH_TABLE_SIZE     16U
#define LCR_EXCITATION_MAX_TRIGGER_RATE_HZ 800000U
#define LCR_EXCITATION_HIGH_FREQUENCY_HZ   50000U
#define LCR_EXCITATION_DAC_MIDPOINT        2048U
#define LCR_EXCITATION_MAX_AMPLITUDE       2047U
#define LCR_EXCITATION_DEFAULT_AMPLITUDE   1200U
#define LCR_EXCITATION_DEFAULT_FREQUENCY_HZ 10000U

typedef struct
{
  uint32_t requested_frequency_hz;
  uint32_t actual_frequency_hz;
  uint32_t samples_per_period;
  uint16_t amplitude;
  bool running;
} LCR_ExcitationStatus;

HAL_StatusTypeDef LCR_ExcitationInit(void);
HAL_StatusTypeDef LCR_SetFrequency(uint32_t frequency_hz);
HAL_StatusTypeDef LCR_SetAmplitude(uint16_t amplitude);
HAL_StatusTypeDef LCR_ExcitationStart(void);
HAL_StatusTypeDef LCR_ExcitationStop(void);
void LCR_ExcitationGetStatus(LCR_ExcitationStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* LCR_EXCITATION_H */
