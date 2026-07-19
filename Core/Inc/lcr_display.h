#ifndef LCR_DISPLAY_H
#define LCR_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_result.h"
#include "stm32g4xx_hal.h"

#define LCR_DISPLAY_PRIMARY_ADDRESS_7BIT   0x3CU
#define LCR_DISPLAY_SECONDARY_ADDRESS_7BIT 0x3DU
#define LCR_DISPLAY_WIDTH                  128U
#define LCR_DISPLAY_HEIGHT                 64U

typedef struct
{
  bool primary_address_found;
  bool secondary_address_found;
  uint8_t detected_address_7bit;
} LCR_DisplayProbeStatus;

HAL_StatusTypeDef LCR_DisplayProbe(LCR_DisplayProbeStatus *status);
HAL_StatusTypeDef LCR_DisplayInit(uint8_t address_7bit);
HAL_StatusTypeDef LCR_DisplayShowTestPattern(void);
HAL_StatusTypeDef LCR_DisplayShowReady(void);
HAL_StatusTypeDef LCR_DisplayShowMeasuring(uint32_t frequency_hz);
HAL_StatusTypeDef LCR_DisplayShowResult(const LCR_FinalResult *result);
bool LCR_DisplayIsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* LCR_DISPLAY_H */
