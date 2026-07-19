#ifndef LCR_COMPONENT_H
#define LCR_COMPONENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "lcr_impedance.h"
#include "stm32g4xx_hal.h"

#define LCR_COMPONENT_RESISTIVE_PHASE_LIMIT_MDEG 5000L
#define LCR_COMPONENT_REACTIVE_PHASE_MIN_MDEG   10000L

typedef enum
{
  LCR_COMPONENT_UNKNOWN = 0,
  LCR_COMPONENT_RESISTOR,
  LCR_COMPONENT_CAPACITOR,
  LCR_COMPONENT_INDUCTOR
} LCR_ComponentType;

typedef struct
{
  LCR_ComponentType type;
  int32_t series_resistance_milliohms;
  uint64_t capacitance_picofarads;
  uint64_t inductance_nanohenries;
  uint32_t parameter_frequency_hz;
  bool parameter_valid;
  bool needs_frequency_confirmation;
} LCR_ComponentResult;

HAL_StatusTypeDef LCR_ComponentClassify(
  const LCR_ImpedanceResult *impedance,
  uint32_t frequency_hz,
  LCR_ComponentResult *result);
HAL_StatusTypeDef LCR_ComponentConfirm(
  const LCR_ImpedanceResult *first_impedance,
  uint32_t first_frequency_hz,
  const LCR_ImpedanceResult *second_impedance,
  uint32_t second_frequency_hz,
  LCR_ComponentResult *result);

#ifdef __cplusplus
}
#endif

#endif /* LCR_COMPONENT_H */
