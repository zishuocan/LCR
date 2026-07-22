#ifndef LCR_100K_DIAGNOSTIC_H
#define LCR_100K_DIAGNOSTIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "stm32g4xx_hal.h"

/* Runs once during startup; reference validation is enabled only on demand. */
HAL_StatusTypeDef LCR_100kDiagnosticRun(bool run_reference_validation);

#ifdef __cplusplus
}
#endif

#endif /* LCR_100K_DIAGNOSTIC_H */
