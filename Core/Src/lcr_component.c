#include "lcr_component.h"

#include <math.h>
#include <stddef.h>

#define LCR_COMPONENT_PI 3.14159265358979323846

static uint64_t LCR_ComponentRoundPositive(double value)
{
  if (value <= 0.0)
  {
    return 0ULL;
  }
  return (uint64_t)(value + 0.5);
}

static uint64_t LCR_ComponentAbsoluteInt32(int32_t value)
{
  return (value < 0) ? (uint64_t)(-(int64_t)value) : (uint64_t)value;
}

static bool LCR_ComponentWithinTenPercent(uint64_t first, uint64_t second)
{
  const uint64_t maximum = (first > second) ? first : second;
  const uint64_t difference = (first > second) ?
    first - second : second - first;

  return (maximum != 0ULL) &&
         ((difference * 100ULL) <= (maximum * 10ULL));
}

static void LCR_ComponentInitializeResult(
  const LCR_ImpedanceResult *impedance,
  LCR_ComponentResult *result)
{
  result->type = LCR_COMPONENT_UNKNOWN;
  result->series_resistance_milliohms =
    impedance->resistance_milliohms;
  result->capacitance_picofarads = 0ULL;
  result->inductance_nanohenries = 0ULL;
  result->parameter_frequency_hz = 0U;
  result->parameter_valid = false;
  result->needs_frequency_confirmation = true;
}

static void LCR_ComponentSetCapacitor(
  const LCR_ImpedanceResult *impedance,
  uint32_t frequency_hz,
  LCR_ComponentResult *result)
{
  const double capacitance_picofarads = 1.0e15 /
    (2.0 * LCR_COMPONENT_PI * (double)frequency_hz *
     (double)LCR_ComponentAbsoluteInt32(
       impedance->reactance_milliohms));

  LCR_ComponentInitializeResult(impedance, result);
  result->type = LCR_COMPONENT_CAPACITOR;
  result->capacitance_picofarads =
    LCR_ComponentRoundPositive(capacitance_picofarads);
  result->parameter_frequency_hz = frequency_hz;
  result->parameter_valid = result->capacitance_picofarads != 0ULL;
  result->needs_frequency_confirmation = false;
}

static void LCR_ComponentSetInductor(
  const LCR_ImpedanceResult *impedance,
  uint32_t frequency_hz,
  LCR_ComponentResult *result)
{
  const double inductance_nanohenries =
    ((double)impedance->reactance_milliohms * 1.0e6) /
    (2.0 * LCR_COMPONENT_PI * (double)frequency_hz);

  LCR_ComponentInitializeResult(impedance, result);
  result->type = LCR_COMPONENT_INDUCTOR;
  result->inductance_nanohenries =
    LCR_ComponentRoundPositive(inductance_nanohenries);
  result->parameter_frequency_hz = frequency_hz;
  result->parameter_valid = result->inductance_nanohenries != 0ULL;
  result->needs_frequency_confirmation = false;
}

HAL_StatusTypeDef LCR_ComponentClassify(
  const LCR_ImpedanceResult *impedance,
  uint32_t frequency_hz,
  LCR_ComponentResult *result)
{
  const int32_t absolute_phase =
    (impedance != NULL) && (impedance->phase_millidegrees < 0) ?
      -impedance->phase_millidegrees :
      ((impedance != NULL) ? impedance->phase_millidegrees : 0L);

  if ((impedance == NULL) || (result == NULL) ||
      !impedance->valid || (frequency_hz == 0U))
  {
    return HAL_ERROR;
  }

  LCR_ComponentInitializeResult(impedance, result);

  if (absolute_phase <= LCR_COMPONENT_RESISTIVE_PHASE_LIMIT_MDEG)
  {
    result->type = LCR_COMPONENT_RESISTOR;
    result->parameter_frequency_hz = frequency_hz;
    result->parameter_valid = true;
    result->needs_frequency_confirmation = false;
  }
  else if ((impedance->phase_millidegrees <=
            -LCR_COMPONENT_REACTIVE_PHASE_MIN_MDEG) &&
           (impedance->reactance_milliohms < 0))
  {
    LCR_ComponentSetCapacitor(impedance, frequency_hz, result);
  }
  else if ((impedance->phase_millidegrees >=
            LCR_COMPONENT_REACTIVE_PHASE_MIN_MDEG) &&
           (impedance->reactance_milliohms > 0))
  {
    LCR_ComponentSetInductor(impedance, frequency_hz, result);
  }

  return HAL_OK;
}

HAL_StatusTypeDef LCR_ComponentConfirm(
  const LCR_ImpedanceResult *first_impedance,
  uint32_t first_frequency_hz,
  const LCR_ImpedanceResult *second_impedance,
  uint32_t second_frequency_hz,
  LCR_ComponentResult *result)
{
  LCR_ComponentResult first_component;
  LCR_ComponentResult second_component;
  uint64_t first_abs_phase;
  uint64_t second_abs_phase;
  uint64_t first_abs_reactance;
  uint64_t second_abs_reactance;
  uint64_t first_resistance;
  uint64_t second_resistance;
  uint64_t resistance_maximum;
  uint64_t resistance_difference;

  if ((first_impedance == NULL) || (second_impedance == NULL) ||
      (result == NULL) || !first_impedance->valid ||
      !second_impedance->valid || (first_frequency_hz == 0U) ||
      (second_frequency_hz == 0U) ||
      (first_frequency_hz == second_frequency_hz))
  {
    return HAL_ERROR;
  }

  LCR_ComponentInitializeResult(first_impedance, result);
  first_abs_phase = LCR_ComponentAbsoluteInt32(
    first_impedance->phase_millidegrees);
  second_abs_phase = LCR_ComponentAbsoluteInt32(
    second_impedance->phase_millidegrees);
  first_abs_reactance = LCR_ComponentAbsoluteInt32(
    first_impedance->reactance_milliohms);
  second_abs_reactance = LCR_ComponentAbsoluteInt32(
    second_impedance->reactance_milliohms);
  first_resistance = LCR_ComponentAbsoluteInt32(
    first_impedance->resistance_milliohms);
  second_resistance = LCR_ComponentAbsoluteInt32(
    second_impedance->resistance_milliohms);
  resistance_maximum = (first_resistance > second_resistance) ?
    first_resistance : second_resistance;
  resistance_difference = (first_resistance > second_resistance) ?
    first_resistance - second_resistance :
    second_resistance - first_resistance;

  /* A resistor remains R-dominant and nearly constant across both frequencies. */
  if ((first_impedance->resistance_milliohms > 0) &&
      (second_impedance->resistance_milliohms > 0) &&
      (first_abs_phase <= LCR_COMPONENT_REACTIVE_PHASE_MIN_MDEG) &&
      (second_abs_phase <= LCR_COMPONENT_REACTIVE_PHASE_MIN_MDEG) &&
      ((first_abs_reactance * 4ULL) <= first_resistance) &&
      ((second_abs_reactance * 4ULL) <= second_resistance) &&
      ((resistance_difference * 100ULL) <=
       (resistance_maximum * 20ULL)))
  {
    result->type = LCR_COMPONENT_RESISTOR;
    result->series_resistance_milliohms =
      first_impedance->resistance_milliohms;
    result->parameter_frequency_hz = first_frequency_hz;
    result->parameter_valid = true;
    result->needs_frequency_confirmation = false;
    return HAL_OK;
  }

  /* Confirm C from each frequency independently; retain the 10 kHz primary. */
  if ((first_impedance->reactance_milliohms < 0) &&
      (second_impedance->reactance_milliohms < 0))
  {
    LCR_ComponentSetCapacitor(
      first_impedance, first_frequency_hz, &first_component);
    LCR_ComponentSetCapacitor(
      second_impedance, second_frequency_hz, &second_component);
    if (first_component.parameter_valid && second_component.parameter_valid &&
        LCR_ComponentWithinTenPercent(
          first_component.capacitance_picofarads,
          second_component.capacitance_picofarads))
    {
      *result = first_component;
    }
    return HAL_OK;
  }

  /* Confirm L from each frequency independently; retain the 10 kHz primary. */
  if ((first_impedance->reactance_milliohms > 0) &&
      (second_impedance->reactance_milliohms > 0))
  {
    LCR_ComponentSetInductor(
      first_impedance, first_frequency_hz, &first_component);
    LCR_ComponentSetInductor(
      second_impedance, second_frequency_hz, &second_component);
    if (first_component.parameter_valid && second_component.parameter_valid &&
        LCR_ComponentWithinTenPercent(
          first_component.inductance_nanohenries,
          second_component.inductance_nanohenries))
    {
      *result = first_component;
    }
  }

  return HAL_OK;
}
