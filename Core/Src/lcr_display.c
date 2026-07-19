#include "lcr_display.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"

#define LCR_DISPLAY_PROBE_TRIALS     2U
#define LCR_DISPLAY_PROBE_TIMEOUT_MS 5U
#define LCR_DISPLAY_IO_TIMEOUT_MS    50U
#define LCR_DISPLAY_PAGE_COUNT       (LCR_DISPLAY_HEIGHT / 8U)
#define LCR_DISPLAY_CONTROL_COMMAND  0x00U
#define LCR_DISPLAY_CONTROL_DATA     0x40U

static uint8_t lcr_display_address_7bit;
static bool lcr_display_ready;
static uint8_t lcr_display_framebuffer[
  LCR_DISPLAY_WIDTH * LCR_DISPLAY_PAGE_COUNT];

static HAL_StatusTypeDef LCR_DisplayWrite(
  const uint8_t *data,
  uint16_t length)
{
  if ((data == NULL) || (length == 0U) ||
      (lcr_display_address_7bit == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Transmit(
    &hi2c1,
    (uint16_t)lcr_display_address_7bit << 1U,
    (uint8_t *)(uintptr_t)data,
    length,
    LCR_DISPLAY_IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef LCR_DisplaySetAddressWindow(void)
{
  static const uint8_t address_commands[] =
  {
    LCR_DISPLAY_CONTROL_COMMAND,
    0x21U, 0x00U, 0x7FU,
    0x22U, 0x00U, 0x07U
  };

  return LCR_DisplayWrite(
    address_commands,
    (uint16_t)sizeof(address_commands));
}

static void LCR_DisplayGetGlyph(char character, uint8_t glyph[5])
{
  static const uint8_t unknown[5] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
  const uint8_t *source = unknown;

  static const uint8_t space[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t dot[5] = {0U, 0x60U, 0x60U, 0U, 0U};
  static const uint8_t colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
  static const uint8_t dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t plus[5] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
  static const uint8_t equal[5] = {0x14U, 0x14U, 0x14U, 0x14U, 0x14U};
  static const uint8_t slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
  static const uint8_t hash[5] = {0x14U, 0x7FU, 0x14U, 0x7FU, 0x14U};
  static const uint8_t digits[10][5] =
  {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
  };
  static const uint8_t letters[26][5] =
  {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
  };

  if (character == ' ')
  {
    source = space;
  }
  else if ((character >= '0') && (character <= '9'))
  {
    source = digits[(uint32_t)(character - '0')];
  }
  else if ((character >= 'A') && (character <= 'Z'))
  {
    source = letters[(uint32_t)(character - 'A')];
  }
  else
  {
    switch (character)
    {
      case '.': source = dot; break;
      case ':': source = colon; break;
      case '-': source = dash; break;
      case '+': source = plus; break;
      case '=': source = equal; break;
      case '/': source = slash; break;
      case '#': source = hash; break;
      default: break;
    }
  }
  memcpy(glyph, source, 5U);
}

static void LCR_DisplayClearBuffer(void)
{
  memset(lcr_display_framebuffer, 0, sizeof(lcr_display_framebuffer));
}

static void LCR_DisplayDrawText(uint32_t row, const char *text)
{
  uint32_t column = 0U;
  uint32_t framebuffer_index;

  if ((row >= LCR_DISPLAY_PAGE_COUNT) || (text == NULL))
  {
    return;
  }

  framebuffer_index = row * LCR_DISPLAY_WIDTH;
  while ((*text != '\0') && ((column + 5U) < LCR_DISPLAY_WIDTH))
  {
    uint8_t glyph[5];
    uint32_t glyph_column;
    char character = *text++;

    if ((character >= 'a') && (character <= 'z'))
    {
      character = (char)(character - ('a' - 'A'));
    }
    LCR_DisplayGetGlyph(character, glyph);
    for (glyph_column = 0U; glyph_column < 5U; ++glyph_column)
    {
      lcr_display_framebuffer[framebuffer_index + column++] =
        glyph[glyph_column];
    }
    lcr_display_framebuffer[framebuffer_index + column++] = 0U;
  }
}

static HAL_StatusTypeDef LCR_DisplayFlush(void)
{
  uint8_t page_buffer[LCR_DISPLAY_WIDTH + 1U];
  uint32_t page;
  HAL_StatusTypeDef status;

  if (!lcr_display_ready)
  {
    return HAL_ERROR;
  }

  status = LCR_DisplaySetAddressWindow();
  for (page = 0U;
       (page < LCR_DISPLAY_PAGE_COUNT) && (status == HAL_OK);
       ++page)
  {
    page_buffer[0] = LCR_DISPLAY_CONTROL_DATA;
    memcpy(
      &page_buffer[1],
      &lcr_display_framebuffer[page * LCR_DISPLAY_WIDTH],
      LCR_DISPLAY_WIDTH);
    status = LCR_DisplayWrite(
      page_buffer,
      (uint16_t)sizeof(page_buffer));
  }
  return status;
}

static void LCR_DisplayFormatOhms(
  char *text,
  size_t text_size,
  int32_t value_milliohms)
{
  const bool negative = value_milliohms < 0;
  const uint32_t absolute_milliohms = negative ?
    (uint32_t)(-(int64_t)value_milliohms) :
    (uint32_t)value_milliohms;

  if (absolute_milliohms >= 1000000U)
  {
    (void)snprintf(
      text,
      text_size,
      "%s%lu.%03luK",
      negative ? "-" : "",
      (unsigned long)(absolute_milliohms / 1000000U),
      (unsigned long)((absolute_milliohms / 1000U) % 1000U));
  }
  else
  {
    (void)snprintf(
      text,
      text_size,
      "%s%lu.%03lu",
      negative ? "-" : "",
      (unsigned long)(absolute_milliohms / 1000U),
      (unsigned long)(absolute_milliohms % 1000U));
  }
}

static void LCR_DisplayFormatFrequency(
  char *text,
  size_t text_size,
  uint32_t frequency_hz)
{
  if ((frequency_hz >= 1000U) && ((frequency_hz % 1000U) == 0U))
  {
    (void)snprintf(
      text, text_size, "%luKHZ", (unsigned long)(frequency_hz / 1000U));
  }
  else
  {
    (void)snprintf(text, text_size, "%luHZ", (unsigned long)frequency_hz);
  }
}

static void LCR_DisplayFormatCapacitance(
  char *text,
  size_t text_size,
  uint64_t capacitance_picofarads)
{
  uint64_t integer_part;
  uint64_t fractional_part;

  if (capacitance_picofarads >= 1000000ULL)
  {
    integer_part = capacitance_picofarads / 1000000ULL;
    fractional_part =
      (capacitance_picofarads % 1000000ULL) / 1000ULL;
    if (integer_part > 999999ULL)
    {
      (void)snprintf(text, text_size, "C=OVER RANGE");
    }
    else
    {
      (void)snprintf(
        text,
        text_size,
        "C=%lu.%03luUF",
        (unsigned long)integer_part,
        (unsigned long)fractional_part);
    }
  }
  else if (capacitance_picofarads >= 1000ULL)
  {
    integer_part = capacitance_picofarads / 1000ULL;
    fractional_part = capacitance_picofarads % 1000ULL;
    (void)snprintf(
      text,
      text_size,
      "C=%lu.%03luNF",
      (unsigned long)integer_part,
      (unsigned long)fractional_part);
  }
  else
  {
    (void)snprintf(
      text,
      text_size,
      "C=%luPF",
      (unsigned long)capacitance_picofarads);
  }
}

static void LCR_DisplayFormatInductance(
  char *text,
  size_t text_size,
  uint64_t inductance_nanohenries)
{
  uint64_t integer_part;
  uint64_t fractional_part;

  if (inductance_nanohenries >= 1000000ULL)
  {
    integer_part = inductance_nanohenries / 1000000ULL;
    fractional_part =
      (inductance_nanohenries % 1000000ULL) / 1000ULL;
    if (integer_part > 999999ULL)
    {
      (void)snprintf(text, text_size, "L=OVER RANGE");
    }
    else
    {
      (void)snprintf(
        text,
        text_size,
        "L=%lu.%03luMH",
        (unsigned long)integer_part,
        (unsigned long)fractional_part);
    }
  }
  else if (inductance_nanohenries >= 1000ULL)
  {
    integer_part = inductance_nanohenries / 1000ULL;
    fractional_part = inductance_nanohenries % 1000ULL;
    (void)snprintf(
      text,
      text_size,
      "L=%lu.%03luUH",
      (unsigned long)integer_part,
      (unsigned long)fractional_part);
  }
  else
  {
    (void)snprintf(
      text,
      text_size,
      "L=%luNH",
      (unsigned long)inductance_nanohenries);
  }
}

static void LCR_DisplayDrawMeasurement(
  const LCR_FinalResult *result,
  const LCR_ResultMeasurement *measurement)
{
  char frequency[12];
  char resistance[14];
  char reactance[14];
  char line[32];
  int32_t absolute_reactance;

  LCR_DisplayFormatFrequency(
    frequency, sizeof(frequency), measurement->frequency_hz);
  LCR_DisplayFormatOhms(
    resistance,
    sizeof(resistance),
    measurement->measurement.average.resistance_milliohms);
  absolute_reactance =
    measurement->measurement.average.reactance_milliohms;
  if (absolute_reactance < 0)
  {
    absolute_reactance = (absolute_reactance == INT32_MIN) ?
      INT32_MAX : -absolute_reactance;
  }
  LCR_DisplayFormatOhms(
    reactance, sizeof(reactance), absolute_reactance);
  (void)snprintf(
    line,
    sizeof(line),
    "Z=%s%cJ%s",
    resistance,
    (measurement->measurement.average.reactance_milliohms < 0) ? '-' : '+',
    reactance);
  LCR_DisplayDrawText(1U, line);

  switch (result->component.type)
  {
    case LCR_COMPONENT_RESISTOR:
      (void)snprintf(line, sizeof(line), "TYPE=R F=%s", frequency);
      LCR_DisplayDrawText(2U, line);
      LCR_DisplayFormatOhms(
        resistance,
        sizeof(resistance),
        result->component.series_resistance_milliohms);
      (void)snprintf(line, sizeof(line), "R=%s OHM", resistance);
      LCR_DisplayDrawText(3U, line);
      (void)snprintf(
        line,
        sizeof(line),
        "PHASE=%ld.%03lu DEG",
        (long)(measurement->measurement.average.phase_millidegrees / 1000L),
        (unsigned long)((measurement->measurement.average.phase_millidegrees < 0 ?
          -(int64_t)measurement->measurement.average.phase_millidegrees :
          measurement->measurement.average.phase_millidegrees) % 1000L));
      LCR_DisplayDrawText(4U, line);
      break;

    case LCR_COMPONENT_CAPACITOR:
      (void)snprintf(line, sizeof(line), "TYPE=C F=%s", frequency);
      LCR_DisplayDrawText(2U, line);
      LCR_DisplayFormatCapacitance(
        line,
        sizeof(line),
        result->component.capacitance_picofarads);
      LCR_DisplayDrawText(3U, line);
      LCR_DisplayFormatOhms(
        resistance,
        sizeof(resistance),
        result->component.series_resistance_milliohms);
      (void)snprintf(line, sizeof(line), "ESR=%s OHM", resistance);
      LCR_DisplayDrawText(4U, line);
      break;

    case LCR_COMPONENT_INDUCTOR:
      (void)snprintf(line, sizeof(line), "TYPE=L F=%s", frequency);
      LCR_DisplayDrawText(2U, line);
      LCR_DisplayFormatInductance(
        line,
        sizeof(line),
        result->component.inductance_nanohenries);
      LCR_DisplayDrawText(3U, line);
      LCR_DisplayFormatOhms(
        resistance,
        sizeof(resistance),
        result->component.series_resistance_milliohms);
      (void)snprintf(line, sizeof(line), "RS=%s OHM", resistance);
      LCR_DisplayDrawText(4U, line);
      break;

    case LCR_COMPONENT_UNKNOWN:
    default:
      (void)snprintf(line, sizeof(line), "TYPE=UNKNOWN F=%s", frequency);
      LCR_DisplayDrawText(2U, line);
      LCR_DisplayDrawText(3U, "CHECK DUT OR RANGE");
      break;
  }

  (void)snprintf(
    line,
    sizeof(line),
    "SAMPLES=%lu/%lu",
    (unsigned long)measurement->measurement.eligible_count,
    (unsigned long)measurement->measurement.attempt_count);
  LCR_DisplayDrawText(5U, line);
  LCR_DisplayDrawText(
    6U,
    measurement->measurement.average_is_calibrated ?
      "CALIBRATED" : "RAW UNCALIBRATED");
}

static bool LCR_DisplayAddressResponds(uint8_t address_7bit)
{
  return HAL_I2C_IsDeviceReady(
           &hi2c1,
           (uint16_t)address_7bit << 1U,
           LCR_DISPLAY_PROBE_TRIALS,
           LCR_DISPLAY_PROBE_TIMEOUT_MS) == HAL_OK;
}

HAL_StatusTypeDef LCR_DisplayProbe(LCR_DisplayProbeStatus *status)
{
  if (status == NULL)
  {
    return HAL_ERROR;
  }

  status->primary_address_found = LCR_DisplayAddressResponds(
    LCR_DISPLAY_PRIMARY_ADDRESS_7BIT);
  status->secondary_address_found = LCR_DisplayAddressResponds(
    LCR_DISPLAY_SECONDARY_ADDRESS_7BIT);
  status->detected_address_7bit = 0U;

  if (status->primary_address_found)
  {
    status->detected_address_7bit = LCR_DISPLAY_PRIMARY_ADDRESS_7BIT;
  }
  else if (status->secondary_address_found)
  {
    status->detected_address_7bit = LCR_DISPLAY_SECONDARY_ADDRESS_7BIT;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LCR_DisplayInit(uint8_t address_7bit)
{
  static const uint8_t initialization_commands[] =
  {
    LCR_DISPLAY_CONTROL_COMMAND,
    0xAEU,
    0xD5U, 0x80U,
    0xA8U, 0x3FU,
    0xD3U, 0x00U,
    0x40U,
    0x8DU, 0x14U,
    0x20U, 0x00U,
    0xA1U,
    0xC8U,
    0xDAU, 0x12U,
    0x81U, 0x7FU,
    0xD9U, 0xF1U,
    0xDBU, 0x40U,
    0xA4U,
    0xA6U,
    0x2EU,
    0xAFU
  };
  HAL_StatusTypeDef status;

  lcr_display_ready = false;
  lcr_display_address_7bit = address_7bit;
  if ((address_7bit != LCR_DISPLAY_PRIMARY_ADDRESS_7BIT) &&
      (address_7bit != LCR_DISPLAY_SECONDARY_ADDRESS_7BIT))
  {
    lcr_display_address_7bit = 0U;
    return HAL_ERROR;
  }

  status = LCR_DisplayWrite(
    initialization_commands,
    (uint16_t)sizeof(initialization_commands));
  if (status == HAL_OK)
  {
    status = LCR_DisplaySetAddressWindow();
  }
  if (status == HAL_OK)
  {
    lcr_display_ready = true;
  }
  return status;
}

HAL_StatusTypeDef LCR_DisplayShowTestPattern(void)
{
  uint8_t page_buffer[LCR_DISPLAY_WIDTH + 1U];
  uint32_t page;
  HAL_StatusTypeDef status;

  if (!lcr_display_ready)
  {
    return HAL_ERROR;
  }

  status = LCR_DisplaySetAddressWindow();
  for (page = 0U;
       (page < LCR_DISPLAY_PAGE_COUNT) && (status == HAL_OK);
       ++page)
  {
    uint32_t column;

    page_buffer[0] = LCR_DISPLAY_CONTROL_DATA;
    for (column = 0U; column < LCR_DISPLAY_WIDTH; ++column)
    {
      const uint32_t pixel_y_a = column / 2U;
      const uint32_t pixel_y_b =
        (LCR_DISPLAY_HEIGHT - 1U) - pixel_y_a;
      uint8_t pixels = 0U;

      if (page == 0U)
      {
        pixels |= 0x01U;
      }
      if (page == (LCR_DISPLAY_PAGE_COUNT - 1U))
      {
        pixels |= 0x80U;
      }
      if ((column == 0U) || (column == (LCR_DISPLAY_WIDTH - 1U)))
      {
        pixels = 0xFFU;
      }
      if ((pixel_y_a / 8U) == page)
      {
        pixels |= (uint8_t)(1UL << (pixel_y_a % 8U));
      }
      if ((pixel_y_b / 8U) == page)
      {
        pixels |= (uint8_t)(1UL << (pixel_y_b % 8U));
      }
      page_buffer[column + 1U] = pixels;
    }

    status = LCR_DisplayWrite(
      page_buffer,
      (uint16_t)sizeof(page_buffer));
  }
  return status;
}

HAL_StatusTypeDef LCR_DisplayShowReady(void)
{
  if (!lcr_display_ready)
  {
    return HAL_ERROR;
  }

  LCR_DisplayClearBuffer();
  LCR_DisplayDrawText(0U, "LCR METER");
  LCR_DisplayDrawText(2U, "READY");
  LCR_DisplayDrawText(4U, "PRESS BUTTON");
  LCR_DisplayDrawText(6U, "R C L AUTO RANGE");
  return LCR_DisplayFlush();
}

HAL_StatusTypeDef LCR_DisplayShowMeasuring(uint32_t frequency_hz)
{
  char frequency[12];
  char line[32];

  if (!lcr_display_ready)
  {
    return HAL_ERROR;
  }

  LCR_DisplayFormatFrequency(
    frequency, sizeof(frequency), frequency_hz);
  LCR_DisplayClearBuffer();
  LCR_DisplayDrawText(0U, "LCR MEASURING");
  (void)snprintf(line, sizeof(line), "F=%s", frequency);
  LCR_DisplayDrawText(2U, line);
  LCR_DisplayDrawText(4U, "AUTO RANGE + 8 AVG");
  LCR_DisplayDrawText(6U, "PLEASE WAIT");
  return LCR_DisplayFlush();
}

HAL_StatusTypeDef LCR_DisplayShowResult(const LCR_FinalResult *result)
{
  const LCR_ResultMeasurement *measurement = NULL;
  char line[32];

  if (!lcr_display_ready || (result == NULL))
  {
    return HAL_ERROR;
  }

  if ((result->state == LCR_RESULT_COMPLETE) &&
      result->component_valid &&
      (result->component.parameter_frequency_hz != 0U) &&
      result->primary.present &&
      (result->primary.frequency_hz ==
       result->component.parameter_frequency_hz))
  {
    measurement = &result->primary;
  }
  else if ((result->state == LCR_RESULT_COMPLETE) &&
           result->component_valid &&
           (result->component.parameter_frequency_hz != 0U) &&
           result->secondary.present &&
           (result->secondary.frequency_hz ==
            result->component.parameter_frequency_hz))
  {
    measurement = &result->secondary;
  }
  else if (result->secondary.present)
  {
    measurement = &result->secondary;
  }
  else if (result->primary.present)
  {
    measurement = &result->primary;
  }

  LCR_DisplayClearBuffer();
  switch (result->state)
  {
    case LCR_RESULT_COMPLETE:
      (void)snprintf(
        line,
        sizeof(line),
        "LCR RESULT #%lu",
        (unsigned long)result->sequence_id);
      LCR_DisplayDrawText(0U, line);
      if ((measurement != NULL) &&
          measurement->measurement.average_valid &&
          result->component_valid)
      {
        LCR_DisplayDrawMeasurement(result, measurement);
      }
      else
      {
        LCR_DisplayDrawText(2U, "RESULT DATA INVALID");
      }
      LCR_DisplayDrawText(7U, "PRESS TO MEASURE");
      break;

    case LCR_RESULT_ABORTED:
      (void)snprintf(
        line,
        sizeof(line),
        "LCR ABORTED #%lu",
        (unsigned long)result->sequence_id);
      LCR_DisplayDrawText(0U, line);
      LCR_DisplayDrawText(2U, "MEASUREMENT STOPPED");
      if (measurement != NULL)
      {
        (void)snprintf(
          line,
          sizeof(line),
          "SAMPLES=%lu/%lu",
          (unsigned long)measurement->measurement.eligible_count,
          (unsigned long)measurement->measurement.attempt_count);
        LCR_DisplayDrawText(4U, line);
      }
      LCR_DisplayDrawText(7U, "PRESS TO RETRY");
      break;

    case LCR_RESULT_ERROR:
      (void)snprintf(
        line,
        sizeof(line),
        "LCR ERROR #%lu",
        (unsigned long)result->sequence_id);
      LCR_DisplayDrawText(0U, line);
      LCR_DisplayDrawText(2U, LCR_WorkflowErrorName(result->error));
      if (measurement != NULL)
      {
        (void)snprintf(
          line,
          sizeof(line),
          "SAMPLES=%lu/%lu",
          (unsigned long)measurement->measurement.eligible_count,
          (unsigned long)measurement->measurement.attempt_count);
        LCR_DisplayDrawText(4U, line);
        (void)snprintf(
          line,
          sizeof(line),
          "RANGE PROBES=%lu",
          (unsigned long)measurement->autorange_probe_count);
        LCR_DisplayDrawText(5U, line);
      }
      LCR_DisplayDrawText(7U, "PRESS TO RETRY");
      break;

    case LCR_RESULT_RUNNING:
      return LCR_DisplayShowMeasuring(
        (measurement != NULL) ? measurement->frequency_hz : 0U);

    case LCR_RESULT_EMPTY:
    default:
      return LCR_DisplayShowReady();
  }

  return LCR_DisplayFlush();
}

bool LCR_DisplayIsReady(void)
{
  return lcr_display_ready;
}
