#include "app_command_parser.h"

#include "app_config.h"

static uint8_t App_CommandTryParseWord(const uint8_t *data, uint8_t length, uint8_t *command_value);
static uint8_t App_CommandEqualsWord(const uint8_t *data, uint8_t length, const char *word);

uint8_t App_CommandTryParse(const uint8_t *data, uint8_t length, uint8_t *command_value)
{
  uint8_t start = 0U;
  uint8_t end = length;
  uint16_t decimal_value = 0U;
  uint8_t i;
  uint8_t has_digit = 0U;

  while ((start < end) && ((data[start] == ' ') || (data[start] == '\t')))
  {
    start++;
  }
  while ((end > start) && ((data[end - 1U] == ' ') || (data[end - 1U] == '\t')))
  {
    end--;
  }

  if (end <= start)
  {
    return 0U;
  }

  if ((end - start) == 1U)
  {
    uint8_t value = data[start];
    if ((value >= APP_MODE_CMD_STRAIGHT) && (value <= APP_MODE_CMD_ENDURANCE))
    {
      *command_value = value;
      return 1U;
    }
  }

  for (i = start; i < end; ++i)
  {
    if ((data[i] < '0') || (data[i] > '9'))
    {
      has_digit = 0U;
      break;
    }
    has_digit = 1U;
    decimal_value = (uint16_t)((decimal_value * 10U) + (uint16_t)(data[i] - '0'));
  }

  if ((has_digit != 0U) &&
      (decimal_value >= APP_MODE_CMD_STRAIGHT) &&
      (decimal_value <= APP_MODE_CMD_ENDURANCE))
  {
    *command_value = (uint8_t)decimal_value;
    return 1U;
  }

  return App_CommandTryParseWord(&data[start], (uint8_t)(end - start), command_value);
}

static uint8_t App_CommandTryParseWord(const uint8_t *data, uint8_t length, uint8_t *command_value)
{
  if (App_CommandEqualsWord(data, length, "straight") != 0U)
  {
    *command_value = APP_MODE_CMD_STRAIGHT;
    return 1U;
  }
  if ((App_CommandEqualsWord(data, length, "autocross") != 0U) ||
      (App_CommandEqualsWord(data, length, "avoidance") != 0U))
  {
    *command_value = APP_MODE_CMD_AUTOCROSS;
    return 1U;
  }
  if (App_CommandEqualsWord(data, length, "skidpad") != 0U)
  {
    *command_value = APP_MODE_CMD_SKIDPAD;
    return 1U;
  }
  if (App_CommandEqualsWord(data, length, "endurance") != 0U)
  {
    *command_value = APP_MODE_CMD_ENDURANCE;
    return 1U;
  }

  return 0U;
}

static uint8_t App_CommandEqualsWord(const uint8_t *data, uint8_t length, const char *word)
{
  uint8_t i = 0U;

  while (word[i] != '\0')
  {
    uint8_t ch;

    if (i >= length)
    {
      return 0U;
    }

    ch = data[i];
    if ((ch >= 'A') && (ch <= 'Z'))
    {
      ch = (uint8_t)(ch + ('a' - 'A'));
    }
    if (ch != (uint8_t)word[i])
    {
      return 0U;
    }
    i++;
  }

  return (i == length) ? 1U : 0U;
}
