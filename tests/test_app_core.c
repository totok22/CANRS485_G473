#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_command_parser.h"
#include "app_config.h"
#include "app_fault.h"
#include "app_utils.h"
#include "main.h"

static int g_failures;

#define EXPECT_U32(name, actual, expected)                                      \
  do                                                                            \
  {                                                                             \
    uint32_t actual_value = (uint32_t)(actual);                                  \
    uint32_t expected_value = (uint32_t)(expected);                              \
    if (actual_value != expected_value)                                          \
    {                                                                           \
      printf("FAIL %s: got 0x%08lX expected 0x%08lX\n",                         \
             name,                                                              \
             (unsigned long)actual_value,                                       \
             (unsigned long)expected_value);                                    \
      g_failures++;                                                             \
    }                                                                           \
  } while (0)

static void ExpectCommand(const char *input, uint8_t expected)
{
  uint8_t value = 0U;
  uint8_t ok = App_CommandTryParse((const uint8_t *)input, (uint8_t)strlen(input), &value);

  EXPECT_U32("command parse ok", ok, 1U);
  EXPECT_U32("command value", value, expected);
}

static void TestCommandParser(void)
{
  uint8_t value = 0U;
  uint8_t raw = APP_MODE_CMD_ENDURANCE;

  ExpectCommand("straight", APP_MODE_CMD_STRAIGHT);
  ExpectCommand(" Avoidance ", APP_MODE_CMD_AUTOCROSS);
  ExpectCommand("50", APP_MODE_CMD_SKIDPAD);

  EXPECT_U32("raw command parse", App_CommandTryParse(&raw, 1U, &value), 1U);
  EXPECT_U32("raw command value", value, APP_MODE_CMD_ENDURANCE);
  EXPECT_U32("invalid command", App_CommandTryParse((const uint8_t *)"52", 2U, &value), 0U);
  EXPECT_U32("empty command", App_CommandTryParse((const uint8_t *)" \t", 2U, &value), 0U);
}

static void TestUtils(void)
{
  const uint8_t data[] = {0x12U, 0x34U, 0x56U, 0x78U};

  EXPECT_U32("read be16", App_ReadBe16(data), 0x1234U);
  EXPECT_U32("read le16", App_ReadLe16(data), 0x3412U);
  EXPECT_U32("read le32", App_ReadLe32(data), 0x78563412UL);
  EXPECT_U32("read be32 signed", (uint32_t)App_ReadBe32Signed(data), 0x12345678UL);
  EXPECT_U32("fresh false zero", App_IsFresh(100U, 0U), 0U);
  EXPECT_U32("fresh true", App_IsFresh(1000U, 900U), 1U);
  EXPECT_U32("fresh false timeout", App_IsFresh(5000U, 1000U), 0U);
  EXPECT_U32("fresh true wrap", App_IsFresh(10U, 0xFFFFFFF0UL), 1U);
  EXPECT_U32("fdcan dlc to bytes", App_FdcanDlcToBytes(9U), 12U);
  EXPECT_U32("fdcan bytes to dlc clamp", App_FdcanDlcFromBytes(12U), FDCAN_DLC_BYTES_8);
}

static void TestFaultCompute(void)
{
  AppTelemetryState state;
  uint32_t expected;

  memset(&state, 0, sizeof(state));
  state.alarm_updated_ms = 900U;
  state.alarm_fault_code = 0x00000005UL;
  state.pack_summary_updated_ms = 900U;
  state.imd_signal = 1U;
  state.hall_updated_ms = 900U;
  state.hall_error = 1U;
  state.can2_diag_updated_ms = 900U;
  state.hall_fault_active = 1U;
  state.imd_fault_active = 1U;
  state.can2_error_rom_low16 = 0x0034U;

  expected = 0x00000035UL | APP_HALL_FAULT_BIT | APP_IMD_FAULT_BIT;
  EXPECT_U32("fault combined fresh", App_FaultCompute(&state, 1000U), expected);
  EXPECT_U32("fault stale", App_FaultCompute(&state, 4000U), 0U);
}

int main(void)
{
  TestCommandParser();
  TestUtils();
  TestFaultCompute();

  if (g_failures != 0)
  {
    printf("%d test failure(s)\n", g_failures);
    return 1;
  }

  printf("app_core_tests passed\n");
  return 0;
}
