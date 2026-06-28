#include "app.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_can_decode.h"
#include "app_can_io.h"
#include "app_command_io.h"
#include "app_config.h"
#include "app_state.h"
#include "app_utils.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"

#include "fsae_telemetry.pb.h"
#include "pb_encode.h"

#define APP_LED_FAST_BLINK_MS           100U
#define APP_LED_SLOW_BLINK_MS           500U
#define APP_BASE_TELEMETRY_PERIOD_MS    100U
#define APP_MODULE_TELEMETRY_PERIOD_MS  500U
#define APP_RS485_TX_TIMEOUT_MS         200U
#define APP_DEBUG_CLI_ENABLE            0U
#define APP_DEBUG_UART_TIMEOUT_MS       50U
#define APP_DEBUG_CAN_TRACE_DEPTH       8U
#define APP_DEBUG_DUMP_COUNT            8U
#define APP_DEBUG_CLI_BUFFER_SIZE       32U
#define APP_DEBUG_CAN_ID_COUNTER_COUNT  16U

/* PC8 LED0 is high-level on in the G473 board notes. */
#define APP_LED_ON_STATE                GPIO_PIN_SET
#define APP_LED_OFF_STATE               GPIO_PIN_RESET


_Static_assert(APP_MODULE_COUNT == (sizeof(((fsae_TelemetryFrame *)0)->modules) / sizeof(((fsae_TelemetryFrame *)0)->modules[0])),
               "APP_MODULE_COUNT must match fsae_TelemetryFrame.modules capacity");
_Static_assert(APP_CELLS_PER_MODULE == 23U, "BatteryModule protobuf layout has 23 voltage fields");
_Static_assert(APP_TEMPS_PER_MODULE == 8U, "BatteryModule protobuf layout has 8 temperature fields");
_Static_assert(APP_CAN1_VOLTAGE_FRAMES_PER_MODULE < 8U, "module voltage validity mask must fit in uint8_t");

typedef struct
{
  uint8_t bus_index;
  uint8_t is_extended_id;
  uint8_t dlc;
  uint32_t id;
  uint8_t data[8];
} AppCanTraceFrame;

typedef struct
{
  uint8_t bus_index;
  uint8_t is_extended_id;
  uint32_t id;
  uint32_t count;
  uint32_t last_ms;
} AppCanIdCounter;


static volatile AppTelemetryState g_app_state;
static AppTelemetryState g_tx_snapshot;
static fsae_TelemetryFrame g_tx_frame;
static uint8_t g_pb_buffer[fsae_TelemetryFrame_size];
static const fsae_TelemetryFrame g_tx_frame_prototype = fsae_TelemetryFrame_init_zero;
static const fsae_BatteryModule g_battery_module_prototype = fsae_BatteryModule_init_zero;
static const fsae_MotorState g_motor_state_prototype = fsae_MotorState_init_zero;
static uint32_t g_frame_counter;
static uint32_t g_last_base_tx_ms;
static uint32_t g_last_module_tx_ms;
static uint32_t g_last_led_toggle_ms;
static uint32_t g_last_rs485_tx_ms;
static uint8_t g_led_blink_on;
static volatile AppCanTraceFrame g_can_trace[APP_DEBUG_CAN_TRACE_DEPTH];
static volatile uint8_t g_can_trace_write_index;
static volatile uint8_t g_can_trace_count;
static volatile AppCanIdCounter g_can_id_counters[APP_DEBUG_CAN_ID_COUNTER_COUNT];
static volatile uint8_t g_can_id_counter_count;
static volatile uint8_t g_cli_rx_byte;
static volatile char g_cli_rx_buffer[APP_DEBUG_CLI_BUFFER_SIZE];
static volatile uint8_t g_cli_rx_length;
static volatile char g_cli_command_buffer[APP_DEBUG_CLI_BUFFER_SIZE];
static volatile uint8_t g_cli_command_ready;
#if APP_DEBUG_CLI_ENABLE
static uint8_t g_debug_last_command_bytes[APP_COMMAND_RX_BUFFER_SIZE];
static uint8_t g_debug_last_command_length;
static uint8_t g_debug_last_command_parse_ok;
static uint8_t g_debug_last_command_value;
static uint32_t g_debug_command_rx_count;
static uint32_t g_debug_command_parse_fail_count;
static uint32_t g_debug_mode_tx_ok_count;
static uint32_t g_debug_mode_tx_error_count;
static uint8_t g_debug_last_mode_tx_value;
static HAL_StatusTypeDef g_debug_last_mode_tx_status;
static uint32_t g_debug_last_mode_tx_ms;
#endif

static uint8_t App_IvtStateIsUsable(uint8_t state);
static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now);
static void App_UpdateFaultCode(void);
static void App_SetStatusLed(uint8_t on);
static void App_UpdateStatusLed(uint32_t now);
static void App_DebugRecordCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data);
static void App_DebugPollCli(void);
#if APP_DEBUG_CLI_ENABLE
static void App_DebugWrite(const char *text);
static void App_DebugPrintf(const char *format, ...);
static void App_DebugHandleCommand(const char *command);
static void App_DebugPrintHelp(void);
static void App_DebugPrintStatus(void);
static void App_DebugPrintRecentCan(const char *command);
static void App_DebugPrintCanIds(void);
static void App_DebugPrintCommandState(void);
static void App_DebugClear(void);
static void App_DebugRecordCanId(uint8_t bus_index, uint8_t is_extended_id, uint32_t id, uint32_t now);
static void App_DebugRecordCommandRx(const uint8_t *data, uint8_t length, uint8_t parse_ok, uint8_t command_value);
static void App_DebugRecordModeTx(uint8_t command_value, HAL_StatusTypeDef status, uint32_t now);
#endif
static HAL_StatusTypeDef App_SendModeCommand(uint8_t command_value);
static void App_ProcessCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data);
static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now);
static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now);
static uint8_t App_GetEnergyMeterSource(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now);
static fsae_DrivingMode App_GetDrivingMode(const AppTelemetryState *state, uint32_t now);
static fsae_AlarmSeverity App_MapBatteryAlarmSeverity(uint8_t alarm_level);
static void App_AddAlarm(fsae_TelemetryFrame *frame, uint32_t alarm_id,
                         fsae_AlarmSeverity severity, const char *message);
static void App_BuildAlarms(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp, uint8_t max_temp_valid);
static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no);
static uint8_t App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                                  int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no);
static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static uint8_t App_BuildMotorStates(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules);
static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size);

void App_Init(void)
{
  memset((void *)&g_app_state, 0, sizeof(g_app_state));
  g_frame_counter = 0U;
  g_last_base_tx_ms = HAL_GetTick();
  g_last_module_tx_ms = HAL_GetTick();
  g_last_led_toggle_ms = HAL_GetTick();
  g_last_rs485_tx_ms = 0U;
  g_led_blink_on = 0U;
  g_can_trace_write_index = 0U;
  g_can_trace_count = 0U;
  g_can_id_counter_count = 0U;
  g_cli_rx_length = 0U;
  g_cli_command_ready = 0U;
#if APP_DEBUG_CLI_ENABLE
  g_debug_last_command_length = 0U;
  g_debug_last_command_parse_ok = 0U;
  g_debug_last_command_value = 0U;
  g_debug_command_rx_count = 0U;
  g_debug_command_parse_fail_count = 0U;
  g_debug_mode_tx_ok_count = 0U;
  g_debug_mode_tx_error_count = 0U;
  g_debug_last_mode_tx_value = 0U;
  g_debug_last_mode_tx_status = HAL_OK;
  g_debug_last_mode_tx_ms = 0U;
#endif

  App_SetStatusLed(0U);

  if (App_CanIo_Init(App_ProcessCanRx) != HAL_OK)
  {
    Error_Handler();
  }

#if APP_DEBUG_CLI_ENABLE
  if (App_CommandIo_Init(App_SendModeCommand, App_DebugRecordCommandRx, App_DebugRecordModeTx) != HAL_OK)
  {
    Error_Handler();
  }
#else
  if (App_CommandIo_Init(App_SendModeCommand, NULL, NULL) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if APP_DEBUG_CLI_ENABLE
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_cli_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
#endif

}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t include_modules;

  App_UpdateStatusLed(now);
  App_DebugPollCli();
  App_CommandIo_Poll(now);
  App_CanIo_Service();

  if ((g_app_state.can1_seen == 0U) && (g_app_state.can2_seen == 0U))
  {
    return;
  }

  if ((uint32_t)(now - g_last_base_tx_ms) < APP_BASE_TELEMETRY_PERIOD_MS)
  {
    return;
  }

  include_modules = (uint8_t)(((uint32_t)(now - g_last_module_tx_ms) >= APP_MODULE_TELEMETRY_PERIOD_MS) ? 1U : 0U);
  if (App_SendTelemetry(now, include_modules) == HAL_OK)
  {
    g_last_base_tx_ms = now;
    if (include_modules != 0U)
    {
      g_last_module_tx_ms = now;
    }
  }
}

static uint8_t App_IvtStateIsUsable(uint8_t state)
{
  return ((state & APP_IVT_RESULT_ERROR_MASK) == 0U) ? 1U : 0U;
}

static void App_UpdateFaultCode(void)
{
  g_app_state.battery_fault_code = App_ComputeFaultCode((const AppTelemetryState *)&g_app_state, HAL_GetTick());
}

static void App_SetStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (on != 0U) ? APP_LED_ON_STATE : APP_LED_OFF_STATE);
}

static void App_UpdateStatusLed(uint32_t now)
{
  uint8_t has_fault = (App_ComputeFaultCode((const AppTelemetryState *)&g_app_state, now) != 0U) ? 1U : 0U;
  uint8_t telemetry_active = ((g_last_rs485_tx_ms != 0U) &&
                              ((g_app_state.can1_seen != 0U) || (g_app_state.can2_seen != 0U))) ? 1U : 0U;

  if (has_fault != 0U)
  {
    if ((uint32_t)(now - g_last_led_toggle_ms) >= APP_LED_FAST_BLINK_MS)
    {
      g_last_led_toggle_ms = now;
      g_led_blink_on = (uint8_t)(g_led_blink_on == 0U);
      App_SetStatusLed(g_led_blink_on);
    }
    return;
  }

  if (telemetry_active != 0U)
  {
    g_led_blink_on = 1U;
    App_SetStatusLed(1U);
    return;
  }

  if ((uint32_t)(now - g_last_led_toggle_ms) >= APP_LED_SLOW_BLINK_MS)
  {
    g_last_led_toggle_ms = now;
    g_led_blink_on = (uint8_t)(g_led_blink_on == 0U);
    App_SetStatusLed(g_led_blink_on);
  }
}

static void App_DebugRecordCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data)
{
#if APP_DEBUG_CLI_ENABLE
  uint32_t primask = __get_PRIMASK();
  AppCanTraceFrame *slot = (AppCanTraceFrame *)&g_can_trace[g_can_trace_write_index];
  uint8_t bus_index = (bus == APP_FDCAN_BUS_CAN1) ? 1U :
                      ((bus == APP_FDCAN_BUS_CANB) ? 2U :
                      ((bus == APP_FDCAN_BUS_CANC) ? 3U : 0U));
  uint8_t is_extended_id = (header->IDE == APP_CAN_ID_EXT) ? 1U : 0U;
  uint32_t id = (header->IDE == APP_CAN_ID_EXT) ? header->ExtId : header->StdId;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  slot->bus_index = bus_index;
  slot->is_extended_id = is_extended_id;
  slot->dlc = header->DLC;
  slot->id = id;
  memcpy(slot->data, data, 8U);

  g_can_trace_write_index = (uint8_t)((g_can_trace_write_index + 1U) % APP_DEBUG_CAN_TRACE_DEPTH);
  if (g_can_trace_count < APP_DEBUG_CAN_TRACE_DEPTH)
  {
    g_can_trace_count++;
  }
  App_DebugRecordCanId(bus_index, is_extended_id, id, now);
  if (primask == 0U)
  {
    __enable_irq();
  }
#else
  (void)bus;
  (void)header;
  (void)data;
#endif
}

static void App_DebugPollCli(void)
{
#if APP_DEBUG_CLI_ENABLE
  uint8_t ready;
  char command[APP_DEBUG_CLI_BUFFER_SIZE];

  __disable_irq();
  ready = g_cli_command_ready;
  if (ready != 0U)
  {
    memcpy(command, (const void *)g_cli_command_buffer, APP_DEBUG_CLI_BUFFER_SIZE);
    g_cli_command_ready = 0U;
  }
  __enable_irq();

  if (ready == 0U)
  {
    return;
  }

  App_DebugHandleCommand(command);
#endif
}

#if APP_DEBUG_CLI_ENABLE
static void App_DebugWrite(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), APP_DEBUG_UART_TIMEOUT_MS);
}

static void App_DebugPrintf(const char *format, ...)
{
  char line[160];
  va_list args;
  int written;

  va_start(args, format);
  written = vsnprintf(line, sizeof(line), format, args);
  va_end(args);

  if (written > 0)
  {
    if (written >= (int)sizeof(line))
    {
      written = (int)sizeof(line) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)written, APP_DEBUG_UART_TIMEOUT_MS);
  }
}

static void App_DebugHandleCommand(const char *command)
{
  if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0))
  {
    App_DebugPrintHelp();
  }
  else if (strcmp(command, "status") == 0)
  {
    App_DebugPrintStatus();
  }
  else if (strcmp(command, "can") == 0)
  {
    App_DebugPrintRecentCan(command);
  }
  else if (strcmp(command, "ids") == 0)
  {
    App_DebugPrintCanIds();
  }
  else if (strcmp(command, "cmd") == 0)
  {
    App_DebugPrintCommandState();
  }
  else if (strcmp(command, "clear") == 0)
  {
    App_DebugClear();
  }
  else
  {
    App_DebugPrintf("unknown command: %s\r\n", command);
    App_DebugPrintHelp();
  }
}

static void App_DebugPrintHelp(void)
{
  App_DebugWrite("commands: help status can ids cmd clear\r\n");
}

static void App_DebugPrintStatus(void)
{
  AppTelemetryState snapshot;
  uint32_t now = HAL_GetTick();
  uint32_t last_rs485_tx_ms = g_last_rs485_tx_ms;
  uint32_t frame_counter = g_frame_counter;

  __disable_irq();
  snapshot = *((const AppTelemetryState *)&g_app_state);
  __enable_irq();

  App_DebugPrintf(
      "status: tick=%lu frames=%lu rs485_last_age=%lu can1=%u can2=%u mode_flag=%d mode_age=%lu fault=0x%08lX\r\n",
      (unsigned long)now,
      (unsigned long)frame_counter,
      (unsigned long)((last_rs485_tx_ms == 0U) ? 0UL : (now - last_rs485_tx_ms)),
      (unsigned int)snapshot.can1_seen,
      (unsigned int)snapshot.can2_seen,
      (int)snapshot.vehicle_mode_flag,
      (unsigned long)((snapshot.vehicle_mode_updated_ms == 0U) ? 0UL : (now - snapshot.vehicle_mode_updated_ms)),
      (unsigned long)App_ComputeFaultCode(&snapshot, now));
}

static void App_DebugPrintRecentCan(const char *command)
{
  AppCanTraceFrame frames[APP_DEBUG_DUMP_COUNT];
  uint8_t count;
  uint8_t write_index;
  uint8_t i;

  __disable_irq();
  count = g_can_trace_count;
  write_index = g_can_trace_write_index;
  if (count > APP_DEBUG_DUMP_COUNT)
  {
    count = APP_DEBUG_DUMP_COUNT;
  }
  for (i = 0U; i < count; ++i)
  {
    uint8_t src_index = (uint8_t)((write_index + APP_DEBUG_CAN_TRACE_DEPTH - count + i) % APP_DEBUG_CAN_TRACE_DEPTH);
    frames[i] = *((const AppCanTraceFrame *)&g_can_trace[src_index]);
  }
  __enable_irq();

  if (count == 0U)
  {
    App_DebugPrintf("cli[%s]: no CAN frames captured yet\r\n", command);
    return;
  }

  App_DebugPrintf("cli[%s]: dumping %u recent CAN frames\r\n", command, (unsigned int)count);

  for (i = 0U; i < count; ++i)
  {
    App_DebugPrintf(
      "CAN%u %s 0x%08lX DLC=%u [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
      (unsigned int)frames[i].bus_index,
      (frames[i].is_extended_id != 0U) ? "EXT" : "STD",
      (unsigned long)frames[i].id,
      (unsigned int)frames[i].dlc,
      (unsigned int)frames[i].data[0],
      (unsigned int)frames[i].data[1],
      (unsigned int)frames[i].data[2],
      (unsigned int)frames[i].data[3],
      (unsigned int)frames[i].data[4],
      (unsigned int)frames[i].data[5],
      (unsigned int)frames[i].data[6],
      (unsigned int)frames[i].data[7]
    );
  }
}

static void App_DebugPrintCanIds(void)
{
  AppCanIdCounter counters[APP_DEBUG_CAN_ID_COUNTER_COUNT];
  uint8_t count;
  uint8_t i;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  count = g_can_id_counter_count;
  memcpy(counters, (const void *)g_can_id_counters, sizeof(counters));
  __enable_irq();

  if (count == 0U)
  {
    App_DebugWrite("ids: none\r\n");
    return;
  }

  App_DebugPrintf("ids: %u tracked\r\n", (unsigned int)count);
  for (i = 0U; i < count; ++i)
  {
    App_DebugPrintf("CAN%u %s 0x%08lX count=%lu age=%lu\r\n",
                    (unsigned int)counters[i].bus_index,
                    (counters[i].is_extended_id != 0U) ? "EXT" : "STD",
                    (unsigned long)counters[i].id,
                    (unsigned long)counters[i].count,
                    (unsigned long)(now - counters[i].last_ms));
  }
}

static void App_DebugPrintCommandState(void)
{
  uint8_t bytes[APP_COMMAND_RX_BUFFER_SIZE];
  uint8_t length;
  uint8_t parse_ok;
  uint8_t command_value = 0U;
  uint32_t rx_count;
  uint32_t fail_count;
  uint32_t tx_ok_count;
  uint32_t tx_error_count;
  uint8_t last_tx_value;
  HAL_StatusTypeDef last_tx_status;
  uint32_t last_tx_ms;
  uint32_t now = HAL_GetTick();
  uint8_t i;

  __disable_irq();
  memcpy(bytes, g_debug_last_command_bytes, sizeof(bytes));
  length = g_debug_last_command_length;
  parse_ok = g_debug_last_command_parse_ok;
  command_value = g_debug_last_command_value;
  rx_count = g_debug_command_rx_count;
  fail_count = g_debug_command_parse_fail_count;
  tx_ok_count = g_debug_mode_tx_ok_count;
  tx_error_count = g_debug_mode_tx_error_count;
  last_tx_value = g_debug_last_mode_tx_value;
  last_tx_status = g_debug_last_mode_tx_status;
  last_tx_ms = g_debug_last_mode_tx_ms;
  __enable_irq();

  App_DebugPrintf("cmd: rx=%lu parse_fail=%lu last_len=%u parse_ok=%u value=%u\r\n",
                  (unsigned long)rx_count,
                  (unsigned long)fail_count,
                  (unsigned int)length,
                  (unsigned int)parse_ok,
                  (unsigned int)command_value);
  App_DebugWrite("cmd bytes:");
  for (i = 0U; i < length; ++i)
  {
    App_DebugPrintf(" %02X", (unsigned int)bytes[i]);
  }
  App_DebugWrite("\r\n");
  App_DebugPrintf("mode_tx: ok=%lu err=%lu last_value=%u last_status=%d age=%lu\r\n",
                  (unsigned long)tx_ok_count,
                  (unsigned long)tx_error_count,
                  (unsigned int)last_tx_value,
                  (int)last_tx_status,
                  (unsigned long)((last_tx_ms == 0U) ? 0UL : (now - last_tx_ms)));
}

static void App_DebugClear(void)
{
  __disable_irq();
  g_can_trace_write_index = 0U;
  g_can_trace_count = 0U;
  g_can_id_counter_count = 0U;
  memset((void *)g_can_id_counters, 0, sizeof(g_can_id_counters));
  g_debug_command_rx_count = 0U;
  g_debug_command_parse_fail_count = 0U;
  g_debug_mode_tx_ok_count = 0U;
  g_debug_mode_tx_error_count = 0U;
  __enable_irq();

  App_DebugWrite("debug counters cleared\r\n");
}

static void App_DebugRecordCanId(uint8_t bus_index, uint8_t is_extended_id, uint32_t id, uint32_t now)
{
  uint8_t i;
  uint8_t count = g_can_id_counter_count;

  for (i = 0U; i < count; ++i)
  {
    if ((g_can_id_counters[i].bus_index == bus_index) &&
        (g_can_id_counters[i].is_extended_id == is_extended_id) &&
        (g_can_id_counters[i].id == id))
    {
      g_can_id_counters[i].count++;
      g_can_id_counters[i].last_ms = now;
      return;
    }
  }

  if (count < APP_DEBUG_CAN_ID_COUNTER_COUNT)
  {
    g_can_id_counters[count].bus_index = bus_index;
    g_can_id_counters[count].is_extended_id = is_extended_id;
    g_can_id_counters[count].id = id;
    g_can_id_counters[count].count = 1U;
    g_can_id_counters[count].last_ms = now;
    g_can_id_counter_count = (uint8_t)(count + 1U);
  }
}

static void App_DebugRecordCommandRx(const uint8_t *data, uint8_t length, uint8_t parse_ok, uint8_t command_value)
{
  if (length > APP_COMMAND_RX_BUFFER_SIZE)
  {
    length = APP_COMMAND_RX_BUFFER_SIZE;
  }

  __disable_irq();
  memcpy(g_debug_last_command_bytes, data, length);
  g_debug_last_command_length = length;
  g_debug_last_command_parse_ok = parse_ok;
  g_debug_last_command_value = command_value;
  g_debug_command_rx_count++;
  if (parse_ok == 0U)
  {
    g_debug_command_parse_fail_count++;
  }
  __enable_irq();
}

static void App_DebugRecordModeTx(uint8_t command_value, HAL_StatusTypeDef status, uint32_t now)
{
  __disable_irq();
  g_debug_last_mode_tx_value = command_value;
  g_debug_last_mode_tx_status = status;
  g_debug_last_mode_tx_ms = now;
  if (status == HAL_OK)
  {
    g_debug_mode_tx_ok_count++;
  }
  else
  {
    g_debug_mode_tx_error_count++;
  }
  __enable_irq();
}
#endif

static HAL_StatusTypeDef App_SendModeCommand(uint8_t command_value)
{
  uint8_t tx_data[1];

  if ((command_value < APP_MODE_CMD_STRAIGHT) || (command_value > APP_MODE_CMD_ENDURANCE))
  {
    return HAL_ERROR;
  }

  tx_data[0] = command_value;

  return App_CanIo_SendCanBStd(APP_CAN2_MODE_COMMAND_ID, tx_data, 1U);
}

static void App_ProcessCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data)
{
  uint8_t decode_events;

  App_DebugRecordCanRx(bus, header, data);
  decode_events = App_CanDecode_Process(&g_app_state, bus, header, data);
  if ((decode_events & APP_CAN_DECODE_EVENT_FAULT_REFRESH) != 0U)
  {
    App_UpdateFaultCode();
  }
}

static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now)
{
  uint32_t faults = 0U;

  if (App_IsFresh(now, state->alarm_updated_ms) != 0U)
  {
    faults |= state->alarm_fault_code;
  }

  if ((App_IsFresh(now, state->pack_summary_updated_ms) != 0U) && (state->imd_signal != 0U))
  {
    faults |= APP_IMD_FAULT_BIT;
  }

  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error != 0U))
  {
    faults |= APP_HALL_FAULT_BIT;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    if (state->hall_fault_active != 0U)
    {
      faults |= APP_HALL_FAULT_BIT;
    }
    if (state->imd_fault_active != 0U)
    {
      faults |= APP_IMD_FAULT_BIT;
    }
    faults |= (uint32_t)state->can2_error_rom_low16;
  }

  return faults;
}

static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now)
{
  uint32_t i;
  uint32_t total_mv = 0U;

  if (App_IsFresh(now, state->fs_voltage_updated_ms) != 0U)
  {
    return (float)state->fs_voltage_mv / 1000.0f;
  }

  if ((App_IsFresh(now, state->ivt_voltage_u1_updated_ms) != 0U) &&
      (App_IvtStateIsUsable(state->ivt_voltage_u1_state) != 0U))
  {
    return (float)state->ivt_voltage_u1_mv / 1000.0f;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return (float)state->pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->cell_voltage_sum_updated_ms) != 0U)
  {
    return (float)state->cell_voltage_sum_deci_v / 10.0f;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != APP_MODULE_VOLTAGE_VALID_MASK)
    {
      return 0.0f;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    total_mv += state->cell_voltage_mv[i];
  }

  return (float)total_mv / 1000.0f;
}

static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->fs_current_updated_ms) != 0U)
  {
    return (float)state->fs_current_ma / 1000.0f;
  }

  if ((App_IsFresh(now, state->ivt_current_updated_ms) != 0U) &&
      (App_IvtStateIsUsable(state->ivt_current_state) != 0U))
  {
    return (float)state->ivt_current_ma / 1000.0f;
  }

  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error == 0U))
  {
    return (float)state->hall_current_ma / 1000.0f;
  }

  if (App_IsFresh(now, state->charger_fb_updated_ms) != 0U)
  {
    return (float)state->charger_fb_current_deci_a / 10.0f;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    if (App_IsFresh(now, state->status_updated_ms) != 0U && (state->charge_state != 0U))
    {
      return (float)state->summary_current_raw / 10.0f;
    }

    return ((float)state->summary_current_raw - 10000.0f) / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return ((float)state->can2_pack_current_raw - 10000.0f) / 10.0f;
  }

  return 0.0f;
}

static uint8_t App_GetEnergyMeterSource(const AppTelemetryState *state, uint32_t now)
{
  if ((App_IsFresh(now, state->fs_current_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_voltage_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_power_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_energy_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_status_updated_ms) != 0U))
  {
    return APP_ENERGY_METER_SOURCE_FS;
  }

  if ((App_IsFresh(now, state->ivt_current_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_voltage_u1_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_power_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_energy_updated_ms) != 0U))
  {
    return APP_ENERGY_METER_SOURCE_IVT;
  }

  return APP_ENERGY_METER_SOURCE_UNKNOWN;
}

static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_soc;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return state->can2_soc;
  }

  return 0U;
}

static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->status_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    return state->can2_battery_state;
  }

  return 0U;
}

static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now)
{
  return (App_GetVcuStatus(state, now) == 5U) ? 1U : 0U;
}

static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now)
{
  return App_ComputeFaultCode(state, now);
}

static fsae_DrivingMode App_GetDrivingMode(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->vehicle_mode_updated_ms) == 0U)
  {
    return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
  }

  switch (state->vehicle_mode_flag)
  {
    case -1:
      return fsae_DrivingMode_DRIVING_MODE_DEFAULT;
    case 0:
      return fsae_DrivingMode_DRIVING_MODE_STRAIGHT;
    case 1:
      return fsae_DrivingMode_DRIVING_MODE_AUTOCROSS;
    case 2:
      return fsae_DrivingMode_DRIVING_MODE_SKIDPAD;
    case 3:
      return fsae_DrivingMode_DRIVING_MODE_ENDURANCE;
    default:
      break;
  }

  return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
}

static fsae_AlarmSeverity App_MapBatteryAlarmSeverity(uint8_t alarm_level)
{
  switch (alarm_level)
  {
    case 1U:
      return fsae_AlarmSeverity_ALARM_SEVERITY_WARNING;
    case 2U:
      return fsae_AlarmSeverity_ALARM_SEVERITY_ERROR;
    default:
      break;
  }

  return fsae_AlarmSeverity_ALARM_SEVERITY_FATAL;
}

static void App_AddAlarm(fsae_TelemetryFrame *frame, uint32_t alarm_id,
                         fsae_AlarmSeverity severity, const char *message)
{
  fsae_Alarm *alarm;

  if (frame->alarms_count >= (sizeof(frame->alarms) / sizeof(frame->alarms[0])))
  {
    return;
  }

  alarm = &frame->alarms[frame->alarms_count];
  alarm->alarm_id = alarm_id;
  alarm->severity = severity;
  (void)snprintf(alarm->message, sizeof(alarm->message), "%s", message);
  frame->alarms_count++;
}

static void App_BuildAlarms(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  char message[64];

  if (App_IsFresh(now, state->pack_summary_updated_ms) == 0U)
  {
    return;
  }

  if (state->battery_alarm_level != 0U)
  {
    (void)snprintf(message, sizeof(message), "BMS summary alarm level %u", (unsigned int)state->battery_alarm_level);
    App_AddAlarm(frame, APP_CAN1_PACK_SUMMARY_ID,
                 App_MapBatteryAlarmSeverity(state->battery_alarm_level), message);
  }

  if (state->imd_signal != 0U)
  {
    (void)snprintf(message, sizeof(message), "IMD signal raw %u", (unsigned int)state->imd_signal);
    App_AddAlarm(frame, APP_IMD_FAULT_BIT,
                 fsae_AlarmSeverity_ALARM_SEVERITY_ERROR, message);
  }
}

static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp, uint8_t max_temp_valid)
{
  if (max_temp_valid != 0U)
  {
    return (float)max_temp / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_max_temp_deci_c / 10.0f;
  }

  return 0.0f;
}

static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t i;
  uint16_t local_max = 0U;
  uint16_t local_min = 0xFFFFU;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;

  if (App_IsFresh(now, state->cell_extrema_updated_ms) != 0U)
  {
    *max_mv = state->max_cell_voltage_mv;
    *min_mv = state->min_cell_voltage_mv;
    *max_no = (uint32_t)state->max_cell_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_cell_index_zero_based + 1U;
    return;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != APP_MODULE_VOLTAGE_VALID_MASK)
    {
      *max_mv = 0U;
      *min_mv = 0U;
      *max_no = 0U;
      *min_no = 0U;
      return;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    if (state->cell_voltage_mv[i] >= local_max)
    {
      local_max = state->cell_voltage_mv[i];
      local_max_no = i + 1U;
    }
    if (state->cell_voltage_mv[i] <= local_min)
    {
      local_min = state->cell_voltage_mv[i];
      local_min_no = i + 1U;
    }
  }

  *max_mv = local_max;
  *min_mv = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
}

static uint8_t App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                                  int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t module_idx;
  uint32_t temp_idx;
  int16_t local_max = INT16_MIN;
  int16_t local_min = INT16_MAX;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;
  uint8_t has_valid = 0U;

  if (App_IsFresh(now, state->temp_extrema_updated_ms) != 0U)
  {
    *max_temp = state->max_temp_deci_c;
    *min_temp = state->min_temp_deci_c;
    *max_no = (uint32_t)state->max_temp_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_temp_index_zero_based + 1U;
    return 1U;
  }

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t temp_base = module_idx * APP_TEMPS_PER_MODULE;

    if (App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_temp_valid[module_idx] == 0U)
    {
      *max_temp = 0;
      *min_temp = 0;
      *max_no = 0U;
      *min_no = 0U;
      return 0U;
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) == 0U)
      {
        continue;
      }

      if (state->cell_temp_deci_c[temp_base + temp_idx] >= local_max)
      {
        local_max = state->cell_temp_deci_c[temp_base + temp_idx];
        local_max_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }
      if (state->cell_temp_deci_c[temp_base + temp_idx] <= local_min)
      {
        local_min = state->cell_temp_deci_c[temp_base + temp_idx];
        local_min_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }

      has_valid = 1U;
    }
  }

  if (has_valid == 0U)
  {
    *max_temp = 0;
    *min_temp = 0;
    *max_no = 0U;
    *min_no = 0U;
    return 0U;
  }

  *max_temp = local_max;
  *min_temp = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
  return 1U;
}

static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  uint32_t module_idx;

  frame->modules_count = 0U;

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t cell_base;
    uint32_t temp_base;
    uint32_t cell_idx;
    uint32_t temp_idx;
    uint32_t *voltage_field;
    int32_t *temp_field;

    if (App_IsFresh(now, state->module_voltage_updated_ms[module_idx]) == 0U ||
        App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_voltage_valid[module_idx] != APP_MODULE_VOLTAGE_VALID_MASK ||
        state->module_temp_valid[module_idx] == 0U)
    {
      continue;
    }

    frame->modules[frame->modules_count] = g_battery_module_prototype;
    frame->modules[frame->modules_count].module_id = module_idx + 1U;

    voltage_field = &frame->modules[frame->modules_count].v01;
    temp_field = &frame->modules[frame->modules_count].t1;
    cell_base = module_idx * APP_CELLS_PER_MODULE;
    temp_base = module_idx * APP_TEMPS_PER_MODULE;

    for (cell_idx = 0U; cell_idx < APP_CELLS_PER_MODULE; ++cell_idx)
    {
      voltage_field[cell_idx] = state->cell_voltage_mv[cell_base + cell_idx];
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) != 0U)
      {
        temp_field[temp_idx] = state->cell_temp_deci_c[temp_base + temp_idx];
      }
    }

    frame->modules_count++;
  }

  return (frame->modules_count > 0U) ? 1U : 0U;
}

static uint8_t App_BuildMotorStates(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  static const uint8_t motor_index[APP_MOTOR_COUNT] = {
    APP_MOTOR_FL,
    APP_MOTOR_FR,
    APP_MOTOR_RL,
    APP_MOTOR_RR
  };
  static const fsae_MotorPosition motor_position[APP_MOTOR_COUNT] = {
    fsae_MotorPosition_MOTOR_POSITION_FRONT_LEFT,
    fsae_MotorPosition_MOTOR_POSITION_FRONT_RIGHT,
    fsae_MotorPosition_MOTOR_POSITION_REAR_LEFT,
    fsae_MotorPosition_MOTOR_POSITION_REAR_RIGHT
  };
  uint8_t torque_fresh = App_IsFresh(now, state->motor_torque_updated_ms);
  uint8_t diag_fresh = App_IsFresh(now, state->motor_diag_updated_ms);
  uint8_t rpm_fresh = App_IsFresh(now, state->motor_rpm_updated_ms);
  uint8_t motor_temp_fresh = App_IsFresh(now, state->motor_temp_updated_ms);
  uint8_t inverter_temp_fresh = App_IsFresh(now, state->motor_inverter_temp_updated_ms);
  uint8_t igbt_temp_fresh = App_IsFresh(now, state->motor_igbt_temp_updated_ms);
  uint8_t logic_state_fresh = App_IsFresh(now, state->motor_logic_state_updated_ms);
  uint32_t out_idx;

  if ((torque_fresh == 0U) && (diag_fresh == 0U) && (rpm_fresh == 0U) &&
      (motor_temp_fresh == 0U) && (inverter_temp_fresh == 0U) && (igbt_temp_fresh == 0U) &&
      (logic_state_fresh == 0U))
  {
    frame->vehicle_state.motors_count = 0U;
    return 0U;
  }

  frame->vehicle_state.motors_count = APP_MOTOR_COUNT;
  for (out_idx = 0U; out_idx < APP_MOTOR_COUNT; ++out_idx)
  {
    uint8_t idx = motor_index[out_idx];
    fsae_MotorState *motor = &frame->vehicle_state.motors[out_idx];

    *motor = g_motor_state_prototype;
    motor->position = motor_position[out_idx];
    if (rpm_fresh != 0U)
    {
      motor->rpm = state->motor_rpm[idx];
    }
    if (torque_fresh != 0U)
    {
      motor->torque_nm = state->motor_torque_0p1pct[idx];
    }
    if (motor_temp_fresh != 0U)
    {
      motor->motor_temp_dc = state->motor_temp_deci_c[idx];
    }
    if (inverter_temp_fresh != 0U)
    {
      motor->inverter_temp_dc = state->motor_inverter_temp_deci_c[idx];
    }
    if (igbt_temp_fresh != 0U)
    {
      motor->igbt_temp_dc = state->motor_igbt_temp_deci_c[idx];
    }
    if (diag_fresh != 0U)
    {
      motor->diagnostic_number = state->motor_diagnostic_number[idx];
      motor->motor_error = (int32_t)state->motor_diagnostic_number[idx];
    }
    if (logic_state_fresh != 0U)
    {
      motor->has_logic_state = true;
      motor->logic_state = state->motor_logic_state[idx];
    }
  }

  return 1U;
}

static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules)
{
  AppTelemetryState *snapshot = &g_tx_snapshot;
  fsae_TelemetryFrame *frame = &g_tx_frame;
  pb_ostream_t stream;
  uint32_t max_mv;
  uint32_t min_mv;
  uint32_t max_mv_no;
  uint32_t min_mv_no;
  int32_t max_temp;
  int32_t min_temp;
  uint32_t max_temp_no;
  uint32_t min_temp_no;
  uint8_t temp_extrema_valid;
  float hv_voltage;
  float hv_current;
  float battery_temp_max;
  fsae_DrivingMode driving_mode;

  __disable_irq();
  *snapshot = *((const AppTelemetryState *)&g_app_state);
  __enable_irq();

  *frame = g_tx_frame_prototype;
  frame->timestamp_ms = now;
  frame->frame_id = ++g_frame_counter;
  hv_voltage = App_GetHvVoltage(snapshot, now);
  hv_current = App_GetHvCurrent(snapshot, now);
  frame->hv_voltage = hv_voltage;
  frame->hv_current = hv_current;
  if (App_IsFresh(now, snapshot->can2_datalogger_updated_ms) != 0U)
  {
    frame->apps_position = (float)snapshot->can2_apps_open_deci_pct / 10.0f;
    frame->brake_pressure = (float)snapshot->can2_oil_pressure_milli_kpa / 1000.0f;
    frame->steering_angle = (float)snapshot->can2_steering_angle_deci_deg / 10.0f;
  }
  frame->battery_soc = App_GetBatterySoc(snapshot, now);
  frame->ready_to_drive = App_GetReadyToDrive(snapshot, now);
  frame->vcu_status = App_GetVcuStatus(snapshot, now);
  frame->fault_code = App_GetFaultCode(snapshot, now);
  frame->battery_fault_code = frame->fault_code;
  driving_mode = App_GetDrivingMode(snapshot, now);

  App_GetCellExtrema(snapshot, now, &max_mv, &min_mv, &max_mv_no, &min_mv_no);
  temp_extrema_valid = App_GetTempExtrema(snapshot, now, &max_temp, &min_temp, &max_temp_no, &min_temp_no);

  frame->max_cell_voltage = max_mv;
  frame->min_cell_voltage = min_mv;
  frame->max_cell_voltage_no = max_mv_no;
  frame->min_cell_voltage_no = min_mv_no;
  frame->max_temp = max_temp;
  frame->min_temp = min_temp;
  frame->max_temp_no = max_temp_no;
  frame->min_temp_no = min_temp_no;
  battery_temp_max = App_GetBatteryTempMax(snapshot, now, max_temp, temp_extrema_valid);
  frame->battery_temp_max = battery_temp_max;

  frame->has_header = true;
  frame->header.timestamp_ms = now;
  frame->header.seq = g_frame_counter;
  frame->header.source_id = 1U;
  frame->has_fast_telemetry = true;
  frame->fast_telemetry.hv_voltage_dv = (int32_t)(hv_voltage * 10.0f);
  frame->fast_telemetry.hv_current_ma = (int32_t)(hv_current * 1000.0f);
  frame->fast_telemetry.battery_temp_max_dc = (int32_t)(battery_temp_max * 10.0f);
  frame->fast_telemetry.driving_mode = driving_mode;
  if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
  {
    frame->fast_telemetry.speed_kmh = snapshot->can2_gps_speed_kmh;
  }
  frame->has_vehicle_state = true;
  frame->vehicle_state.driving_mode = driving_mode;
  if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
  {
    frame->vehicle_state.speed_kmh = snapshot->can2_gps_speed_kmh;
  }
  if (App_IsFresh(now, snapshot->can2_datalogger_updated_ms) != 0U)
  {
    frame->vehicle_state.throttle_position = (uint32_t)((snapshot->can2_apps_open_deci_pct + 5U) / 10U);
  }
  frame->vehicle_state.vcu_status = (frame->vcu_status == 0U) ?
                                    fsae_VcuStatus_VCU_STATUS_UNSPECIFIED :
                                    fsae_VcuStatus_VCU_STATUS_HV_ENABLED;
  App_BuildAlarms(snapshot, now, frame);
  (void)App_BuildMotorStates(snapshot, now, frame);

  if ((App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_accel_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_gyro_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_yaw_updated_ms) != 0U))
  {
    frame->has_motion = true;
    if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
    {
      frame->motion.gps_speed_kmh = snapshot->can2_gps_speed_kmh;
    }
    if (App_IsFresh(now, snapshot->can2_accel_updated_ms) != 0U)
    {
      frame->motion.accel_x_g = (float)snapshot->can2_accel_x_raw * 0.00048828125f;
      frame->motion.accel_y_g = (float)snapshot->can2_accel_y_raw * 0.00048828125f;
      frame->motion.accel_z_g = (float)snapshot->can2_accel_z_raw * 0.00048828125f;
    }
    if (App_IsFresh(now, snapshot->can2_gyro_updated_ms) != 0U)
    {
      frame->motion.yaw_rate_dps = (float)snapshot->can2_yaw_rate_raw * 0.0610352f;
    }
    if (App_IsFresh(now, snapshot->can2_yaw_updated_ms) != 0U)
    {
      frame->motion.yaw_deg = (float)snapshot->can2_yaw_raw * 0.005493f;
    }
  }

  if (App_IsFresh(now, snapshot->motor_rpm_updated_ms) != 0U)
  {
    frame->motor_rpm = snapshot->motor_rpm[APP_MOTOR_RL];
  }
  if (App_IsFresh(now, snapshot->motor_temp_updated_ms) != 0U)
  {
    frame->motor_temp = (float)snapshot->motor_temp_deci_c[APP_MOTOR_RL] / 10.0f;
  }
  if (App_IsFresh(now, snapshot->motor_inverter_temp_updated_ms) != 0U)
  {
    frame->inverter_temp = (float)snapshot->motor_inverter_temp_deci_c[APP_MOTOR_RL] / 10.0f;
  }

  if ((App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U))
  {
    const uint8_t ivt_current_fresh = App_IsFresh(now, snapshot->ivt_current_updated_ms);
    const uint8_t ivt_voltage_u1_fresh = App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms);

    frame->has_ivt_telemetry = true;
    if (ivt_current_fresh != 0U)
    {
      frame->ivt_telemetry.current_ma = snapshot->ivt_current_ma;
      frame->ivt_telemetry.current_state = snapshot->ivt_current_state;
    }
    if (ivt_voltage_u1_fresh != 0U)
    {
      frame->ivt_telemetry.voltage_u1_mv = snapshot->ivt_voltage_u1_mv;
      frame->ivt_telemetry.voltage_u1_state = snapshot->ivt_voltage_u1_state;
    }
    if (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U)
    {
      frame->ivt_telemetry.power_w = snapshot->ivt_power_w;
    }
    else if ((ivt_current_fresh != 0U) && (ivt_voltage_u1_fresh != 0U))
    {
      frame->ivt_telemetry.power_w =
          (int32_t)(((int64_t)snapshot->ivt_voltage_u1_mv * (int64_t)snapshot->ivt_current_ma) / 1000000LL);
    }
    if (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U)
    {
      frame->ivt_telemetry.energy_wh = snapshot->ivt_energy_wh;
      frame->ivt_telemetry.energy_state = snapshot->ivt_energy_state;
    }
  }

  switch (App_GetEnergyMeterSource(snapshot, now))
  {
    case APP_ENERGY_METER_SOURCE_FS:
      frame->has_energy_meter = true;
      frame->energy_meter.source = APP_ENERGY_METER_SOURCE_FS;
      if (App_IsFresh(now, snapshot->fs_current_updated_ms) != 0U)
      {
        frame->energy_meter.current_ma = snapshot->fs_current_ma;
      }
      if (App_IsFresh(now, snapshot->fs_voltage_updated_ms) != 0U)
      {
        frame->energy_meter.voltage_mv = snapshot->fs_voltage_mv;
      }
      if (App_IsFresh(now, snapshot->fs_energy_updated_ms) != 0U)
      {
        frame->energy_meter.energy_wh = snapshot->fs_energy_wh;
      }
      if (App_IsFresh(now, snapshot->fs_power_updated_ms) != 0U)
      {
        frame->energy_meter.power_w = snapshot->fs_power_w;
      }
      if (App_IsFresh(now, snapshot->fs_status_updated_ms) != 0U)
      {
        frame->energy_meter.status = snapshot->fs_status;
        frame->energy_meter.msg_counter = snapshot->fs_msg_counter;
      }
      if ((App_IsFresh(now, snapshot->fs_power_updated_ms) == 0U) &&
          (App_IsFresh(now, snapshot->fs_current_updated_ms) != 0U) &&
          (App_IsFresh(now, snapshot->fs_voltage_updated_ms) != 0U))
      {
        frame->energy_meter.power_w =
            (int32_t)(((int64_t)snapshot->fs_voltage_mv * (int64_t)snapshot->fs_current_ma) / 1000000LL);
      }
      break;

    case APP_ENERGY_METER_SOURCE_IVT:
      frame->has_energy_meter = true;
      frame->energy_meter.source = APP_ENERGY_METER_SOURCE_IVT;
      if (App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U)
      {
        frame->energy_meter.current_ma = snapshot->ivt_current_ma;
        frame->energy_meter.status = snapshot->ivt_current_state;
      }
      if (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U)
      {
        frame->energy_meter.voltage_mv = snapshot->ivt_voltage_u1_mv;
      }
      if (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U)
      {
        frame->energy_meter.energy_wh = snapshot->ivt_energy_wh;
      }
      if (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U)
      {
        frame->energy_meter.power_w = snapshot->ivt_power_w;
      }
      else if ((App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U) &&
          (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U))
      {
        frame->energy_meter.power_w =
            (int32_t)(((int64_t)snapshot->ivt_voltage_u1_mv * (int64_t)snapshot->ivt_current_ma) / 1000000LL);
      }
      break;

    default:
      break;
  }

  if (include_modules != 0U)
  {
    (void)App_BuildModules(snapshot, now, frame);
  }

  stream = pb_ostream_from_buffer(g_pb_buffer, sizeof(g_pb_buffer));
  if (!pb_encode(&stream, fsae_TelemetryFrame_fields, frame))
  {
    return HAL_ERROR;
  }

  return App_RS485_Transmit(g_pb_buffer, (uint16_t)stream.bytes_written);
}

static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  uint32_t start_ms;

  (void)HAL_UART_AbortReceive(&huart2);

  status = HAL_UART_Transmit(&huart2, (uint8_t *)data, size, APP_RS485_TX_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    (void)App_CommandIo_RestartRx();
    return status;
  }

  start_ms = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) > APP_RS485_TX_TIMEOUT_MS)
    {
      (void)App_CommandIo_RestartRx();
      return HAL_TIMEOUT;
    }
  }

  (void)App_CommandIo_RestartRx();
  g_last_rs485_tx_ms = HAL_GetTick();
  return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if APP_DEBUG_CLI_ENABLE
  if (huart->Instance == USART1)
  {
    uint8_t rx = g_cli_rx_byte;

    if ((rx == '\r') || (rx == '\n'))
    {
      if ((g_cli_rx_length > 0U) && (g_cli_command_ready == 0U))
      {
        memcpy((void *)g_cli_command_buffer, (const void *)g_cli_rx_buffer, g_cli_rx_length);
        g_cli_command_buffer[g_cli_rx_length] = '\0';
        g_cli_command_ready = 1U;
      }
      g_cli_rx_length = 0U;
    }
    else if (g_cli_rx_length < (APP_DEBUG_CLI_BUFFER_SIZE - 1U))
    {
      g_cli_rx_buffer[g_cli_rx_length] = (char)rx;
      g_cli_rx_length++;
    }
    else
    {
      g_cli_rx_length = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_cli_rx_byte, 1U);
  }
#endif

  if (huart->Instance == USART2)
  {
    App_CommandIo_OnRxComplete();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    (void)App_CommandIo_RestartRx();
  }
}
