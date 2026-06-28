#include "app.h"

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_can_decode.h"
#include "app_can_io.h"
#include "app_command_io.h"
#include "app_config.h"
#include "app_fault.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "app_utils.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"


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
static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size);

void App_Init(void)
{
  memset((void *)&g_app_state, 0, sizeof(g_app_state));
  App_Telemetry_Init();
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
  if (App_Telemetry_Send(&g_app_state, now, include_modules, App_RS485_Transmit) == HAL_OK)
  {
    g_last_base_tx_ms = now;
    if (include_modules != 0U)
    {
      g_last_module_tx_ms = now;
    }
  }
}

static void App_UpdateFaultCode(void)
{
  g_app_state.battery_fault_code = App_FaultCompute((const AppTelemetryState *)&g_app_state, HAL_GetTick());
}

static void App_SetStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (on != 0U) ? APP_LED_ON_STATE : APP_LED_OFF_STATE);
}

static void App_UpdateStatusLed(uint32_t now)
{
  uint8_t has_fault = (App_FaultCompute((const AppTelemetryState *)&g_app_state, now) != 0U) ? 1U : 0U;
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
  uint32_t frame_counter = App_Telemetry_FrameCount();

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
      (unsigned long)App_FaultCompute(&snapshot, now));
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
