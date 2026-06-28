#include "app_command_io.h"

#include <string.h>

#include "app_command_parser.h"
#include "app_config.h"
#include "usart.h"

static volatile uint8_t g_command_rx_byte;
static volatile uint8_t g_command_rx_buffer[APP_COMMAND_RX_BUFFER_SIZE];
static volatile uint8_t g_command_rx_length;
static volatile uint8_t g_command_rx_ready;
static volatile uint8_t g_command_rx_overflow;
static volatile uint32_t g_command_rx_updated_ms;
static uint8_t g_command_pending_value;
static uint8_t g_command_pending;
static AppCommandIoSendCallback g_send_callback;
static AppCommandIoParseDebugCallback g_parse_debug_callback;
static AppCommandIoTxDebugCallback g_tx_debug_callback;

static void App_CommandIo_RxByte(uint8_t rx);
static void App_CommandIo_Queue(uint8_t command_value);
static void App_CommandIo_ServiceTx(void);

HAL_StatusTypeDef App_CommandIo_Init(AppCommandIoSendCallback send_callback,
                                     AppCommandIoParseDebugCallback parse_debug_callback,
                                     AppCommandIoTxDebugCallback tx_debug_callback)
{
  g_command_rx_length = 0U;
  g_command_rx_ready = 0U;
  g_command_rx_overflow = 0U;
  g_command_rx_updated_ms = 0U;
  g_command_pending_value = 0U;
  g_command_pending = 0U;
  g_send_callback = send_callback;
  g_parse_debug_callback = parse_debug_callback;
  g_tx_debug_callback = tx_debug_callback;

  return App_CommandIo_RestartRx();
}

void App_CommandIo_Poll(uint32_t now)
{
  uint8_t buffer[APP_COMMAND_RX_BUFFER_SIZE];
  uint8_t length = 0U;
  uint8_t ready = 0U;
  uint8_t command_value = 0U;
  uint8_t parse_ok;

  __disable_irq();
  if ((g_command_rx_ready != 0U) ||
      ((g_command_rx_length > 0U) && ((uint32_t)(now - g_command_rx_updated_ms) >= APP_COMMAND_RX_IDLE_MS)))
  {
    ready = 1U;
    length = g_command_rx_length;
    memcpy(buffer, (const void *)g_command_rx_buffer, length);
    g_command_rx_length = 0U;
    g_command_rx_ready = 0U;
    g_command_rx_overflow = 0U;
  }
  __enable_irq();

  if (ready != 0U)
  {
    parse_ok = App_CommandTryParse(buffer, length, &command_value);
    if (g_parse_debug_callback != NULL)
    {
      g_parse_debug_callback(buffer, length, parse_ok, command_value);
    }

    if (parse_ok != 0U)
    {
      App_CommandIo_Queue(command_value);
    }
  }

  App_CommandIo_ServiceTx();
}

HAL_StatusTypeDef App_CommandIo_RestartRx(void)
{
  return HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_command_rx_byte, 1U);
}

void App_CommandIo_OnRxComplete(void)
{
  App_CommandIo_RxByte(g_command_rx_byte);
  (void)App_CommandIo_RestartRx();
}

static void App_CommandIo_RxByte(uint8_t rx)
{
  g_command_rx_updated_ms = HAL_GetTick();

  if ((rx == '\r') || (rx == '\n') || (rx == 0U))
  {
    if (g_command_rx_length > 0U)
    {
      g_command_rx_ready = 1U;
    }
    return;
  }

  if (g_command_rx_ready != 0U)
  {
    return;
  }

  if (g_command_rx_length < APP_COMMAND_RX_BUFFER_SIZE)
  {
    g_command_rx_buffer[g_command_rx_length] = rx;
    g_command_rx_length++;
  }
  else
  {
    g_command_rx_length = 0U;
    g_command_rx_overflow = 1U;
  }
}

static void App_CommandIo_Queue(uint8_t command_value)
{
  __disable_irq();
  g_command_pending_value = command_value;
  g_command_pending = 1U;
  __enable_irq();
}

static void App_CommandIo_ServiceTx(void)
{
  uint8_t command_value;
  HAL_StatusTypeDef status;

  __disable_irq();
  if (g_command_pending == 0U)
  {
    __enable_irq();
    return;
  }
  command_value = g_command_pending_value;
  g_command_pending = 0U;
  __enable_irq();

  if (g_send_callback == NULL)
  {
    return;
  }

  status = g_send_callback(command_value);
  if (g_tx_debug_callback != NULL)
  {
    g_tx_debug_callback(command_value, status, HAL_GetTick());
  }

  if (status != HAL_OK)
  {
    App_CommandIo_Queue(command_value);
  }
}
