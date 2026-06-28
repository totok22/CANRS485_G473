#include "app.h"

#include <string.h>

#include "fdcan.h"
#include "gpio.h"
#include "main.h"

#define APP_FDCAN_RX_MAX_FRAMES_PER_IRQ 8U
#define APP_LED_BLINK_MS                500U

static volatile AppCanRxStats g_can_stats[APP_CAN_BUS_COUNT];
static volatile uint32_t g_mcp2518_irq_count;
static uint32_t g_last_led_tick_ms;
static GPIO_PinState g_led_state = GPIO_PIN_RESET;

static void App_FDCAN_ConfigAcceptAll(FDCAN_HandleTypeDef *hfdcan);
static AppCanBus App_BusFromFdcan(const FDCAN_HandleTypeDef *hfdcan);
static uint8_t App_FdcanDlcToBytes(uint32_t dlc);
static void App_RecordFdcanFrame(AppCanBus bus, const FDCAN_RxHeaderTypeDef *header, const uint8_t *data);
static uint8_t App_HasAnyFdcanTraffic(void);

void App_Init(void)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCP2518_CS_GPIO_Port, MCP2518_CS_Pin, GPIO_PIN_SET);

  App_FDCAN_ConfigAcceptAll(&hfdcan1);
  App_FDCAN_ConfigAcceptAll(&hfdcan2);
  App_FDCAN_ConfigAcceptAll(&hfdcan3);
}

void App_Run(void)
{
  const uint32_t now = HAL_GetTick();

  if (App_HasAnyFdcanTraffic())
  {
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    g_led_state = GPIO_PIN_SET;
    return;
  }

  if ((now - g_last_led_tick_ms) >= APP_LED_BLINK_MS)
  {
    g_last_led_tick_ms = now;
    g_led_state = (g_led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, g_led_state);
  }
}

void App_GetCanRxStats(AppCanBus bus, AppCanRxStats *stats)
{
  if ((stats == NULL) || (bus >= APP_CAN_BUS_COUNT))
  {
    return;
  }

  __disable_irq();
  *stats = g_can_stats[bus];
  __enable_irq();
}

uint32_t App_GetMcp2518IrqCount(void)
{
  return g_mcp2518_irq_count;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint32_t handled = 0;
  const AppCanBus bus = App_BusFromFdcan(hfdcan);

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while ((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) &&
         (handled < APP_FDCAN_RX_MAX_FRAMES_PER_IRQ))
  {
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[64] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      g_can_stats[bus].dropped_frames++;
      break;
    }

    App_RecordFdcanFrame(bus, &header, data);
    handled++;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MCP2518_INT_Pin)
  {
    g_mcp2518_irq_count++;
  }
}

static void App_FDCAN_ConfigAcceptAll(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000U;
  filter.FilterID2 = 0x000U;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0;
  filter.FilterID1 = 0x00000000UL;
  filter.FilterID2 = 0x00000000UL;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    Error_Handler();
  }
}

static AppCanBus App_BusFromFdcan(const FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN1)
  {
    return APP_CAN_BUS_CANA;
  }
  if (hfdcan->Instance == FDCAN2)
  {
    return APP_CAN_BUS_CANB;
  }
  return APP_CAN_BUS_CAN1;
}

static uint8_t App_FdcanDlcToBytes(uint32_t dlc)
{
  static const uint8_t dlc_to_bytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
                                         8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

  if (dlc < (sizeof(dlc_to_bytes) / sizeof(dlc_to_bytes[0])))
  {
    return dlc_to_bytes[dlc];
  }

  return 0U;
}

static void App_RecordFdcanFrame(AppCanBus bus, const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  AppCanRxStats *stats = (AppCanRxStats *)&g_can_stats[bus];
  uint8_t bytes = App_FdcanDlcToBytes(header->DataLength);

  stats->rx_frames++;
  stats->last_tick_ms = HAL_GetTick();
  stats->last_id = header->Identifier;
  stats->last_is_extended = (header->IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;

  if (header->RxFrameType == FDCAN_REMOTE_FRAME)
  {
    stats->remote_frames++;
    stats->last_dlc = 0U;
    return;
  }

  if (header->FDFormat != FDCAN_CLASSIC_CAN)
  {
    stats->fd_frames++;
  }

  if (bytes > sizeof(stats->last_data))
  {
    bytes = sizeof(stats->last_data);
  }

  stats->last_dlc = bytes;
  memcpy(stats->last_data, data, bytes);
}

static uint8_t App_HasAnyFdcanTraffic(void)
{
  return (g_can_stats[APP_CAN_BUS_CANA].rx_frames > 0U) ||
         (g_can_stats[APP_CAN_BUS_CANB].rx_frames > 0U) ||
         (g_can_stats[APP_CAN_BUS_CAN1].rx_frames > 0U);
}
