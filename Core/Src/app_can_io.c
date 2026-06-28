#include "app_can_io.h"

#include "app_utils.h"
#include "fdcan.h"
#include "mcp2518fd.h"
#include "spi.h"

#define APP_CAN_RX_MAX_FRAMES_PER_IRQ       8U
#define APP_MCP2518_RX_MAX_FRAMES_PER_POLL  8U
#define APP_CANB_TX_QUEUE_DEPTH             16U

typedef struct
{
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} AppCanTxFrame;

static volatile AppCanTxFrame g_canb_tx_queue[APP_CANB_TX_QUEUE_DEPTH];
static volatile uint8_t g_canb_tx_head;
static volatile uint8_t g_canb_tx_tail;
static volatile uint8_t g_canb_tx_count;
static volatile uint32_t g_canb_tx_drop_count;
static uint8_t g_canc_ready;
static volatile uint32_t g_canc_rx_count;
static volatile uint32_t g_canc_rx_error_count;
static AppCanIoRxCallback g_rx_callback;

static HAL_StatusTypeDef App_CanIo_ConfigFilter(FDCAN_HandleTypeDef *hfdcan);
static AppFdcanBus App_CanIo_BusFromHandle(const FDCAN_HandleTypeDef *hfdcan);
static void App_CanIo_ConvertFdcanRxHeader(const FDCAN_RxHeaderTypeDef *src, AppCanRxHeader *dst);
static void App_CanIo_ConvertMcp2518RxHeader(const MCP2518FD_CanFrame *src, AppCanRxHeader *dst);
static void App_CanIo_ServiceCanBTxQueue(void);
static void App_CanIo_ServiceMcp2518Rx(void);

HAL_StatusTypeDef App_CanIo_Init(AppCanIoRxCallback rx_callback)
{
  g_rx_callback = rx_callback;
  g_canb_tx_head = 0U;
  g_canb_tx_tail = 0U;
  g_canb_tx_count = 0U;
  g_canb_tx_drop_count = 0U;
  g_canc_ready = 0U;
  g_canc_rx_count = 0U;
  g_canc_rx_error_count = 0U;

  if (App_CanIo_ConfigFilter(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (App_CanIo_ConfigFilter(&hfdcan2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (App_CanIo_ConfigFilter(&hfdcan3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_Init(&hspi1) == HAL_OK)
  {
    g_canc_ready = 1U;
    MCP2518FD_OnInterrupt();
  }

  return HAL_OK;
}

void App_CanIo_Service(void)
{
  App_CanIo_ServiceCanBTxQueue();
  App_CanIo_ServiceMcp2518Rx();
}

HAL_StatusTypeDef App_CanIo_SendCanBStd(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};

  if ((data == NULL) || (dlc > 8U))
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    return HAL_BUSY;
  }

  tx_header.Identifier = std_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = App_FdcanDlcFromBytes(dlc);
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, (uint8_t *)data);
}

uint8_t App_CanIo_QueueCanBStd(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
  uint8_t i;

  if ((data == NULL) || (dlc == 0U) || (dlc > 8U))
  {
    return 0U;
  }

  __disable_irq();
  if (g_canb_tx_count >= APP_CANB_TX_QUEUE_DEPTH)
  {
    g_canb_tx_drop_count++;
    __enable_irq();
    return 0U;
  }

  g_canb_tx_queue[g_canb_tx_head].std_id = std_id;
  g_canb_tx_queue[g_canb_tx_head].dlc = dlc;
  for (i = 0U; i < dlc; ++i)
  {
    g_canb_tx_queue[g_canb_tx_head].data[i] = data[i];
  }
  g_canb_tx_head = (uint8_t)((g_canb_tx_head + 1U) % APP_CANB_TX_QUEUE_DEPTH);
  g_canb_tx_count++;
  __enable_irq();

  return 1U;
}

void App_CanIo_GetStatus(AppCanIoStatus *status)
{
  if (status == NULL)
  {
    return;
  }

  __disable_irq();
  status->canb_tx_count = g_canb_tx_count;
  status->canb_tx_drop_count = g_canb_tx_drop_count;
  status->canc_ready = g_canc_ready;
  status->canc_interrupt_pending = MCP2518FD_HasPendingInterrupt();
  status->canc_rx_count = g_canc_rx_count;
  status->canc_rx_error_count = g_canc_rx_error_count;
  __enable_irq();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  AppCanRxHeader header;
  FDCAN_RxHeaderTypeDef fdcan_header;
  uint8_t data[64];
  uint8_t frames_processed = 0U;
  AppFdcanBus bus = App_CanIo_BusFromHandle(hfdcan);

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while ((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) &&
         (frames_processed < APP_CAN_RX_MAX_FRAMES_PER_IRQ))
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_header, data) != HAL_OK)
    {
      return;
    }

    if ((fdcan_header.RxFrameType == FDCAN_DATA_FRAME) && (fdcan_header.FDFormat == FDCAN_CLASSIC_CAN))
    {
      App_CanIo_ConvertFdcanRxHeader(&fdcan_header, &header);
      if (g_rx_callback != NULL)
      {
        g_rx_callback(bus, &header, data);
      }
    }
    frames_processed++;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MCP2518_INT_Pin)
  {
    MCP2518FD_OnInterrupt();
  }
}

static HAL_StatusTypeDef App_CanIo_ConfigFilter(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000U;
  filter.FilterID2 = 0x000U;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0U;
  filter.FilterID1 = 0x00000000UL;
  filter.FilterID2 = 0x00000000UL;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                      FDCAN_ACCEPT_IN_RX_FIFO0,
                                      FDCAN_ACCEPT_IN_RX_FIFO0,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE);
}

static AppFdcanBus App_CanIo_BusFromHandle(const FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN3)
  {
    return APP_FDCAN_BUS_CAN1;
  }
  if (hfdcan->Instance == FDCAN2)
  {
    return APP_FDCAN_BUS_CANB;
  }
  return APP_FDCAN_BUS_CANA;
}

static void App_CanIo_ConvertFdcanRxHeader(const FDCAN_RxHeaderTypeDef *src, AppCanRxHeader *dst)
{
  dst->IDE = (src->IdType == FDCAN_EXTENDED_ID) ? APP_CAN_ID_EXT : APP_CAN_ID_STD;
  dst->StdId = (src->IdType == FDCAN_STANDARD_ID) ? src->Identifier : 0U;
  dst->ExtId = (src->IdType == FDCAN_EXTENDED_ID) ? src->Identifier : 0U;
  dst->DLC = App_FdcanDlcToBytes(src->DataLength);
}

static void App_CanIo_ConvertMcp2518RxHeader(const MCP2518FD_CanFrame *src, AppCanRxHeader *dst)
{
  dst->IDE = (src->id_type == MCP2518FD_ID_EXTENDED) ? APP_CAN_ID_EXT : APP_CAN_ID_STD;
  dst->StdId = (src->id_type == MCP2518FD_ID_STANDARD) ? src->id : 0U;
  dst->ExtId = (src->id_type == MCP2518FD_ID_EXTENDED) ? src->id : 0U;
  dst->DLC = src->dlc;
}

static void App_CanIo_ServiceCanBTxQueue(void)
{
  AppCanTxFrame frame;
  uint8_t i;

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U)
  {
    __disable_irq();
    if (g_canb_tx_count == 0U)
    {
      __enable_irq();
      return;
    }

    frame.std_id = g_canb_tx_queue[g_canb_tx_tail].std_id;
    frame.dlc = g_canb_tx_queue[g_canb_tx_tail].dlc;
    for (i = 0U; i < frame.dlc; ++i)
    {
      frame.data[i] = g_canb_tx_queue[g_canb_tx_tail].data[i];
    }
    __enable_irq();

    if (App_CanIo_SendCanBStd(frame.std_id, frame.data, frame.dlc) != HAL_OK)
    {
      return;
    }

    __disable_irq();
    if (g_canb_tx_count > 0U)
    {
      g_canb_tx_tail = (uint8_t)((g_canb_tx_tail + 1U) % APP_CANB_TX_QUEUE_DEPTH);
      g_canb_tx_count--;
    }
    __enable_irq();
  }
}

static void App_CanIo_ServiceMcp2518Rx(void)
{
  MCP2518FD_CanFrame frame;
  AppCanRxHeader header;
  uint8_t received;
  uint8_t frames_processed = 0U;

  if ((g_canc_ready == 0U) || (MCP2518FD_IsReady() == 0U))
  {
    return;
  }

  if (MCP2518FD_HasPendingInterrupt() == 0U)
  {
    return;
  }

  while (frames_processed < APP_MCP2518_RX_MAX_FRAMES_PER_POLL)
  {
    if (MCP2518FD_Receive(&frame, &received) != HAL_OK)
    {
      g_canc_rx_error_count++;
      return;
    }
    if (received == 0U)
    {
      return;
    }

    App_CanIo_ConvertMcp2518RxHeader(&frame, &header);
    if (g_rx_callback != NULL)
    {
      g_rx_callback(APP_FDCAN_BUS_CANC, &header, frame.data);
    }
    g_canc_rx_count++;
    frames_processed++;
  }
}
