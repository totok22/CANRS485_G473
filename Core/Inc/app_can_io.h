#ifndef __APP_CAN_IO_H__
#define __APP_CAN_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32g4xx_hal.h"

#define APP_CAN_ID_STD 0U
#define APP_CAN_ID_EXT 1U

typedef enum
{
  APP_FDCAN_BUS_CANA = 0,
  APP_FDCAN_BUS_CANB,
  APP_FDCAN_BUS_CAN1,
  APP_FDCAN_BUS_CANC
} AppFdcanBus;

typedef struct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint8_t DLC;
} AppCanRxHeader;

typedef struct
{
  uint8_t canb_tx_count;
  uint32_t canb_tx_drop_count;
  uint8_t canc_ready;
  uint8_t canc_interrupt_pending;
  uint32_t canc_rx_count;
  uint32_t canc_rx_error_count;
} AppCanIoStatus;

typedef void (*AppCanIoRxCallback)(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data);

HAL_StatusTypeDef App_CanIo_Init(AppCanIoRxCallback rx_callback);
void App_CanIo_Service(void);
HAL_StatusTypeDef App_CanIo_SendCanBStd(uint32_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t App_CanIo_QueueCanBStd(uint32_t std_id, const uint8_t *data, uint8_t dlc);
void App_CanIo_GetStatus(AppCanIoStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAN_IO_H__ */
