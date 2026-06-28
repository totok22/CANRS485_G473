#ifndef APP_H
#define APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  APP_CAN_BUS_CANA = 0,
  APP_CAN_BUS_CANB,
  APP_CAN_BUS_CAN1,
  APP_CAN_BUS_CANC,
  APP_CAN_BUS_COUNT
} AppCanBus;

typedef struct
{
  uint32_t rx_frames;
  uint32_t dropped_frames;
  uint32_t remote_frames;
  uint32_t fd_frames;
  uint32_t last_id;
  uint32_t last_tick_ms;
  uint8_t last_dlc;
  uint8_t last_is_extended;
  uint8_t last_data[8];
} AppCanRxStats;

void App_Init(void);
void App_Run(void);
void App_GetCanRxStats(AppCanBus bus, AppCanRxStats *stats);
uint32_t App_GetMcp2518IrqCount(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
