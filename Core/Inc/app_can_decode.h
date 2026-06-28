#ifndef __APP_CAN_DECODE_H__
#define __APP_CAN_DECODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_can_io.h"
#include "app_state.h"

#define APP_CAN_DECODE_EVENT_NONE          0U
#define APP_CAN_DECODE_EVENT_FAULT_REFRESH (1U << 0)

uint8_t App_CanDecode_Process(volatile AppTelemetryState *state,
                              AppFdcanBus bus,
                              const AppCanRxHeader *header,
                              const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAN_DECODE_H__ */
