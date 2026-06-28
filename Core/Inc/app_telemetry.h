#ifndef __APP_TELEMETRY_H__
#define __APP_TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_state.h"
#include "main.h"

typedef HAL_StatusTypeDef (*AppTelemetryTransmitFn)(const uint8_t *data, uint16_t size);

void App_Telemetry_Init(void);
uint32_t App_Telemetry_FrameCount(void);
HAL_StatusTypeDef App_Telemetry_Send(volatile const AppTelemetryState *state,
                                     uint32_t now,
                                     uint8_t include_modules,
                                     AppTelemetryTransmitFn transmit);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEMETRY_H__ */
