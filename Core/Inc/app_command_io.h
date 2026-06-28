#ifndef __APP_COMMAND_IO_H__
#define __APP_COMMAND_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef HAL_StatusTypeDef (*AppCommandIoSendCallback)(uint8_t command_value);
typedef void (*AppCommandIoParseDebugCallback)(const uint8_t *data, uint8_t length,
                                               uint8_t parse_ok, uint8_t command_value);
typedef void (*AppCommandIoTxDebugCallback)(uint8_t command_value,
                                            HAL_StatusTypeDef status,
                                            uint32_t now);

HAL_StatusTypeDef App_CommandIo_Init(AppCommandIoSendCallback send_callback,
                                     AppCommandIoParseDebugCallback parse_debug_callback,
                                     AppCommandIoTxDebugCallback tx_debug_callback);
void App_CommandIo_Poll(uint32_t now);
HAL_StatusTypeDef App_CommandIo_RestartRx(void);
void App_CommandIo_OnRxComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_COMMAND_IO_H__ */
