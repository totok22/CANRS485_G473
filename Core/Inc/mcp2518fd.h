#ifndef __MCP2518FD_H__
#define __MCP2518FD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#define MCP2518FD_ID_STANDARD 0U
#define MCP2518FD_ID_EXTENDED 1U

typedef struct
{
  uint32_t id;
  uint8_t id_type;
  uint8_t dlc;
  uint8_t data[8];
} MCP2518FD_CanFrame;

HAL_StatusTypeDef MCP2518FD_Init(SPI_HandleTypeDef *hspi);
uint8_t MCP2518FD_IsReady(void);
void MCP2518FD_OnInterrupt(void);
uint8_t MCP2518FD_HasPendingInterrupt(void);
HAL_StatusTypeDef MCP2518FD_Receive(MCP2518FD_CanFrame *frame, uint8_t *received);
HAL_StatusTypeDef MCP2518FD_Transmit(const MCP2518FD_CanFrame *frame);
HAL_StatusTypeDef MCP2518FD_ReadRegister(uint16_t address, uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __MCP2518FD_H__ */
