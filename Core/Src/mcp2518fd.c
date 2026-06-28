#include "mcp2518fd.h"

#include <string.h>

#include "main.h"

#define MCP2518FD_CMD_RESET             0x0U
#define MCP2518FD_CMD_WRITE             0x2U
#define MCP2518FD_CMD_READ              0x3U

#define MCP2518FD_REG_C1CON             0x000U
#define MCP2518FD_REG_C1NBTCFG          0x004U
#define MCP2518FD_REG_C1DBTCFG          0x008U
#define MCP2518FD_REG_C1TDC             0x00CU
#define MCP2518FD_REG_C1INT             0x01CU
#define MCP2518FD_REG_C1FLTCON0         0x1D0U
#define MCP2518FD_REG_C1FLTOBJ0         0x1F0U
#define MCP2518FD_REG_C1MASK0           0x1F4U
#define MCP2518FD_REG_OSC               0xE00U
#define MCP2518FD_REG_IOCON             0xE04U
#define MCP2518FD_REG_ECCCON            0xE0CU

#define MCP2518FD_FIFO_CON(fifo)        (0x05CU + (((uint16_t)(fifo) - 1U) * 0x0CU))
#define MCP2518FD_FIFO_STA(fifo)        (MCP2518FD_FIFO_CON(fifo) + 0x04U)
#define MCP2518FD_FIFO_UA(fifo)         (MCP2518FD_FIFO_CON(fifo) + 0x08U)

#define MCP2518FD_C1CON_REQOP_POS       24U
#define MCP2518FD_C1CON_REQOP_MASK      (0x7UL << MCP2518FD_C1CON_REQOP_POS)
#define MCP2518FD_C1CON_OPMOD_POS       21U
#define MCP2518FD_C1CON_OPMOD_MASK      (0x7UL << MCP2518FD_C1CON_OPMOD_POS)
#define MCP2518FD_C1CON_BRSDIS          (1UL << 12)
#define MCP2518FD_MODE_NORMAL_FD        0x0UL
#define MCP2518FD_MODE_CONFIGURATION    0x4UL

#define MCP2518FD_OSC_SCLKRDY           (1UL << 8)
#define MCP2518FD_OSC_OSCRDY            (1UL << 9)
#define MCP2518FD_OSC_READY_MASK        (MCP2518FD_OSC_SCLKRDY | MCP2518FD_OSC_OSCRDY)

#define MCP2518FD_ECCCON_ECCEN          (1UL << 0)

#define MCP2518FD_FIFO_PLSIZE_8         (0UL << 29)
#define MCP2518FD_FIFO_FSIZE(depth)     ((((uint32_t)(depth) - 1UL) & 0x1FUL) << 24)
#define MCP2518FD_FIFO_TXREQ            (1UL << 9)
#define MCP2518FD_FIFO_UINC             (1UL << 8)
#define MCP2518FD_FIFO_TXEN             (1UL << 7)
#define MCP2518FD_FIFO_TFNRFNIF         (1UL << 0)

#define MCP2518FD_RX_FIFO               1U
#define MCP2518FD_TX_FIFO               2U
#define MCP2518FD_RX_FIFO_DEPTH         16U
#define MCP2518FD_TX_FIFO_DEPTH         8U
#define MCP2518FD_RX_FIFO_CONFIG        (MCP2518FD_FIFO_PLSIZE_8 | MCP2518FD_FIFO_FSIZE(MCP2518FD_RX_FIFO_DEPTH))
#define MCP2518FD_TX_FIFO_CONFIG        (MCP2518FD_FIFO_PLSIZE_8 | MCP2518FD_FIFO_FSIZE(MCP2518FD_TX_FIFO_DEPTH) | MCP2518FD_FIFO_TXEN)

#define MCP2518FD_FILTER0_TO_FIFO1      0x81U

#define MCP2518FD_SPI_TIMEOUT_MS        50U
#define MCP2518FD_MODE_TIMEOUT_MS       20U
#define MCP2518FD_OSC_TIMEOUT_MS        20U

#define MCP2518FD_NBTCFG_500K_40MHZ     ((4UL << 24) | (12UL << 16) | (1UL << 8) | 1UL)
#define MCP2518FD_DBTCFG_CLASSIC_SAFE   0x00000000UL

static SPI_HandleTypeDef *g_mcp2518fd_hspi;
static volatile uint8_t g_mcp2518fd_interrupt_pending;
static uint8_t g_mcp2518fd_ready;

static HAL_StatusTypeDef MCP2518FD_Reset(void);
static HAL_StatusTypeDef MCP2518FD_Read(uint16_t address, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef MCP2518FD_Write(uint16_t address, const uint8_t *data, uint16_t length);
static HAL_StatusTypeDef MCP2518FD_WriteRegister(uint16_t address, uint32_t value);
static HAL_StatusTypeDef MCP2518FD_ModifyRegister(uint16_t address, uint32_t clear_mask, uint32_t set_mask);
static HAL_StatusTypeDef MCP2518FD_SetMode(uint32_t mode);
static HAL_StatusTypeDef MCP2518FD_WaitOscillatorReady(void);
static HAL_StatusTypeDef MCP2518FD_ConfigRamAndFilters(void);
static void MCP2518FD_Select(void);
static void MCP2518FD_Deselect(void);
static uint16_t MCP2518FD_CommandHeader(uint8_t command, uint16_t address);
static void MCP2518FD_StoreLe32(uint8_t *data, uint32_t value);
static uint32_t MCP2518FD_LoadLe32(const uint8_t *data);
static uint8_t MCP2518FD_DlcToBytes(uint8_t dlc);
static uint8_t MCP2518FD_LengthToDlc(uint8_t length);
static uint32_t MCP2518FD_BuildIdWord(const MCP2518FD_CanFrame *frame);
static void MCP2518FD_ParseRxObject(const uint8_t *object, MCP2518FD_CanFrame *frame);

HAL_StatusTypeDef MCP2518FD_Init(SPI_HandleTypeDef *hspi)
{
  uint32_t value;

  if (hspi == NULL)
  {
    return HAL_ERROR;
  }

  g_mcp2518fd_hspi = hspi;
  g_mcp2518fd_ready = 0U;
  g_mcp2518fd_interrupt_pending = 0U;
  MCP2518FD_Deselect();
  HAL_Delay(2U);

  if (MCP2518FD_Reset() != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(2U);

  if (MCP2518FD_SetMode(MCP2518FD_MODE_CONFIGURATION) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(MCP2518FD_REG_OSC, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WaitOscillatorReady() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(MCP2518FD_REG_IOCON, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_ECCCON, MCP2518FD_ECCCON_ECCEN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1NBTCFG, MCP2518FD_NBTCFG_500K_40MHZ) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1DBTCFG, MCP2518FD_DBTCFG_CLASSIC_SAFE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1TDC, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ReadRegister(MCP2518FD_REG_C1CON, &value) != HAL_OK)
  {
    return HAL_ERROR;
  }
  value &= ~MCP2518FD_C1CON_REQOP_MASK;
  value |= (MCP2518FD_MODE_CONFIGURATION << MCP2518FD_C1CON_REQOP_POS) | MCP2518FD_C1CON_BRSDIS;
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1CON, value) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ConfigRamAndFilters() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1INT, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_SetMode(MCP2518FD_MODE_NORMAL_FD) != HAL_OK)
  {
    return HAL_ERROR;
  }

  g_mcp2518fd_ready = 1U;
  return HAL_OK;
}

uint8_t MCP2518FD_IsReady(void)
{
  return g_mcp2518fd_ready;
}

void MCP2518FD_OnInterrupt(void)
{
  g_mcp2518fd_interrupt_pending = 1U;
}

uint8_t MCP2518FD_HasPendingInterrupt(void)
{
  return g_mcp2518fd_interrupt_pending;
}

HAL_StatusTypeDef MCP2518FD_Receive(MCP2518FD_CanFrame *frame, uint8_t *received)
{
  uint32_t status;
  uint32_t user_address;
  uint8_t object[16];

  if ((frame == NULL) || (received == NULL) || (g_mcp2518fd_ready == 0U))
  {
    return HAL_ERROR;
  }

  *received = 0U;
  if (MCP2518FD_ReadRegister(MCP2518FD_FIFO_STA(MCP2518FD_RX_FIFO), &status) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((status & MCP2518FD_FIFO_TFNRFNIF) == 0U)
  {
    g_mcp2518fd_interrupt_pending = 0U;
    return HAL_OK;
  }

  if (MCP2518FD_ReadRegister(MCP2518FD_FIFO_UA(MCP2518FD_RX_FIFO), &user_address) != HAL_OK)
  {
    return HAL_ERROR;
  }
  user_address &= 0x0FFFU;

  if (((user_address & 0x3U) != 0U) ||
      (user_address < 0x400U) ||
      (user_address > 0xBFCU))
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_Read((uint16_t)user_address, object, sizeof(object)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  MCP2518FD_ParseRxObject(object, frame);

  if (MCP2518FD_WriteRegister(MCP2518FD_FIFO_CON(MCP2518FD_RX_FIFO),
                              MCP2518FD_RX_FIFO_CONFIG | MCP2518FD_FIFO_UINC) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *received = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef MCP2518FD_Transmit(const MCP2518FD_CanFrame *frame)
{
  uint32_t status;
  uint32_t user_address;
  uint8_t object[16];
  uint8_t i;

  if ((frame == NULL) || (g_mcp2518fd_ready == 0U) || (frame->dlc > 8U))
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_ReadRegister(MCP2518FD_FIFO_STA(MCP2518FD_TX_FIFO), &status) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if ((status & MCP2518FD_FIFO_TFNRFNIF) == 0U)
  {
    return HAL_BUSY;
  }

  if (MCP2518FD_ReadRegister(MCP2518FD_FIFO_UA(MCP2518FD_TX_FIFO), &user_address) != HAL_OK)
  {
    return HAL_ERROR;
  }
  user_address &= 0x0FFFU;

  if (((user_address & 0x3U) != 0U) ||
      (user_address < 0x400U) ||
      (user_address > 0xBFCU))
  {
    return HAL_ERROR;
  }

  memset(object, 0, sizeof(object));
  MCP2518FD_StoreLe32(&object[0], MCP2518FD_BuildIdWord(frame));
  MCP2518FD_StoreLe32(&object[4],
                      (uint32_t)MCP2518FD_LengthToDlc(frame->dlc) |
                      ((frame->id_type == MCP2518FD_ID_EXTENDED) ? (1UL << 4) : 0UL));
  for (i = 0U; i < frame->dlc; ++i)
  {
    object[8U + i] = frame->data[i];
  }

  if (MCP2518FD_Write((uint16_t)user_address, object, sizeof(object)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2518FD_WriteRegister(MCP2518FD_FIFO_CON(MCP2518FD_TX_FIFO),
                                 MCP2518FD_TX_FIFO_CONFIG | MCP2518FD_FIFO_UINC | MCP2518FD_FIFO_TXREQ);
}

HAL_StatusTypeDef MCP2518FD_ReadRegister(uint16_t address, uint32_t *value)
{
  uint8_t data[4];

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_Read(address, data, sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *value = MCP2518FD_LoadLe32(data);
  return HAL_OK;
}

static HAL_StatusTypeDef MCP2518FD_Reset(void)
{
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_RESET, 0x000U);
  uint8_t tx[2];
  HAL_StatusTypeDef status;

  tx[0] = (uint8_t)(header >> 8);
  tx[1] = (uint8_t)(header & 0xFFU);

  MCP2518FD_Select();
  status = HAL_SPI_Transmit(g_mcp2518fd_hspi, tx, sizeof(tx), MCP2518FD_SPI_TIMEOUT_MS);
  MCP2518FD_Deselect();

  return status;
}

static HAL_StatusTypeDef MCP2518FD_Read(uint16_t address, uint8_t *data, uint16_t length)
{
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_READ, address);
  uint8_t tx[2];
  HAL_StatusTypeDef status;

  if ((g_mcp2518fd_hspi == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(header >> 8);
  tx[1] = (uint8_t)(header & 0xFFU);

  MCP2518FD_Select();
  status = HAL_SPI_Transmit(g_mcp2518fd_hspi, tx, sizeof(tx), MCP2518FD_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(g_mcp2518fd_hspi, data, length, MCP2518FD_SPI_TIMEOUT_MS);
  }
  MCP2518FD_Deselect();

  return status;
}

static HAL_StatusTypeDef MCP2518FD_Write(uint16_t address, const uint8_t *data, uint16_t length)
{
  uint16_t header = MCP2518FD_CommandHeader(MCP2518FD_CMD_WRITE, address);
  uint8_t tx[2];
  HAL_StatusTypeDef status;

  if ((g_mcp2518fd_hspi == NULL) || (data == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(header >> 8);
  tx[1] = (uint8_t)(header & 0xFFU);

  MCP2518FD_Select();
  status = HAL_SPI_Transmit(g_mcp2518fd_hspi, tx, sizeof(tx), MCP2518FD_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(g_mcp2518fd_hspi, (uint8_t *)data, length, MCP2518FD_SPI_TIMEOUT_MS);
  }
  MCP2518FD_Deselect();

  return status;
}

static HAL_StatusTypeDef MCP2518FD_WriteRegister(uint16_t address, uint32_t value)
{
  uint8_t data[4];

  MCP2518FD_StoreLe32(data, value);
  return MCP2518FD_Write(address, data, sizeof(data));
}

static HAL_StatusTypeDef MCP2518FD_ModifyRegister(uint16_t address, uint32_t clear_mask, uint32_t set_mask)
{
  uint32_t value;

  if (MCP2518FD_ReadRegister(address, &value) != HAL_OK)
  {
    return HAL_ERROR;
  }
  value &= ~clear_mask;
  value |= set_mask;

  return MCP2518FD_WriteRegister(address, value);
}

static HAL_StatusTypeDef MCP2518FD_SetMode(uint32_t mode)
{
  uint32_t value;
  uint32_t start = HAL_GetTick();

  if (MCP2518FD_ModifyRegister(MCP2518FD_REG_C1CON,
                               MCP2518FD_C1CON_REQOP_MASK,
                               mode << MCP2518FD_C1CON_REQOP_POS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  do
  {
    if (MCP2518FD_ReadRegister(MCP2518FD_REG_C1CON, &value) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (((value & MCP2518FD_C1CON_OPMOD_MASK) >> MCP2518FD_C1CON_OPMOD_POS) == mode)
    {
      return HAL_OK;
    }
  } while ((uint32_t)(HAL_GetTick() - start) < MCP2518FD_MODE_TIMEOUT_MS);

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef MCP2518FD_WaitOscillatorReady(void)
{
  uint32_t value;
  uint32_t start = HAL_GetTick();

  do
  {
    if (MCP2518FD_ReadRegister(MCP2518FD_REG_OSC, &value) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((value & MCP2518FD_OSC_READY_MASK) == MCP2518FD_OSC_READY_MASK)
    {
      return HAL_OK;
    }
  } while ((uint32_t)(HAL_GetTick() - start) < MCP2518FD_OSC_TIMEOUT_MS);

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef MCP2518FD_ConfigRamAndFilters(void)
{
  if (MCP2518FD_WriteRegister(MCP2518FD_FIFO_CON(MCP2518FD_RX_FIFO), MCP2518FD_RX_FIFO_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_FIFO_CON(MCP2518FD_TX_FIFO), MCP2518FD_TX_FIFO_CONFIG) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1FLTCON0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1FLTOBJ0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (MCP2518FD_WriteRegister(MCP2518FD_REG_C1MASK0, 0x00000000UL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MCP2518FD_WriteRegister(MCP2518FD_REG_C1FLTCON0, MCP2518FD_FILTER0_TO_FIFO1);
}

static void MCP2518FD_Select(void)
{
  HAL_GPIO_WritePin(MCP2518_CS_GPIO_Port, MCP2518_CS_Pin, GPIO_PIN_RESET);
}

static void MCP2518FD_Deselect(void)
{
  HAL_GPIO_WritePin(MCP2518_CS_GPIO_Port, MCP2518_CS_Pin, GPIO_PIN_SET);
}

static uint16_t MCP2518FD_CommandHeader(uint8_t command, uint16_t address)
{
  return (uint16_t)((((uint16_t)command) << 12) | (address & 0x0FFFU));
}

static void MCP2518FD_StoreLe32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint32_t MCP2518FD_LoadLe32(const uint8_t *data)
{
  return ((uint32_t)data[3] << 24) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[1] << 8) |
         (uint32_t)data[0];
}

static uint8_t MCP2518FD_DlcToBytes(uint8_t dlc)
{
  static const uint8_t dlc_to_bytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
                                         8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

  if (dlc < (sizeof(dlc_to_bytes) / sizeof(dlc_to_bytes[0])))
  {
    return dlc_to_bytes[dlc];
  }

  return 0U;
}

static uint8_t MCP2518FD_LengthToDlc(uint8_t length)
{
  if (length >= 8U)
  {
    return 8U;
  }

  return length;
}

static uint32_t MCP2518FD_BuildIdWord(const MCP2518FD_CanFrame *frame)
{
  if (frame->id_type == MCP2518FD_ID_EXTENDED)
  {
    return ((frame->id & 0x3FFFFUL) << 11) |
           ((frame->id >> 18) & 0x7FFUL);
  }

  return frame->id & 0x7FFUL;
}

static void MCP2518FD_ParseRxObject(const uint8_t *object, MCP2518FD_CanFrame *frame)
{
  uint32_t id_word = MCP2518FD_LoadLe32(&object[0]);
  uint32_t control_word = MCP2518FD_LoadLe32(&object[4]);
  uint8_t bytes = MCP2518FD_DlcToBytes((uint8_t)(control_word & 0x0FU));
  uint8_t i;

  if ((control_word & (1UL << 4)) != 0UL)
  {
    frame->id_type = MCP2518FD_ID_EXTENDED;
    frame->id = (((id_word & 0x7FFUL) << 18) | ((id_word >> 11) & 0x3FFFFUL));
  }
  else
  {
    frame->id_type = MCP2518FD_ID_STANDARD;
    frame->id = id_word & 0x7FFUL;
  }

  if (bytes > 8U)
  {
    bytes = 8U;
  }
  frame->dlc = bytes;
  memset(frame->data, 0, sizeof(frame->data));
  for (i = 0U; i < bytes; ++i)
  {
    frame->data[i] = object[8U + i];
  }
}
