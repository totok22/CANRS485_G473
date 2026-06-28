#include "app_utils.h"

#include "app_config.h"
#include "main.h"

uint16_t App_ReadBe16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

uint16_t App_ReadLe16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

int16_t App_ReadLe16Signed(const uint8_t *data)
{
  return (int16_t)App_ReadLe16(data);
}

uint32_t App_ReadLe32(const uint8_t *data)
{
  return ((uint32_t)data[3] << 24) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[1] << 8) |
         (uint32_t)data[0];
}

int32_t App_ReadBe32Signed(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];
  return (int32_t)value;
}

int32_t App_ReadLe32Signed(const uint8_t *data)
{
  return (int32_t)App_ReadLe32(data);
}

uint8_t App_IsFresh(uint32_t now, uint32_t updated_ms)
{
  return (updated_ms != 0U) && (((uint32_t)(now - updated_ms)) <= APP_SNAPSHOT_TIMEOUT_MS);
}

uint8_t App_FdcanDlcToBytes(uint32_t dlc)
{
  static const uint8_t dlc_to_bytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
                                         8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

  if (dlc < (sizeof(dlc_to_bytes) / sizeof(dlc_to_bytes[0])))
  {
    return dlc_to_bytes[dlc];
  }

  return 0U;
}

uint32_t App_FdcanDlcFromBytes(uint8_t bytes)
{
  if (bytes >= 8U)
  {
    return FDCAN_DLC_BYTES_8;
  }

  return (uint32_t)bytes;
}
