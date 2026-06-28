#ifndef __APP_UTILS_H__
#define __APP_UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint16_t App_ReadBe16(const uint8_t *data);
uint16_t App_ReadLe16(const uint8_t *data);
int16_t App_ReadLe16Signed(const uint8_t *data);
uint32_t App_ReadLe32(const uint8_t *data);
int32_t App_ReadBe32Signed(const uint8_t *data);
int32_t App_ReadLe32Signed(const uint8_t *data);
uint8_t App_IsFresh(uint32_t now, uint32_t updated_ms);
uint8_t App_FdcanDlcToBytes(uint32_t dlc);
uint32_t App_FdcanDlcFromBytes(uint8_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UTILS_H__ */
