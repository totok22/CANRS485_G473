#ifndef __APP_COMMAND_PARSER_H__
#define __APP_COMMAND_PARSER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t App_CommandTryParse(const uint8_t *data, uint8_t length, uint8_t *command_value);

#ifdef __cplusplus
}
#endif

#endif /* __APP_COMMAND_PARSER_H__ */
