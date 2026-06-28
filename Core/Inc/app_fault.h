#ifndef __APP_FAULT_H__
#define __APP_FAULT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_state.h"

uint32_t App_FaultCompute(const AppTelemetryState *state, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FAULT_H__ */
