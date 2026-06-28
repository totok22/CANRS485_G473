#include "app_fault.h"

#include <stdint.h>

#include "app_config.h"
#include "app_utils.h"

uint32_t App_FaultCompute(const AppTelemetryState *state, uint32_t now)
{
  uint32_t faults = 0U;

  if (App_IsFresh(now, state->alarm_updated_ms) != 0U)
  {
    faults |= state->alarm_fault_code;
  }

  if ((App_IsFresh(now, state->pack_summary_updated_ms) != 0U) && (state->imd_signal != 0U))
  {
    faults |= APP_IMD_FAULT_BIT;
  }

  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error != 0U))
  {
    faults |= APP_HALL_FAULT_BIT;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    if (state->hall_fault_active != 0U)
    {
      faults |= APP_HALL_FAULT_BIT;
    }
    if (state->imd_fault_active != 0U)
    {
      faults |= APP_IMD_FAULT_BIT;
    }
    faults |= (uint32_t)state->can2_error_rom_low16;
  }

  return faults;
}

