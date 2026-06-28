#include "app_telemetry.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_fault.h"
#include "app_utils.h"
#include "main.h"

#include "fsae_telemetry.pb.h"
#include "pb_encode.h"

_Static_assert(APP_MODULE_COUNT == (sizeof(((fsae_TelemetryFrame *)0)->modules) / sizeof(((fsae_TelemetryFrame *)0)->modules[0])),
               "APP_MODULE_COUNT must match fsae_TelemetryFrame.modules capacity");
_Static_assert(APP_CELLS_PER_MODULE == 23U, "BatteryModule protobuf layout has 23 voltage fields");
_Static_assert(APP_TEMPS_PER_MODULE == 8U, "BatteryModule protobuf layout has 8 temperature fields");
_Static_assert(APP_CAN1_VOLTAGE_FRAMES_PER_MODULE < 8U, "module voltage validity mask must fit in uint8_t");

static AppTelemetryState g_tx_snapshot;
static fsae_TelemetryFrame g_tx_frame;
static uint8_t g_pb_buffer[fsae_TelemetryFrame_size];
static const fsae_TelemetryFrame g_tx_frame_prototype = fsae_TelemetryFrame_init_zero;
static const fsae_BatteryModule g_battery_module_prototype = fsae_BatteryModule_init_zero;
static const fsae_MotorState g_motor_state_prototype = fsae_MotorState_init_zero;
static uint32_t g_frame_counter;

static uint8_t App_IvtStateIsUsable(uint8_t state);
static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now);
static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now);
static uint8_t App_GetEnergyMeterSource(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now);
static fsae_DrivingMode App_GetDrivingMode(const AppTelemetryState *state, uint32_t now);
static fsae_AlarmSeverity App_MapBatteryAlarmSeverity(uint8_t alarm_level);
static void App_AddAlarm(fsae_TelemetryFrame *frame, uint32_t alarm_id,
                         fsae_AlarmSeverity severity, const char *message);
static void App_BuildAlarms(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp, uint8_t max_temp_valid);
static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no);
static uint8_t App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                                  int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no);
static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);
static uint8_t App_BuildMotorStates(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame);

void App_Telemetry_Init(void)
{
  g_frame_counter = 0U;
}

uint32_t App_Telemetry_FrameCount(void)
{
  return g_frame_counter;
}

static uint8_t App_IvtStateIsUsable(uint8_t state)
{
  return ((state & APP_IVT_RESULT_ERROR_MASK) == 0U) ? 1U : 0U;
}

static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now)
{
  uint32_t i;
  uint32_t total_mv = 0U;

  if (App_IsFresh(now, state->fs_voltage_updated_ms) != 0U)
  {
    return (float)state->fs_voltage_mv / 1000.0f;
  }

  if ((App_IsFresh(now, state->ivt_voltage_u1_updated_ms) != 0U) &&
      (App_IvtStateIsUsable(state->ivt_voltage_u1_state) != 0U))
  {
    return (float)state->ivt_voltage_u1_mv / 1000.0f;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return (float)state->pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_pack_voltage_deci_v / 10.0f;
  }

  if (App_IsFresh(now, state->cell_voltage_sum_updated_ms) != 0U)
  {
    return (float)state->cell_voltage_sum_deci_v / 10.0f;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != APP_MODULE_VOLTAGE_VALID_MASK)
    {
      return 0.0f;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    total_mv += state->cell_voltage_mv[i];
  }

  return (float)total_mv / 1000.0f;
}

static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->fs_current_updated_ms) != 0U)
  {
    return (float)state->fs_current_ma / 1000.0f;
  }

  if ((App_IsFresh(now, state->ivt_current_updated_ms) != 0U) &&
      (App_IvtStateIsUsable(state->ivt_current_state) != 0U))
  {
    return (float)state->ivt_current_ma / 1000.0f;
  }

  if (App_IsFresh(now, state->hall_updated_ms) != 0U && (state->hall_error == 0U))
  {
    return (float)state->hall_current_ma / 1000.0f;
  }

  if (App_IsFresh(now, state->charger_fb_updated_ms) != 0U)
  {
    return (float)state->charger_fb_current_deci_a / 10.0f;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    if (App_IsFresh(now, state->status_updated_ms) != 0U && (state->charge_state != 0U))
    {
      return (float)state->summary_current_raw / 10.0f;
    }

    return ((float)state->summary_current_raw - 10000.0f) / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return ((float)state->can2_pack_current_raw - 10000.0f) / 10.0f;
  }

  return 0.0f;
}

static uint8_t App_GetEnergyMeterSource(const AppTelemetryState *state, uint32_t now)
{
  if ((App_IsFresh(now, state->fs_current_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_voltage_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_power_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_energy_updated_ms) != 0U) ||
      (App_IsFresh(now, state->fs_status_updated_ms) != 0U))
  {
    return APP_ENERGY_METER_SOURCE_FS;
  }

  if ((App_IsFresh(now, state->ivt_current_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_voltage_u1_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_power_updated_ms) != 0U) ||
      (App_IsFresh(now, state->ivt_energy_updated_ms) != 0U))
  {
    return APP_ENERGY_METER_SOURCE_IVT;
  }

  return APP_ENERGY_METER_SOURCE_UNKNOWN;
}

static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_soc;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return state->can2_soc;
  }

  return 0U;
}

static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->status_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->pack_summary_updated_ms) != 0U)
  {
    return state->battery_state;
  }

  if (App_IsFresh(now, state->can2_diag_updated_ms) != 0U)
  {
    return state->can2_battery_state;
  }

  return 0U;
}

static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now)
{
  return (App_GetVcuStatus(state, now) == 5U) ? 1U : 0U;
}

static fsae_DrivingMode App_GetDrivingMode(const AppTelemetryState *state, uint32_t now)
{
  if (App_IsFresh(now, state->vehicle_mode_updated_ms) == 0U)
  {
    return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
  }

  switch (state->vehicle_mode_flag)
  {
    case -1:
      return fsae_DrivingMode_DRIVING_MODE_DEFAULT;
    case 0:
      return fsae_DrivingMode_DRIVING_MODE_STRAIGHT;
    case 1:
      return fsae_DrivingMode_DRIVING_MODE_AUTOCROSS;
    case 2:
      return fsae_DrivingMode_DRIVING_MODE_SKIDPAD;
    case 3:
      return fsae_DrivingMode_DRIVING_MODE_ENDURANCE;
    default:
      break;
  }

  return fsae_DrivingMode_DRIVING_MODE_UNSPECIFIED;
}

static fsae_AlarmSeverity App_MapBatteryAlarmSeverity(uint8_t alarm_level)
{
  switch (alarm_level)
  {
    case 1U:
      return fsae_AlarmSeverity_ALARM_SEVERITY_WARNING;
    case 2U:
      return fsae_AlarmSeverity_ALARM_SEVERITY_ERROR;
    default:
      break;
  }

  return fsae_AlarmSeverity_ALARM_SEVERITY_FATAL;
}

static void App_AddAlarm(fsae_TelemetryFrame *frame, uint32_t alarm_id,
                         fsae_AlarmSeverity severity, const char *message)
{
  fsae_Alarm *alarm;

  if (frame->alarms_count >= (sizeof(frame->alarms) / sizeof(frame->alarms[0])))
  {
    return;
  }

  alarm = &frame->alarms[frame->alarms_count];
  alarm->alarm_id = alarm_id;
  alarm->severity = severity;
  (void)snprintf(alarm->message, sizeof(alarm->message), "%s", message);
  frame->alarms_count++;
}

static void App_BuildAlarms(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  char message[64];

  if (App_IsFresh(now, state->pack_summary_updated_ms) == 0U)
  {
    return;
  }

  if (state->battery_alarm_level != 0U)
  {
    (void)snprintf(message, sizeof(message), "BMS summary alarm level %u", (unsigned int)state->battery_alarm_level);
    App_AddAlarm(frame, APP_CAN1_PACK_SUMMARY_ID,
                 App_MapBatteryAlarmSeverity(state->battery_alarm_level), message);
  }

  if (state->imd_signal != 0U)
  {
    (void)snprintf(message, sizeof(message), "IMD signal raw %u", (unsigned int)state->imd_signal);
    App_AddAlarm(frame, APP_IMD_FAULT_BIT,
                 fsae_AlarmSeverity_ALARM_SEVERITY_ERROR, message);
  }
}

static float App_GetBatteryTempMax(const AppTelemetryState *state, uint32_t now, int32_t max_temp, uint8_t max_temp_valid)
{
  if (max_temp_valid != 0U)
  {
    return (float)max_temp / 10.0f;
  }

  if (App_IsFresh(now, state->can2_power_updated_ms) != 0U)
  {
    return (float)state->can2_max_temp_deci_c / 10.0f;
  }

  return 0.0f;
}

static void App_GetCellExtrema(const AppTelemetryState *state, uint32_t now,
                               uint32_t *max_mv, uint32_t *min_mv, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t i;
  uint16_t local_max = 0U;
  uint16_t local_min = 0xFFFFU;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;

  if (App_IsFresh(now, state->cell_extrema_updated_ms) != 0U)
  {
    *max_mv = state->max_cell_voltage_mv;
    *min_mv = state->min_cell_voltage_mv;
    *max_no = (uint32_t)state->max_cell_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_cell_index_zero_based + 1U;
    return;
  }

  for (i = 0U; i < APP_MODULE_COUNT; ++i)
  {
    if (App_IsFresh(now, state->module_voltage_updated_ms[i]) == 0U ||
        state->module_voltage_valid[i] != APP_MODULE_VOLTAGE_VALID_MASK)
    {
      *max_mv = 0U;
      *min_mv = 0U;
      *max_no = 0U;
      *min_no = 0U;
      return;
    }
  }

  for (i = 0U; i < APP_TOTAL_CELL_COUNT; ++i)
  {
    if (state->cell_voltage_mv[i] >= local_max)
    {
      local_max = state->cell_voltage_mv[i];
      local_max_no = i + 1U;
    }
    if (state->cell_voltage_mv[i] <= local_min)
    {
      local_min = state->cell_voltage_mv[i];
      local_min_no = i + 1U;
    }
  }

  *max_mv = local_max;
  *min_mv = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
}

static uint8_t App_GetTempExtrema(const AppTelemetryState *state, uint32_t now,
                                  int32_t *max_temp, int32_t *min_temp, uint32_t *max_no, uint32_t *min_no)
{
  uint32_t module_idx;
  uint32_t temp_idx;
  int16_t local_max = INT16_MIN;
  int16_t local_min = INT16_MAX;
  uint32_t local_max_no = 0U;
  uint32_t local_min_no = 0U;
  uint8_t has_valid = 0U;

  if (App_IsFresh(now, state->temp_extrema_updated_ms) != 0U)
  {
    *max_temp = state->max_temp_deci_c;
    *min_temp = state->min_temp_deci_c;
    *max_no = (uint32_t)state->max_temp_index_zero_based + 1U;
    *min_no = (uint32_t)state->min_temp_index_zero_based + 1U;
    return 1U;
  }

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t temp_base = module_idx * APP_TEMPS_PER_MODULE;

    if (App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_temp_valid[module_idx] == 0U)
    {
      *max_temp = 0;
      *min_temp = 0;
      *max_no = 0U;
      *min_no = 0U;
      return 0U;
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) == 0U)
      {
        continue;
      }

      if (state->cell_temp_deci_c[temp_base + temp_idx] >= local_max)
      {
        local_max = state->cell_temp_deci_c[temp_base + temp_idx];
        local_max_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }
      if (state->cell_temp_deci_c[temp_base + temp_idx] <= local_min)
      {
        local_min = state->cell_temp_deci_c[temp_base + temp_idx];
        local_min_no = (module_idx * APP_TEMPS_PER_MODULE) + temp_idx + 1U;
      }

      has_valid = 1U;
    }
  }

  if (has_valid == 0U)
  {
    *max_temp = 0;
    *min_temp = 0;
    *max_no = 0U;
    *min_no = 0U;
    return 0U;
  }

  *max_temp = local_max;
  *min_temp = local_min;
  *max_no = local_max_no;
  *min_no = local_min_no;
  return 1U;
}

static uint8_t App_BuildModules(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  uint32_t module_idx;

  frame->modules_count = 0U;

  for (module_idx = 0U; module_idx < APP_MODULE_COUNT; ++module_idx)
  {
    uint32_t cell_base;
    uint32_t temp_base;
    uint32_t cell_idx;
    uint32_t temp_idx;
    uint32_t *voltage_field;
    int32_t *temp_field;

    if (App_IsFresh(now, state->module_voltage_updated_ms[module_idx]) == 0U ||
        App_IsFresh(now, state->module_temp_updated_ms[module_idx]) == 0U ||
        state->module_voltage_valid[module_idx] != APP_MODULE_VOLTAGE_VALID_MASK ||
        state->module_temp_valid[module_idx] == 0U)
    {
      continue;
    }

    frame->modules[frame->modules_count] = g_battery_module_prototype;
    frame->modules[frame->modules_count].module_id = module_idx + 1U;

    voltage_field = &frame->modules[frame->modules_count].v01;
    temp_field = &frame->modules[frame->modules_count].t1;
    cell_base = module_idx * APP_CELLS_PER_MODULE;
    temp_base = module_idx * APP_TEMPS_PER_MODULE;

    for (cell_idx = 0U; cell_idx < APP_CELLS_PER_MODULE; ++cell_idx)
    {
      voltage_field[cell_idx] = state->cell_voltage_mv[cell_base + cell_idx];
    }

    for (temp_idx = 0U; temp_idx < APP_TEMPS_PER_MODULE; ++temp_idx)
    {
      if ((state->module_temp_valid[module_idx] & (uint8_t)(1U << temp_idx)) != 0U)
      {
        temp_field[temp_idx] = state->cell_temp_deci_c[temp_base + temp_idx];
      }
    }

    frame->modules_count++;
  }

  return (frame->modules_count > 0U) ? 1U : 0U;
}

static uint8_t App_BuildMotorStates(const AppTelemetryState *state, uint32_t now, fsae_TelemetryFrame *frame)
{
  static const uint8_t motor_index[APP_MOTOR_COUNT] = {
    APP_MOTOR_FL,
    APP_MOTOR_FR,
    APP_MOTOR_RL,
    APP_MOTOR_RR
  };
  static const fsae_MotorPosition motor_position[APP_MOTOR_COUNT] = {
    fsae_MotorPosition_MOTOR_POSITION_FRONT_LEFT,
    fsae_MotorPosition_MOTOR_POSITION_FRONT_RIGHT,
    fsae_MotorPosition_MOTOR_POSITION_REAR_LEFT,
    fsae_MotorPosition_MOTOR_POSITION_REAR_RIGHT
  };
  uint8_t torque_fresh = App_IsFresh(now, state->motor_torque_updated_ms);
  uint8_t diag_fresh = App_IsFresh(now, state->motor_diag_updated_ms);
  uint8_t rpm_fresh = App_IsFresh(now, state->motor_rpm_updated_ms);
  uint8_t motor_temp_fresh = App_IsFresh(now, state->motor_temp_updated_ms);
  uint8_t inverter_temp_fresh = App_IsFresh(now, state->motor_inverter_temp_updated_ms);
  uint8_t igbt_temp_fresh = App_IsFresh(now, state->motor_igbt_temp_updated_ms);
  uint8_t logic_state_fresh = App_IsFresh(now, state->motor_logic_state_updated_ms);
  uint32_t out_idx;

  if ((torque_fresh == 0U) && (diag_fresh == 0U) && (rpm_fresh == 0U) &&
      (motor_temp_fresh == 0U) && (inverter_temp_fresh == 0U) && (igbt_temp_fresh == 0U) &&
      (logic_state_fresh == 0U))
  {
    frame->vehicle_state.motors_count = 0U;
    return 0U;
  }

  frame->vehicle_state.motors_count = APP_MOTOR_COUNT;
  for (out_idx = 0U; out_idx < APP_MOTOR_COUNT; ++out_idx)
  {
    uint8_t idx = motor_index[out_idx];
    fsae_MotorState *motor = &frame->vehicle_state.motors[out_idx];

    *motor = g_motor_state_prototype;
    motor->position = motor_position[out_idx];
    if (rpm_fresh != 0U)
    {
      motor->rpm = state->motor_rpm[idx];
    }
    if (torque_fresh != 0U)
    {
      motor->torque_nm = state->motor_torque_0p1pct[idx];
    }
    if (motor_temp_fresh != 0U)
    {
      motor->motor_temp_dc = state->motor_temp_deci_c[idx];
    }
    if (inverter_temp_fresh != 0U)
    {
      motor->inverter_temp_dc = state->motor_inverter_temp_deci_c[idx];
    }
    if (igbt_temp_fresh != 0U)
    {
      motor->igbt_temp_dc = state->motor_igbt_temp_deci_c[idx];
    }
    if (diag_fresh != 0U)
    {
      motor->diagnostic_number = state->motor_diagnostic_number[idx];
      motor->motor_error = (int32_t)state->motor_diagnostic_number[idx];
    }
    if (logic_state_fresh != 0U)
    {
      motor->has_logic_state = true;
      motor->logic_state = state->motor_logic_state[idx];
    }
  }

  return 1U;
}

HAL_StatusTypeDef App_Telemetry_Send(volatile const AppTelemetryState *state, uint32_t now, uint8_t include_modules, AppTelemetryTransmitFn transmit)
{
  AppTelemetryState *snapshot = &g_tx_snapshot;
  fsae_TelemetryFrame *frame = &g_tx_frame;
  pb_ostream_t stream;
  uint32_t max_mv;
  uint32_t min_mv;
  uint32_t max_mv_no;
  uint32_t min_mv_no;
  int32_t max_temp;
  int32_t min_temp;
  uint32_t max_temp_no;
  uint32_t min_temp_no;
  uint8_t temp_extrema_valid;
  float hv_voltage;
  float hv_current;
  float battery_temp_max;
  fsae_DrivingMode driving_mode;

  __disable_irq();
  *snapshot = *((const AppTelemetryState *)state);
  __enable_irq();

  *frame = g_tx_frame_prototype;
  frame->timestamp_ms = now;
  frame->frame_id = ++g_frame_counter;
  hv_voltage = App_GetHvVoltage(snapshot, now);
  hv_current = App_GetHvCurrent(snapshot, now);
  frame->hv_voltage = hv_voltage;
  frame->hv_current = hv_current;
  if (App_IsFresh(now, snapshot->can2_datalogger_updated_ms) != 0U)
  {
    frame->apps_position = (float)snapshot->can2_apps_open_deci_pct / 10.0f;
    frame->brake_pressure = (float)snapshot->can2_oil_pressure_milli_kpa / 1000.0f;
    frame->steering_angle = (float)snapshot->can2_steering_angle_deci_deg / 10.0f;
  }
  frame->battery_soc = App_GetBatterySoc(snapshot, now);
  frame->ready_to_drive = App_GetReadyToDrive(snapshot, now);
  frame->vcu_status = App_GetVcuStatus(snapshot, now);
  frame->fault_code = App_FaultCompute(snapshot, now);
  frame->battery_fault_code = frame->fault_code;
  driving_mode = App_GetDrivingMode(snapshot, now);

  App_GetCellExtrema(snapshot, now, &max_mv, &min_mv, &max_mv_no, &min_mv_no);
  temp_extrema_valid = App_GetTempExtrema(snapshot, now, &max_temp, &min_temp, &max_temp_no, &min_temp_no);

  frame->max_cell_voltage = max_mv;
  frame->min_cell_voltage = min_mv;
  frame->max_cell_voltage_no = max_mv_no;
  frame->min_cell_voltage_no = min_mv_no;
  frame->max_temp = max_temp;
  frame->min_temp = min_temp;
  frame->max_temp_no = max_temp_no;
  frame->min_temp_no = min_temp_no;
  battery_temp_max = App_GetBatteryTempMax(snapshot, now, max_temp, temp_extrema_valid);
  frame->battery_temp_max = battery_temp_max;

  frame->has_header = true;
  frame->header.timestamp_ms = now;
  frame->header.seq = g_frame_counter;
  frame->header.source_id = 1U;
  frame->has_fast_telemetry = true;
  frame->fast_telemetry.hv_voltage_dv = (int32_t)(hv_voltage * 10.0f);
  frame->fast_telemetry.hv_current_ma = (int32_t)(hv_current * 1000.0f);
  frame->fast_telemetry.battery_temp_max_dc = (int32_t)(battery_temp_max * 10.0f);
  frame->fast_telemetry.driving_mode = driving_mode;
  if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
  {
    frame->fast_telemetry.speed_kmh = snapshot->can2_gps_speed_kmh;
  }
  frame->has_vehicle_state = true;
  frame->vehicle_state.driving_mode = driving_mode;
  if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
  {
    frame->vehicle_state.speed_kmh = snapshot->can2_gps_speed_kmh;
  }
  if (App_IsFresh(now, snapshot->can2_datalogger_updated_ms) != 0U)
  {
    frame->vehicle_state.throttle_position = (uint32_t)((snapshot->can2_apps_open_deci_pct + 5U) / 10U);
  }
  frame->vehicle_state.vcu_status = (frame->vcu_status == 0U) ?
                                    fsae_VcuStatus_VCU_STATUS_UNSPECIFIED :
                                    fsae_VcuStatus_VCU_STATUS_HV_ENABLED;
  App_BuildAlarms(snapshot, now, frame);
  (void)App_BuildMotorStates(snapshot, now, frame);

  if ((App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_accel_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_gyro_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->can2_yaw_updated_ms) != 0U))
  {
    frame->has_motion = true;
    if (App_IsFresh(now, snapshot->can2_gps_speed_updated_ms) != 0U)
    {
      frame->motion.gps_speed_kmh = snapshot->can2_gps_speed_kmh;
    }
    if (App_IsFresh(now, snapshot->can2_accel_updated_ms) != 0U)
    {
      frame->motion.accel_x_g = (float)snapshot->can2_accel_x_raw * 0.00048828125f;
      frame->motion.accel_y_g = (float)snapshot->can2_accel_y_raw * 0.00048828125f;
      frame->motion.accel_z_g = (float)snapshot->can2_accel_z_raw * 0.00048828125f;
    }
    if (App_IsFresh(now, snapshot->can2_gyro_updated_ms) != 0U)
    {
      frame->motion.yaw_rate_dps = (float)snapshot->can2_yaw_rate_raw * 0.0610352f;
    }
    if (App_IsFresh(now, snapshot->can2_yaw_updated_ms) != 0U)
    {
      frame->motion.yaw_deg = (float)snapshot->can2_yaw_raw * 0.005493f;
    }
  }

  if (App_IsFresh(now, snapshot->motor_rpm_updated_ms) != 0U)
  {
    frame->motor_rpm = snapshot->motor_rpm[APP_MOTOR_RL];
  }
  if (App_IsFresh(now, snapshot->motor_temp_updated_ms) != 0U)
  {
    frame->motor_temp = (float)snapshot->motor_temp_deci_c[APP_MOTOR_RL] / 10.0f;
  }
  if (App_IsFresh(now, snapshot->motor_inverter_temp_updated_ms) != 0U)
  {
    frame->inverter_temp = (float)snapshot->motor_inverter_temp_deci_c[APP_MOTOR_RL] / 10.0f;
  }

  if ((App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U) ||
      (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U))
  {
    const uint8_t ivt_current_fresh = App_IsFresh(now, snapshot->ivt_current_updated_ms);
    const uint8_t ivt_voltage_u1_fresh = App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms);

    frame->has_ivt_telemetry = true;
    if (ivt_current_fresh != 0U)
    {
      frame->ivt_telemetry.current_ma = snapshot->ivt_current_ma;
      frame->ivt_telemetry.current_state = snapshot->ivt_current_state;
    }
    if (ivt_voltage_u1_fresh != 0U)
    {
      frame->ivt_telemetry.voltage_u1_mv = snapshot->ivt_voltage_u1_mv;
      frame->ivt_telemetry.voltage_u1_state = snapshot->ivt_voltage_u1_state;
    }
    if (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U)
    {
      frame->ivt_telemetry.power_w = snapshot->ivt_power_w;
    }
    else if ((ivt_current_fresh != 0U) && (ivt_voltage_u1_fresh != 0U))
    {
      frame->ivt_telemetry.power_w =
          (int32_t)(((int64_t)snapshot->ivt_voltage_u1_mv * (int64_t)snapshot->ivt_current_ma) / 1000000LL);
    }
    if (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U)
    {
      frame->ivt_telemetry.energy_wh = snapshot->ivt_energy_wh;
      frame->ivt_telemetry.energy_state = snapshot->ivt_energy_state;
    }
  }

  switch (App_GetEnergyMeterSource(snapshot, now))
  {
    case APP_ENERGY_METER_SOURCE_FS:
      frame->has_energy_meter = true;
      frame->energy_meter.source = APP_ENERGY_METER_SOURCE_FS;
      if (App_IsFresh(now, snapshot->fs_current_updated_ms) != 0U)
      {
        frame->energy_meter.current_ma = snapshot->fs_current_ma;
      }
      if (App_IsFresh(now, snapshot->fs_voltage_updated_ms) != 0U)
      {
        frame->energy_meter.voltage_mv = snapshot->fs_voltage_mv;
      }
      if (App_IsFresh(now, snapshot->fs_energy_updated_ms) != 0U)
      {
        frame->energy_meter.energy_wh = snapshot->fs_energy_wh;
      }
      if (App_IsFresh(now, snapshot->fs_power_updated_ms) != 0U)
      {
        frame->energy_meter.power_w = snapshot->fs_power_w;
      }
      if (App_IsFresh(now, snapshot->fs_status_updated_ms) != 0U)
      {
        frame->energy_meter.status = snapshot->fs_status;
        frame->energy_meter.msg_counter = snapshot->fs_msg_counter;
      }
      if ((App_IsFresh(now, snapshot->fs_power_updated_ms) == 0U) &&
          (App_IsFresh(now, snapshot->fs_current_updated_ms) != 0U) &&
          (App_IsFresh(now, snapshot->fs_voltage_updated_ms) != 0U))
      {
        frame->energy_meter.power_w =
            (int32_t)(((int64_t)snapshot->fs_voltage_mv * (int64_t)snapshot->fs_current_ma) / 1000000LL);
      }
      break;

    case APP_ENERGY_METER_SOURCE_IVT:
      frame->has_energy_meter = true;
      frame->energy_meter.source = APP_ENERGY_METER_SOURCE_IVT;
      if (App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U)
      {
        frame->energy_meter.current_ma = snapshot->ivt_current_ma;
        frame->energy_meter.status = snapshot->ivt_current_state;
      }
      if (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U)
      {
        frame->energy_meter.voltage_mv = snapshot->ivt_voltage_u1_mv;
      }
      if (App_IsFresh(now, snapshot->ivt_energy_updated_ms) != 0U)
      {
        frame->energy_meter.energy_wh = snapshot->ivt_energy_wh;
      }
      if (App_IsFresh(now, snapshot->ivt_power_updated_ms) != 0U)
      {
        frame->energy_meter.power_w = snapshot->ivt_power_w;
      }
      else if ((App_IsFresh(now, snapshot->ivt_current_updated_ms) != 0U) &&
          (App_IsFresh(now, snapshot->ivt_voltage_u1_updated_ms) != 0U))
      {
        frame->energy_meter.power_w =
            (int32_t)(((int64_t)snapshot->ivt_voltage_u1_mv * (int64_t)snapshot->ivt_current_ma) / 1000000LL);
      }
      break;

    default:
      break;
  }

  if (include_modules != 0U)
  {
    (void)App_BuildModules(snapshot, now, frame);
  }

  stream = pb_ostream_from_buffer(g_pb_buffer, sizeof(g_pb_buffer));
  if (!pb_encode(&stream, fsae_TelemetryFrame_fields, frame))
  {
    return HAL_ERROR;
  }

  return transmit(g_pb_buffer, (uint16_t)stream.bytes_written);
}
