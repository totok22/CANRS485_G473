#include "app_can_decode.h"

#include <stdint.h>
#include <string.h>

#include "app_can_io.h"
#include "app_config.h"
#include "app_utils.h"
#include "main.h"

#define APP_CAN1_BASE_VOLTAGE_ID        0x180050F3UL
#define APP_CAN1_BASE_TEMP_ID           0x184050F3UL
#define APP_CAN1_CELL_EXTREMA_ID        0x186150F4UL
#define APP_CAN1_TEMP_EXTREMA_ID        0x186250F4UL
#define APP_CAN1_STATUS_ID              0x186350F4UL
#define APP_CAN1_ALARM_ID               0x187650F4UL
#define APP_CAN1_CELL_SUM_ID            0x186750F4UL
#define APP_CAN1_IMD_DIAG_ID            0x186850F4UL
#define APP_CAN1_TOOL_FAULT_RESET_ID    0x188350F5UL
#define APP_CAN1_TOOL_ADC_CAL_ID        0x18A050F5UL
#define APP_CAN1_TOOL_CURRENT_DIR_ID    0x18A150F5UL
#define APP_CAN1_TOOL_RTC_SET_ID        0x18A350F5UL
#define APP_CAN1_HALL_ID                0x03C0U

#define APP_CAN2_CHARGER_FB_ID          0x18FF50E5UL
#define APP_CAN2_GPS_SPEED_ID           0x0301U
#define APP_CAN2_DATALOGGER_ID          0x0305U
#define APP_CAN2_POWER_STATUS_ID        0x0401U
#define APP_CAN2_DIAG_STATUS_ID         0x0402U
#define APP_CAN2_FS_DATALOGGER_STATUS_ID 0x0430U
#define APP_IMU_RAW_ID                  0x0050U
#define APP_IMU_TIME_ID                 0x0060U
#define APP_IMU_ACCEL_ID                0x0061U
#define APP_IMU_GYRO_ID                 0x0062U
#define APP_IMU_ROLL_ID                 0x0063U
#define APP_IMU_PITCH_ID                0x0064U
#define APP_IMU_YAW_ID                  0x0065U
#define APP_IMU_MAGNETIC_ID             0x0066U
#define APP_CAN2_DEBUG2_ID              0x0502U
#define APP_CAN2_DEBUG3_ID              0x0503U
#define APP_CAN2_DEBUG4_ID              0x0504U
#define APP_CAN2_DEBUG5_ID              0x0505U
#define APP_CAN2_DEBUG6_ID              0x0506U
#define APP_CAN2_DEBUG7_ID              0x0507U
#define APP_CAN2_DEBUG8_ID              0x0508U
#define APP_CAN2_DEBUG9_ID              0x0509U
#define APP_CAN2_IVT_CURRENT_ID         0x0521U
#define APP_CAN2_IVT_U1_ID              0x0522U
#define APP_CAN2_IVT_POWER_ID           0x0526U
#define APP_CAN2_IVT_WH_ID              0x0528U
#define APP_IVT_MUX_CURRENT             0x00U
#define APP_IVT_MUX_U1                  0x01U
#define APP_IVT_MUX_POWER               0x05U
#define APP_IVT_MUX_WH                  0x07U
#define APP_IVT_BYTE_ORDER_UNKNOWN      0U
#define APP_IVT_BYTE_ORDER_BE           1U
#define APP_IVT_BYTE_ORDER_LE           2U
#define APP_IVT_CURRENT_ABS_MAX_MA      600000L
#define APP_IVT_VOLTAGE_U1_ABS_MAX_MV   1000000L
#define APP_IVT_ENERGY_ABS_MAX_WH       100000000L

#define APP_CAN2_ENERGY_METER_MODE_IVT  1U
#define APP_CAN2_ENERGY_METER_MODE_FS   2U
#define APP_CAN2_ENERGY_METER_MODE_AUTO 3U
#define APP_CAN2_ENERGY_METER_MODE      APP_CAN2_ENERGY_METER_MODE_AUTO

#define APP_FS_STATUS_READY_BIT         (1U << 0)
#define APP_FS_STATUS_LOGGING_BIT       (1U << 1)
#define APP_FS_STATUS_TRIG_VOLTAGE_BIT  (1U << 2)
#define APP_FS_STATUS_TRIG_CURRENT_BIT  (1U << 3)

#define APP_IMU_RAW_HEADER              0x55U
#define APP_IMU_RAW_SUBTYPE_TIME        0x50U
#define APP_IMU_RAW_SUBTYPE_ACCEL       0x51U
#define APP_IMU_RAW_SUBTYPE_GYRO        0x52U
#define APP_IMU_RAW_SUBTYPE_ANGLE       0x53U
#define APP_IMU_RAW_SUBTYPE_MAGNETIC    0x54U
#define APP_IMU_ANGLE_SUBCMD_ROLL       0x01U
#define APP_IMU_ANGLE_SUBCMD_PITCH      0x02U
#define APP_IMU_ANGLE_SUBCMD_YAW        0x03U

typedef struct
{
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} AppCan2TxFrame;

static volatile AppTelemetryState *g_decode_state;
static uint8_t g_decode_events;

#define g_app_state (*g_decode_state)

#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_FS
static uint8_t App_IvtValueIsPlausible(uint32_t std_id, int32_t value);
static uint8_t App_IvtDecodeResult(uint32_t std_id, const uint8_t *data, int32_t *value);
#endif
static void App_SetProtocol(AppProtocol protocol);
static void App_RequestFaultRefresh(void);
static void App_ProcessCan1Rx(const AppCanRxHeader *header, const uint8_t *data);
static void App_ProcessCan2Rx(const AppCanRxHeader *header, const uint8_t *data);
static uint8_t App_ProcessCan1Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
static uint8_t App_ProcessCan2Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
static uint8_t App_ProcessCan2Std(uint32_t std_id, const uint8_t *data, uint8_t dlc);
static uint8_t App_ProcessCan2DataLogger(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2GpsSpeed(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessImuRaw(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_DecodeImuRawForward(const uint8_t *data, uint8_t dlc, AppCan2TxFrame *frame);
static uint8_t App_ProcessCan2ImuAccel(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2ImuGyro(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2ImuYaw(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessVoltageFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessTempFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessPackSummary(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCellVoltageSum(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessImdDiag(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCellExtrema(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessTempExtrema(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessStatusFrame(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessAlarmFrame(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessChargerFeedback(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2PowerStatus(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2DiagStatus(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessFsDataloggerStatus(const uint8_t *data, uint8_t dlc, uint32_t now);
#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_IVT
static uint8_t App_ProcessFsDataloggerResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now);
#endif
#if APP_CAN2_ENERGY_METER_MODE == APP_CAN2_ENERGY_METER_MODE_AUTO
static uint8_t App_ProcessEnergyMeterAutoResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now);
#endif
#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_FS
static uint8_t App_ProcessIvtResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now);
#endif
static uint8_t App_ProcessCan2MotorFrame(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now);
static uint8_t App_ProcessCan2Debug9(const uint8_t *data, uint8_t dlc, uint32_t now);
static uint32_t App_BuildBatteryFaultCode(const uint8_t *data);

uint8_t App_CanDecode_Process(volatile AppTelemetryState *state,
                              AppFdcanBus bus,
                              const AppCanRxHeader *header,
                              const uint8_t *data)
{
  if ((state == NULL) || (header == NULL) || (data == NULL))
  {
    return APP_CAN_DECODE_EVENT_NONE;
  }

  g_decode_state = state;
  g_decode_events = APP_CAN_DECODE_EVENT_NONE;

  if (bus == APP_FDCAN_BUS_CAN1)
  {
    App_ProcessCan1Rx(header, data);
  }
  else if (bus == APP_FDCAN_BUS_CANB)
  {
    App_ProcessCan2Rx(header, data);
  }

  return g_decode_events;
}

#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_FS
static uint8_t App_IvtValueIsPlausible(uint32_t std_id, int32_t value)
{
  int64_t abs_value = value;

  if (abs_value < 0)
  {
    abs_value = -abs_value;
  }

  switch (std_id)
  {
    case APP_CAN2_IVT_CURRENT_ID:
      return (abs_value <= APP_IVT_CURRENT_ABS_MAX_MA) ? 1U : 0U;

    case APP_CAN2_IVT_U1_ID:
      return (abs_value <= APP_IVT_VOLTAGE_U1_ABS_MAX_MV) ? 1U : 0U;

    case APP_CAN2_IVT_POWER_ID:
    case APP_CAN2_IVT_WH_ID:
      return (abs_value <= APP_IVT_ENERGY_ABS_MAX_WH) ? 1U : 0U;

    default:
      break;
  }

  return 0U;
}

static uint8_t App_IvtDecodeResult(uint32_t std_id, const uint8_t *data, int32_t *value)
{
  int32_t be_value;
  int32_t le_value;
  uint8_t be_plausible;
  uint8_t le_plausible;

  if (g_app_state.ivt_byte_order == APP_IVT_BYTE_ORDER_BE)
  {
    *value = App_ReadBe32Signed(data);
    if (App_IvtValueIsPlausible(std_id, *value) != 0U)
    {
      return 1U;
    }
    g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_UNKNOWN;
  }

  if (g_app_state.ivt_byte_order == APP_IVT_BYTE_ORDER_LE)
  {
    *value = App_ReadLe32Signed(data);
    if (App_IvtValueIsPlausible(std_id, *value) != 0U)
    {
      return 1U;
    }
    g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_UNKNOWN;
  }

  be_value = App_ReadBe32Signed(data);
  le_value = App_ReadLe32Signed(data);
  be_plausible = App_IvtValueIsPlausible(std_id, be_value);
  le_plausible = App_IvtValueIsPlausible(std_id, le_value);

  if ((be_plausible != 0U) && (le_plausible == 0U))
  {
    g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_BE;
    *value = be_value;
    return 1U;
  }

  if ((le_plausible != 0U) && (be_plausible == 0U))
  {
    g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_LE;
    *value = le_value;
    return 1U;
  }

  if ((be_plausible != 0U) && (le_plausible != 0U) && (be_value == le_value))
  {
    *value = be_value;
    return 1U;
  }

  return 0U;
}
#endif

static void App_SetProtocol(AppProtocol protocol)
{
  if ((protocol != APP_PROTOCOL_UNKNOWN) && (g_app_state.protocol != protocol))
  {
    g_app_state.protocol = protocol;
  }
}


static void App_RequestFaultRefresh(void)
{
  g_decode_events |= APP_CAN_DECODE_EVENT_FAULT_REFRESH;
}

static void App_ProcessCan1Rx(const AppCanRxHeader *header, const uint8_t *data)
{
  if (header->IDE == APP_CAN_ID_EXT)
  {
    if (App_ProcessCan1Ext(header->ExtId, data, header->DLC) != 0U)
    {
      g_app_state.can1_seen = 1U;
    }
    return;
  }

  if ((header->IDE == APP_CAN_ID_STD) && (header->StdId == APP_CAN1_HALL_ID) && (header->DLC >= 8U))
  {
    App_SetProtocol(APP_PROTOCOL_MODERN);
    g_app_state.hall_current_ma = App_ReadBe32Signed(data);
    g_app_state.hall_error = (uint8_t)(data[4] & 0x01U);
    g_app_state.hall_error_code = (uint8_t)((data[4] >> 1) & 0x7FU);
    g_app_state.hall_sensor_name = App_ReadBe16(&data[5]);
    g_app_state.hall_sw_version = data[7];
    g_app_state.hall_updated_ms = HAL_GetTick();
    g_app_state.can1_seen = 1U;
    App_RequestFaultRefresh();
  }
  else if ((header->IDE == APP_CAN_ID_STD) && (header->StdId == APP_IMU_RAW_ID))
  {
    if (App_ProcessImuRaw(data, header->DLC, HAL_GetTick()) != 0U)
    {
      g_app_state.can1_seen = 1U;
    }
  }
}

static void App_ProcessCan2Rx(const AppCanRxHeader *header, const uint8_t *data)
{
  if (header->IDE == APP_CAN_ID_EXT)
  {
    if (App_ProcessCan2Ext(header->ExtId, data, header->DLC) != 0U)
    {
      g_app_state.can2_seen = 1U;
    }
  }
  else if (header->IDE == APP_CAN_ID_STD)
  {
    if (App_ProcessCan2Std(header->StdId, data, header->DLC) != 0U)
    {
      g_app_state.can2_seen = 1U;
    }
  }
}

static uint8_t App_ProcessCan1Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
  uint32_t now = HAL_GetTick();

  if ((ext_id >= APP_CAN1_BASE_VOLTAGE_ID) &&
      (ext_id < (APP_CAN1_BASE_VOLTAGE_ID + (36UL << 16))) &&
      (((ext_id - APP_CAN1_BASE_VOLTAGE_ID) & 0xFFFFU) == 0U))
  {
    return App_ProcessVoltageFrame(ext_id, data, dlc, now);
  }

  if ((ext_id >= APP_CAN1_BASE_TEMP_ID) &&
      (ext_id < (APP_CAN1_BASE_TEMP_ID + (APP_MODULE_COUNT << 16))) &&
      (((ext_id - APP_CAN1_BASE_TEMP_ID) & 0xFFFFU) == 0U))
  {
    return App_ProcessTempFrame(ext_id, data, dlc, now);
  }

  switch (ext_id)
  {
    case APP_CAN1_PACK_SUMMARY_ID:
      return App_ProcessPackSummary(data, dlc, now);

    case APP_CAN1_CELL_SUM_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      return App_ProcessCellVoltageSum(data, dlc, now);

    case APP_CAN1_IMD_DIAG_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      return App_ProcessImdDiag(data, dlc, now);

    case APP_CAN1_CELL_EXTREMA_ID:
      return App_ProcessCellExtrema(data, dlc, now);

    case APP_CAN1_TEMP_EXTREMA_ID:
      return App_ProcessTempExtrema(data, dlc, now);

    case APP_CAN1_STATUS_ID:
      return App_ProcessStatusFrame(data, dlc, now);

    case APP_CAN1_ALARM_ID:
      return App_ProcessAlarmFrame(data, dlc, now);

    case APP_CAN1_TOOL_FAULT_RESET_ID:
    case APP_CAN1_TOOL_ADC_CAL_ID:
    case APP_CAN1_TOOL_CURRENT_DIR_ID:
    case APP_CAN1_TOOL_RTC_SET_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      return 0U;

    default:
      break;
  }

  return 0U;
}

static uint8_t App_ProcessCan2Ext(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
  if (ext_id == APP_CAN2_CHARGER_FB_ID)
  {
    return App_ProcessChargerFeedback(data, dlc, HAL_GetTick());
  }

  return 0U;
}

static uint8_t App_ProcessCan2Std(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
  switch (std_id)
  {
    case APP_CAN2_GPS_SPEED_ID:
      return App_ProcessCan2GpsSpeed(data, dlc, HAL_GetTick());

    case APP_CAN2_DATALOGGER_ID:
      return App_ProcessCan2DataLogger(data, dlc, HAL_GetTick());

    case APP_IMU_RAW_ID:
      return App_ProcessImuRaw(data, dlc, HAL_GetTick());

    case APP_IMU_ACCEL_ID:
      return App_ProcessCan2ImuAccel(data, dlc, HAL_GetTick());

    case APP_IMU_GYRO_ID:
      return App_ProcessCan2ImuGyro(data, dlc, HAL_GetTick());

    case APP_IMU_YAW_ID:
      return App_ProcessCan2ImuYaw(data, dlc, HAL_GetTick());

    case APP_CAN2_IVT_CURRENT_ID:
    case APP_CAN2_IVT_U1_ID:
    case APP_CAN2_IVT_POWER_ID:
    case APP_CAN2_IVT_WH_ID:
#if APP_CAN2_ENERGY_METER_MODE == APP_CAN2_ENERGY_METER_MODE_FS
      return App_ProcessFsDataloggerResult(std_id, data, dlc, HAL_GetTick());
#elif APP_CAN2_ENERGY_METER_MODE == APP_CAN2_ENERGY_METER_MODE_IVT
      return App_ProcessIvtResult(std_id, data, dlc, HAL_GetTick());
#else
      return App_ProcessEnergyMeterAutoResult(std_id, data, dlc, HAL_GetTick());
#endif

    case APP_CAN2_FS_DATALOGGER_STATUS_ID:
      return App_ProcessFsDataloggerStatus(data, dlc, HAL_GetTick());

    case APP_CAN2_DEBUG2_ID:
    case APP_CAN2_DEBUG3_ID:
    case APP_CAN2_DEBUG4_ID:
    case APP_CAN2_DEBUG5_ID:
    case APP_CAN2_DEBUG6_ID:
    case APP_CAN2_DEBUG7_ID:
    case APP_CAN2_DEBUG8_ID:
      return App_ProcessCan2MotorFrame(std_id, data, dlc, HAL_GetTick());

    case APP_CAN2_DEBUG9_ID:
      return App_ProcessCan2Debug9(data, dlc, HAL_GetTick());

    case APP_CAN2_POWER_STATUS_ID:
      return App_ProcessCan2PowerStatus(data, dlc, HAL_GetTick());

    case APP_CAN2_DIAG_STATUS_ID:
      App_SetProtocol(APP_PROTOCOL_MODERN);
      return App_ProcessCan2DiagStatus(data, dlc, HAL_GetTick());

    default:
      break;
  }

  return 0U;
}

static uint8_t App_ProcessCan2DataLogger(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return 0U;
  }

  g_app_state.can2_steering_angle_deci_deg = App_ReadLe16Signed(&data[0]);
  g_app_state.can2_apps_open_deci_pct = App_ReadLe16(&data[2]);
  g_app_state.can2_oil_pressure_milli_kpa = App_ReadLe16(&data[4]);
  g_app_state.can2_datalogger_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCan2GpsSpeed(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 2U)
  {
    return 0U;
  }

  g_app_state.can2_gps_speed_kmh = App_ReadLe16(&data[0]);
  g_app_state.can2_gps_speed_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessImuRaw(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  AppCan2TxFrame forward_frame;
  uint8_t queued;

  if (App_DecodeImuRawForward(data, dlc, &forward_frame) == 0U)
  {
    return 0U;
  }

  queued = App_CanIo_QueueCanBStd(forward_frame.std_id, forward_frame.data, forward_frame.dlc);

  switch (forward_frame.std_id)
  {
    case APP_IMU_TIME_ID:
    case APP_IMU_MAGNETIC_ID:
    case APP_IMU_ROLL_ID:
    case APP_IMU_PITCH_ID:
      /* MotionTelemetry has no time, magnetic, roll, or pitch fields. */
      return queued;

    case APP_IMU_ACCEL_ID:
    {
      uint8_t processed = App_ProcessCan2ImuAccel(forward_frame.data, forward_frame.dlc, now);
      return (uint8_t)(((queued != 0U) && (processed != 0U)) ? 1U : 0U);
    }

    case APP_IMU_GYRO_ID:
    {
      uint8_t processed = App_ProcessCan2ImuGyro(forward_frame.data, forward_frame.dlc, now);
      return (uint8_t)(((queued != 0U) && (processed != 0U)) ? 1U : 0U);
    }

    case APP_IMU_YAW_ID:
    {
      uint8_t processed = App_ProcessCan2ImuYaw(forward_frame.data, forward_frame.dlc, now);
      return (uint8_t)(((queued != 0U) && (processed != 0U)) ? 1U : 0U);
    }

    default:
      break;
  }

  return 0U;
}

static uint8_t App_DecodeImuRawForward(const uint8_t *data, uint8_t dlc, AppCan2TxFrame *frame)
{
  const uint8_t *forward_data;
  uint8_t i;

  if ((dlc < 2U) || (data[0] != APP_IMU_RAW_HEADER))
  {
    return 0U;
  }

  switch (data[1])
  {
    case APP_IMU_RAW_SUBTYPE_TIME:
      if (dlc < 8U)
      {
        return 0U;
      }
      frame->std_id = APP_IMU_TIME_ID;
      forward_data = &data[2];
      frame->dlc = 6U;
      break;

    case APP_IMU_RAW_SUBTYPE_ACCEL:
      if (dlc < 8U)
      {
        return 0U;
      }
      frame->std_id = APP_IMU_ACCEL_ID;
      forward_data = &data[2];
      frame->dlc = 6U;
      break;

    case APP_IMU_RAW_SUBTYPE_GYRO:
      if (dlc < 8U)
      {
        return 0U;
      }
      frame->std_id = APP_IMU_GYRO_ID;
      forward_data = &data[2];
      frame->dlc = 6U;
      break;

    case APP_IMU_RAW_SUBTYPE_ANGLE:
      if (dlc < 5U)
      {
        return 0U;
      }
      if (data[2] == APP_IMU_ANGLE_SUBCMD_ROLL)
      {
        frame->std_id = APP_IMU_ROLL_ID;
      }
      else if (data[2] == APP_IMU_ANGLE_SUBCMD_PITCH)
      {
        frame->std_id = APP_IMU_PITCH_ID;
      }
      else if (data[2] == APP_IMU_ANGLE_SUBCMD_YAW)
      {
        frame->std_id = APP_IMU_YAW_ID;
      }
      else
      {
        return 0U;
      }
      forward_data = &data[3];
      frame->dlc = 2U;
      break;

    case APP_IMU_RAW_SUBTYPE_MAGNETIC:
      if (dlc < 8U)
      {
        return 0U;
      }
      frame->std_id = APP_IMU_MAGNETIC_ID;
      forward_data = &data[2];
      frame->dlc = 6U;
      break;

    default:
      return 0U;
  }

  for (i = 0U; i < frame->dlc; ++i)
  {
    frame->data[i] = forward_data[i];
  }
  return 1U;
}

static uint8_t App_ProcessCan2ImuAccel(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return 0U;
  }

  g_app_state.can2_accel_x_raw = App_ReadLe16Signed(&data[0]);
  g_app_state.can2_accel_y_raw = App_ReadLe16Signed(&data[2]);
  g_app_state.can2_accel_z_raw = App_ReadLe16Signed(&data[4]);
  g_app_state.can2_accel_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCan2ImuGyro(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return 0U;
  }

  g_app_state.can2_yaw_rate_raw = App_ReadLe16Signed(&data[4]);
  g_app_state.can2_gyro_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCan2ImuYaw(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 2U)
  {
    return 0U;
  }

  g_app_state.can2_yaw_raw = App_ReadLe16Signed(&data[0]);
  g_app_state.can2_yaw_updated_ms = now;
  return 1U;
}

#if APP_CAN2_ENERGY_METER_MODE == APP_CAN2_ENERGY_METER_MODE_AUTO
static uint8_t App_ProcessEnergyMeterAutoResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  int32_t be_value;
  int32_t le_value;
  uint8_t be_plausible;
  uint8_t le_plausible;

  if (dlc < 6U)
  {
    return 0U;
  }

  if (App_IsFresh(now, g_app_state.fs_status_updated_ms) != 0U)
  {
    g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
    return App_ProcessFsDataloggerResult(std_id, data, dlc, now);
  }

  if (g_app_state.energy_meter_auto_source == APP_ENERGY_METER_SOURCE_FS)
  {
    be_value = App_ReadBe32Signed(&data[2]);
    if (App_IvtValueIsPlausible(std_id, be_value) != 0U)
    {
      return App_ProcessFsDataloggerResult(std_id, data, dlc, now);
    }
    g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_UNKNOWN;
  }

  if (g_app_state.energy_meter_auto_source == APP_ENERGY_METER_SOURCE_IVT)
  {
    le_value = App_ReadLe32Signed(&data[2]);
    if (App_IvtValueIsPlausible(std_id, le_value) != 0U)
    {
      g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_LE;
      return App_ProcessIvtResult(std_id, data, dlc, now);
    }
    g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_UNKNOWN;
  }

  be_value = App_ReadBe32Signed(&data[2]);
  le_value = App_ReadLe32Signed(&data[2]);
  be_plausible = App_IvtValueIsPlausible(std_id, be_value);
  le_plausible = App_IvtValueIsPlausible(std_id, le_value);

  if ((be_plausible != 0U) && (le_plausible == 0U))
  {
    g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
    return App_ProcessFsDataloggerResult(std_id, data, dlc, now);
  }

  if ((le_plausible != 0U) && (be_plausible == 0U))
  {
    g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_IVT;
    g_app_state.ivt_byte_order = APP_IVT_BYTE_ORDER_LE;
    return App_ProcessIvtResult(std_id, data, dlc, now);
  }

  return 0U;
}
#endif

#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_FS
static uint8_t App_ProcessIvtResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint8_t mux;
  uint8_t state;
  int32_t value;

  if (dlc < 6U)
  {
    return 0U;
  }

  mux = data[0];
  state = (uint8_t)((data[1] >> 4) & 0x0FU);
  if (App_IvtDecodeResult(std_id, &data[2], &value) == 0U)
  {
    return 0U;
  }

  switch (std_id)
  {
    case APP_CAN2_IVT_CURRENT_ID:
      if (mux != APP_IVT_MUX_CURRENT)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_IVT;
      g_app_state.ivt_current_ma = value;
      g_app_state.ivt_current_state = state;
      g_app_state.ivt_current_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_U1_ID:
      if (mux != APP_IVT_MUX_U1)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_IVT;
      g_app_state.ivt_voltage_u1_mv = value;
      g_app_state.ivt_voltage_u1_state = state;
      g_app_state.ivt_voltage_u1_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_POWER_ID:
      if (mux != APP_IVT_MUX_POWER)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_IVT;
      g_app_state.ivt_power_w = value;
      g_app_state.ivt_power_state = state;
      g_app_state.ivt_power_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_WH_ID:
      if (mux != APP_IVT_MUX_WH)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_IVT;
      g_app_state.ivt_energy_wh = value;
      g_app_state.ivt_energy_state = state;
      g_app_state.ivt_energy_updated_ms = now;
      return 1U;

    default:
      break;
  }

  return 0U;
}
#endif

static uint8_t App_ProcessFsDataloggerStatus(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint8_t status_bits;

  if (dlc < 6U)
  {
    return 0U;
  }

  status_bits = data[1];
  g_app_state.fs_status = 0U;
  if ((status_bits & (1U << 0)) != 0U) g_app_state.fs_status |= APP_FS_STATUS_READY_BIT;
  if ((status_bits & (1U << 1)) != 0U) g_app_state.fs_status |= APP_FS_STATUS_LOGGING_BIT;
  if ((status_bits & (1U << 2)) != 0U) g_app_state.fs_status |= APP_FS_STATUS_TRIG_VOLTAGE_BIT;
  if ((status_bits & (1U << 3)) != 0U) g_app_state.fs_status |= APP_FS_STATUS_TRIG_CURRENT_BIT;

  g_app_state.fs_msg_counter = data[0];
  g_app_state.fs_voltage_mv = (int32_t)App_ReadBe16(&data[2]) * 16;
  g_app_state.fs_current_ma = (int32_t)App_ReadBe16(&data[4]) * 64;
  g_app_state.fs_status_updated_ms = now;
  g_app_state.fs_voltage_updated_ms = now;
  g_app_state.fs_current_updated_ms = now;
  g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
  return 1U;
}

#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_IVT
static uint8_t App_ProcessFsDataloggerResult(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint8_t mux;
  uint8_t state;
  int32_t value;

  if (dlc < 6U)
  {
    return 0U;
  }

  mux = data[0];
  state = (uint8_t)((data[1] >> 4) & 0x0FU);
  value = App_ReadBe32Signed(&data[2]);

  /* FS Datalogger result frames keep the IVT mux IDs but use big-endian values. */
  switch (std_id)
  {
    case APP_CAN2_IVT_CURRENT_ID:
      if (mux != APP_IVT_MUX_CURRENT)
      {
        return 0U;
      }
      /* FS current result unit is mA. */
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
      g_app_state.fs_current_ma = value;
      g_app_state.fs_status = state;
      g_app_state.fs_msg_counter = (uint8_t)(data[1] & 0x0FU);
      g_app_state.fs_status_updated_ms = now;
      g_app_state.fs_current_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_U1_ID:
      if (mux != APP_IVT_MUX_U1)
      {
        return 0U;
      }
      /* FS result voltage unit is mV; status frame 0x430 is raw * 16 mV. */
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
      g_app_state.fs_voltage_mv = value;
      g_app_state.fs_status = state;
      g_app_state.fs_msg_counter = (uint8_t)(data[1] & 0x0FU);
      g_app_state.fs_status_updated_ms = now;
      g_app_state.fs_voltage_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_POWER_ID:
      if (mux != APP_IVT_MUX_POWER)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
      g_app_state.fs_power_w = value;
      g_app_state.fs_status = state;
      g_app_state.fs_msg_counter = (uint8_t)(data[1] & 0x0FU);
      g_app_state.fs_status_updated_ms = now;
      g_app_state.fs_power_updated_ms = now;
      return 1U;

    case APP_CAN2_IVT_WH_ID:
      if (mux != APP_IVT_MUX_WH)
      {
        return 0U;
      }
      g_app_state.energy_meter_auto_source = APP_ENERGY_METER_SOURCE_FS;
      g_app_state.fs_energy_wh = value;
      g_app_state.fs_status = state;
      g_app_state.fs_msg_counter = (uint8_t)(data[1] & 0x0FU);
      g_app_state.fs_status_updated_ms = now;
      g_app_state.fs_energy_updated_ms = now;
      return 1U;

    default:
      break;
  }

  return 0U;
}
#endif

static uint8_t App_ProcessCan2MotorFrame(uint32_t std_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return 0U;
  }

  switch (std_id)
  {
    case APP_CAN2_DEBUG2_ID:
      g_app_state.motor_torque_0p1pct[APP_MOTOR_RL] = App_ReadLe16Signed(&data[0]);
      g_app_state.motor_torque_0p1pct[APP_MOTOR_RR] = App_ReadLe16Signed(&data[2]);
      g_app_state.motor_torque_0p1pct[APP_MOTOR_FL] = App_ReadLe16Signed(&data[4]);
      g_app_state.motor_torque_0p1pct[APP_MOTOR_FR] = App_ReadLe16Signed(&data[6]);
      g_app_state.motor_torque_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG3_ID:
      g_app_state.motor_diagnostic_number[APP_MOTOR_RL] = App_ReadLe32(&data[0]);
      g_app_state.motor_diagnostic_number[APP_MOTOR_RR] = App_ReadLe32(&data[4]);
      g_app_state.motor_diag_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG4_ID:
      g_app_state.motor_diagnostic_number[APP_MOTOR_FL] = App_ReadLe32(&data[0]);
      g_app_state.motor_diagnostic_number[APP_MOTOR_FR] = App_ReadLe32(&data[4]);
      g_app_state.motor_diag_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG5_ID:
      g_app_state.motor_rpm[APP_MOTOR_RL] = App_ReadLe16Signed(&data[0]);
      g_app_state.motor_rpm[APP_MOTOR_RR] = App_ReadLe16Signed(&data[2]);
      g_app_state.motor_rpm[APP_MOTOR_FL] = App_ReadLe16Signed(&data[4]);
      g_app_state.motor_rpm[APP_MOTOR_FR] = App_ReadLe16Signed(&data[6]);
      g_app_state.motor_rpm_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG6_ID:
      g_app_state.motor_temp_deci_c[APP_MOTOR_RL] = App_ReadLe16Signed(&data[0]);
      g_app_state.motor_temp_deci_c[APP_MOTOR_RR] = App_ReadLe16Signed(&data[2]);
      g_app_state.motor_temp_deci_c[APP_MOTOR_FL] = App_ReadLe16Signed(&data[4]);
      g_app_state.motor_temp_deci_c[APP_MOTOR_FR] = App_ReadLe16Signed(&data[6]);
      g_app_state.motor_temp_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG7_ID:
      g_app_state.motor_inverter_temp_deci_c[APP_MOTOR_RL] = App_ReadLe16Signed(&data[0]);
      g_app_state.motor_inverter_temp_deci_c[APP_MOTOR_RR] = App_ReadLe16Signed(&data[2]);
      g_app_state.motor_inverter_temp_deci_c[APP_MOTOR_FL] = App_ReadLe16Signed(&data[4]);
      g_app_state.motor_inverter_temp_deci_c[APP_MOTOR_FR] = App_ReadLe16Signed(&data[6]);
      g_app_state.motor_inverter_temp_updated_ms = now;
      return 1U;

    case APP_CAN2_DEBUG8_ID:
      g_app_state.motor_igbt_temp_deci_c[APP_MOTOR_RL] = App_ReadLe16Signed(&data[0]);
      g_app_state.motor_igbt_temp_deci_c[APP_MOTOR_RR] = App_ReadLe16Signed(&data[2]);
      g_app_state.motor_igbt_temp_deci_c[APP_MOTOR_FL] = App_ReadLe16Signed(&data[4]);
      g_app_state.motor_igbt_temp_deci_c[APP_MOTOR_FR] = App_ReadLe16Signed(&data[6]);
      g_app_state.motor_igbt_temp_updated_ms = now;
      return 1U;

    default:
      break;
  }

  return 0U;
}

static uint8_t App_ProcessCan2Debug9(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint8_t raw_mode;

  if (dlc < 1U)
  {
    return 0U;
  }

  raw_mode = (uint8_t)((data[0] >> 4) & 0x0FU);
  g_app_state.vehicle_mode_flag = (raw_mode >= 8U) ? (int8_t)(raw_mode | 0xF0U) : (int8_t)raw_mode;
  g_app_state.vehicle_mode_updated_ms = now;
  if (dlc >= 5U)
  {
    g_app_state.motor_logic_state[APP_MOTOR_RR] = (uint8_t)(data[3] & 0x0FU);
    g_app_state.motor_logic_state[APP_MOTOR_RL] = (uint8_t)((data[3] >> 4) & 0x0FU);
    g_app_state.motor_logic_state[APP_MOTOR_FR] = (uint8_t)(data[4] & 0x0FU);
    g_app_state.motor_logic_state[APP_MOTOR_FL] = (uint8_t)((data[4] >> 4) & 0x0FU);
    g_app_state.motor_logic_state_updated_ms = now;
  }
  return 1U;
}

static uint8_t App_ProcessVoltageFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint32_t frame_no = (ext_id - APP_CAN1_BASE_VOLTAGE_ID) >> 16;
  uint32_t module_idx;
  uint32_t frame_idx;
  uint32_t cell_offset;
  uint32_t cell_count;
  uint32_t base_idx;
  uint32_t src_idx;
  uint32_t i;

  if ((dlc < 8U) || (frame_no >= 36U))
  {
    return 0U;
  }

  module_idx = frame_no / 6U;
  frame_idx = frame_no % 6U;

  if (frame_idx == 0U)
  {
    cell_offset = 0U;
    cell_count = 3U;
    src_idx = 2U;
  }
  else
  {
    cell_offset = 3U + ((frame_idx - 1U) * 4U);
    cell_count = 4U;
    src_idx = 0U;
  }

  base_idx = (module_idx * APP_CELLS_PER_MODULE) + cell_offset;
  for (i = 0U; i < cell_count; ++i)
  {
    g_app_state.cell_voltage_mv[base_idx + i] = App_ReadLe16(&data[src_idx + (i * 2U)]);
  }

  g_app_state.module_voltage_valid[module_idx] |= (uint8_t)(1U << frame_idx);
  g_app_state.module_voltage_updated_ms[module_idx] = now;
  return 1U;
}

static uint8_t App_ProcessTempFrame(uint32_t ext_id, const uint8_t *data, uint8_t dlc, uint32_t now)
{
  uint32_t module_idx = (ext_id - APP_CAN1_BASE_TEMP_ID) >> 16;
  uint32_t base_idx;
  uint32_t i;
  uint8_t temp_count = APP_TEMPS_PER_MODULE;
  uint8_t src_idx = 0U;

  if ((dlc < 8U) || (module_idx >= APP_MODULE_COUNT))
  {
    return 0U;
  }

  base_idx = module_idx * APP_TEMPS_PER_MODULE;
  if ((g_app_state.protocol != APP_PROTOCOL_MODERN) &&
      ((module_idx == 2U) || (module_idx == 3U)))
  {
    temp_count = APP_TEMPS_PER_MODULE - 1U;
    src_idx = 1U;
  }

  g_app_state.module_temp_valid[module_idx] = 0U;
  for (i = 0U; i < temp_count; ++i)
  {
    g_app_state.cell_temp_deci_c[base_idx + i] = (int16_t)(((int16_t)data[src_idx + i] - 30) * 10);
    g_app_state.module_temp_valid[module_idx] |= (uint8_t)(1U << i);
  }

  for (; i < APP_TEMPS_PER_MODULE; ++i)
  {
    g_app_state.cell_temp_deci_c[base_idx + i] = 0;
  }

  g_app_state.module_temp_updated_ms[module_idx] = now;
  return 1U;
}

static uint8_t App_ProcessPackSummary(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 7U)
  {
    return 0U;
  }

  g_app_state.pack_voltage_deci_v = App_ReadBe16(&data[0]);
  g_app_state.summary_current_raw = App_ReadBe16(&data[2]);
  g_app_state.battery_soc = data[4];
  g_app_state.imd_signal = data[5];
  g_app_state.battery_state = (uint8_t)((data[6] >> 4) & 0x0FU);
  g_app_state.battery_alarm_level = (uint8_t)(data[6] & 0x0FU);
  g_app_state.pack_summary_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCellVoltageSum(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 2U)
  {
    return 0U;
  }

  g_app_state.cell_voltage_sum_deci_v = App_ReadBe16(&data[0]);
  g_app_state.cell_voltage_sum_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessImdDiag(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return 0U;
  }

  memcpy((void *)g_app_state.imd_diag_payload, data, 8U);
  g_app_state.imd_diag_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCellExtrema(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return 0U;
  }

  g_app_state.max_cell_voltage_mv = App_ReadBe16(&data[0]);
  g_app_state.min_cell_voltage_mv = App_ReadBe16(&data[2]);
  g_app_state.max_cell_index_zero_based = data[4];
  g_app_state.min_cell_index_zero_based = data[5];
  g_app_state.cell_extrema_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessTempExtrema(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 5U)
  {
    return 0U;
  }

  g_app_state.max_temp_deci_c = (int16_t)(((int16_t)data[0] - 30) * 10);
  g_app_state.min_temp_deci_c = (int16_t)(((int16_t)data[1] - 30) * 10);
  g_app_state.max_temp_index_zero_based = data[2];
  g_app_state.min_temp_index_zero_based = data[3];
  g_app_state.cooling_control = data[4];
  g_app_state.temp_extrema_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessStatusFrame(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return 0U;
  }

  g_app_state.pos_relay_state = (uint8_t)((data[0] >> 6) & 0x03U);
  g_app_state.neg_relay_state = (uint8_t)((data[0] >> 4) & 0x03U);
  g_app_state.pre_relay_state = (uint8_t)((data[0] >> 2) & 0x03U);
  g_app_state.charge_state = (uint8_t)((data[1] >> 4) & 0x0FU);
  g_app_state.charge_comm_state = (uint8_t)((data[1] >> 3) & 0x01U);
  g_app_state.charge_request_voltage_deci_v = App_ReadBe16(&data[2]);
  g_app_state.charge_request_current_deci_a = App_ReadBe16(&data[4]);
  g_app_state.precharge_voltage_deci_v = App_ReadBe16(&data[6]);
  g_app_state.status_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessAlarmFrame(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 6U)
  {
    return 0U;
  }

  g_app_state.alarm_fault_code = App_BuildBatteryFaultCode(data) & ~(APP_HALL_FAULT_BIT | APP_IMD_FAULT_BIT);
  g_app_state.slave_offline_mask = (uint8_t)((((uint8_t)(data[4] >> 1) & 0x03U) << 4) |
                                             (((uint8_t)(data[5] >> 1) & 0x0FU)));
  g_app_state.alarm_updated_ms = now;
  App_RequestFaultRefresh();
  return 1U;
}

static uint8_t App_ProcessChargerFeedback(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 5U)
  {
    return 0U;
  }

  g_app_state.charger_fb_voltage_deci_v = App_ReadBe16(&data[0]);
  g_app_state.charger_fb_current_deci_a = App_ReadBe16(&data[2]);
  g_app_state.charger_fb_state = data[4];
  g_app_state.charger_fb_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCan2PowerStatus(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return 0U;
  }

  g_app_state.can2_pack_voltage_deci_v = App_ReadLe16(&data[0]);
  g_app_state.can2_pack_power_raw = App_ReadLe16(&data[2]);
  g_app_state.can2_pack_current_raw = App_ReadLe16(&data[4]);
  g_app_state.can2_soc = data[6];
  g_app_state.can2_max_temp_deci_c = (int16_t)(((int16_t)data[7] - 30) * 10);
  g_app_state.can2_power_updated_ms = now;
  return 1U;
}

static uint8_t App_ProcessCan2DiagStatus(const uint8_t *data, uint8_t dlc, uint32_t now)
{
  if (dlc < 8U)
  {
    return 0U;
  }

  g_app_state.can2_battery_state = (uint8_t)((data[0] >> 4) & 0x0FU);
  g_app_state.can2_alarm_level = (uint8_t)(data[0] & 0x0FU);
  g_app_state.imd_fault_active = (uint8_t)((data[4] >> 2) & 0x01U);
  g_app_state.hall_fault_active = (uint8_t)((data[4] >> 4) & 0x01U);
  g_app_state.can2_error_rom_low16 = App_ReadLe16(&data[5]);
  g_app_state.slave_offline_mask = (uint8_t)(data[7] & 0x3FU);
  g_app_state.can2_diag_updated_ms = now;
  App_RequestFaultRefresh();
  return 1U;
}

static uint32_t App_BuildBatteryFaultCode(const uint8_t *data)
{
  uint32_t faults = 0U;

  if ((data[0] & 0xC0U) != 0U) faults |= (1UL << 0);
  if ((data[0] & 0x30U) != 0U) faults |= (1UL << 1);
  if ((data[0] & 0x0CU) != 0U) faults |= (1UL << 2);
  if ((data[0] & 0x03U) != 0U) faults |= (1UL << 3);
  if ((data[1] & 0x80U) != 0U) faults |= (1UL << 4);
  if ((data[1] & 0x40U) != 0U) faults |= (1UL << 5);
  if ((data[1] & 0x30U) != 0U) faults |= (1UL << 6);
  if ((data[1] & 0x0CU) != 0U) faults |= (1UL << 7);
  if ((data[1] & 0x03U) != 0U) faults |= (1UL << 8);
  if ((data[2] & 0xC0U) != 0U) faults |= (1UL << 9);
  if ((data[2] & 0x30U) != 0U) faults |= (1UL << 10);
  if ((data[2] & 0x0CU) != 0U) faults |= (1UL << 11);
  if (((data[2] & 0x03U) != 0U) || ((data[3] & 0x30U) != 0U)) faults |= (1UL << 12);
  if (((data[3] & 0xC0U) != 0U) || ((data[3] & 0x0CU) != 0U)) faults |= (1UL << 13);
  if ((data[4] & 0x10U) != 0U) faults |= (1UL << 14);
  if ((data[4] & 0x08U) != 0U) faults |= (1UL << 15);
  if (((data[4] & 0x06U) != 0U) || ((data[5] & 0x20U) != 0U)) faults |= (1UL << 16);
  if ((data[5] & 0x80U) != 0U) faults |= (1UL << 17);
  if ((data[5] & 0x40U) != 0U) faults |= APP_HALL_FAULT_BIT;

  return faults;
}

