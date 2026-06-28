#include "app.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_command_parser.h"
#include "app_can_io.h"
#include "app_config.h"
#include "app_utils.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"

#include "fsae_telemetry.pb.h"
#include "pb_encode.h"

#define APP_MODULE_COUNT                6U
#define APP_CELLS_PER_MODULE            23U
#define APP_TEMPS_PER_MODULE            8U
#define APP_TOTAL_CELL_COUNT            (APP_MODULE_COUNT * APP_CELLS_PER_MODULE)
#define APP_TOTAL_TEMP_COUNT            (APP_MODULE_COUNT * APP_TEMPS_PER_MODULE)
#define APP_CAN1_VOLTAGE_FRAMES_PER_MODULE  6U
#define APP_MODULE_VOLTAGE_VALID_MASK       ((uint8_t)((1U << APP_CAN1_VOLTAGE_FRAMES_PER_MODULE) - 1U))

#define APP_CAN1_BASE_VOLTAGE_ID        0x180050F3UL
#define APP_CAN1_BASE_TEMP_ID           0x184050F3UL
#define APP_CAN1_PACK_SUMMARY_ID        0x186050F4UL
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
#define APP_CAN2_MODE_COMMAND_ID        0x0310U
#define APP_CAN2_IVT_CURRENT_ID         0x0521U
#define APP_CAN2_IVT_U1_ID              0x0522U
#define APP_CAN2_IVT_POWER_ID           0x0526U
#define APP_CAN2_IVT_WH_ID              0x0528U
#define APP_IVT_MUX_CURRENT             0x00U
#define APP_IVT_MUX_U1                  0x01U
#define APP_IVT_MUX_POWER               0x05U
#define APP_IVT_MUX_WH                  0x07U
#define APP_IVT_RESULT_ERROR_MASK       0x0EU
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

#define APP_ENERGY_METER_SOURCE_UNKNOWN 0U
#define APP_ENERGY_METER_SOURCE_IVT     1U
#define APP_ENERGY_METER_SOURCE_FS      2U
#define APP_FS_STATUS_READY_BIT         (1U << 0)
#define APP_FS_STATUS_LOGGING_BIT       (1U << 1)
#define APP_FS_STATUS_TRIG_VOLTAGE_BIT  (1U << 2)
#define APP_FS_STATUS_TRIG_CURRENT_BIT  (1U << 3)

#define APP_MOTOR_COUNT                 4U
#define APP_MOTOR_FL                    0U
#define APP_MOTOR_FR                    1U
#define APP_MOTOR_RL                    2U
#define APP_MOTOR_RR                    3U

#define APP_LED_FAST_BLINK_MS           100U
#define APP_LED_SLOW_BLINK_MS           500U
#define APP_BASE_TELEMETRY_PERIOD_MS    100U
#define APP_MODULE_TELEMETRY_PERIOD_MS  500U
#define APP_RS485_TX_TIMEOUT_MS         200U
#define APP_DEBUG_CLI_ENABLE            0U
#define APP_DEBUG_UART_TIMEOUT_MS       50U
#define APP_DEBUG_CAN_TRACE_DEPTH       8U
#define APP_DEBUG_DUMP_COUNT            8U
#define APP_DEBUG_CLI_BUFFER_SIZE       32U
#define APP_DEBUG_CAN_ID_COUNTER_COUNT  16U
#define APP_COMMAND_RX_BUFFER_SIZE      16U
#define APP_COMMAND_RX_IDLE_MS          20U

#define APP_IMU_RAW_HEADER              0x55U
#define APP_IMU_RAW_SUBTYPE_TIME        0x50U
#define APP_IMU_RAW_SUBTYPE_ACCEL       0x51U
#define APP_IMU_RAW_SUBTYPE_GYRO        0x52U
#define APP_IMU_RAW_SUBTYPE_ANGLE       0x53U
#define APP_IMU_RAW_SUBTYPE_MAGNETIC    0x54U
#define APP_IMU_ANGLE_SUBCMD_ROLL       0x01U
#define APP_IMU_ANGLE_SUBCMD_PITCH      0x02U
#define APP_IMU_ANGLE_SUBCMD_YAW        0x03U

/* PC8 LED0 is high-level on in the G473 board notes. */
#define APP_LED_ON_STATE                GPIO_PIN_SET
#define APP_LED_OFF_STATE               GPIO_PIN_RESET

#define APP_HALL_FAULT_BIT              (1UL << 18)
#define APP_IMD_FAULT_BIT               (1UL << 19)

_Static_assert(APP_MODULE_COUNT == (sizeof(((fsae_TelemetryFrame *)0)->modules) / sizeof(((fsae_TelemetryFrame *)0)->modules[0])),
               "APP_MODULE_COUNT must match fsae_TelemetryFrame.modules capacity");
_Static_assert(APP_CELLS_PER_MODULE == 23U, "BatteryModule protobuf layout has 23 voltage fields");
_Static_assert(APP_TEMPS_PER_MODULE == 8U, "BatteryModule protobuf layout has 8 temperature fields");
_Static_assert(APP_CAN1_VOLTAGE_FRAMES_PER_MODULE < 8U, "module voltage validity mask must fit in uint8_t");

typedef enum
{
  APP_PROTOCOL_UNKNOWN = 0,
  APP_PROTOCOL_LEGACY,
  APP_PROTOCOL_MODERN
} AppProtocol;

typedef struct
{
  uint16_t cell_voltage_mv[APP_TOTAL_CELL_COUNT];
  int16_t cell_temp_deci_c[APP_TOTAL_TEMP_COUNT];
  uint8_t module_voltage_valid[APP_MODULE_COUNT];
  uint8_t module_temp_valid[APP_MODULE_COUNT];
  uint32_t module_voltage_updated_ms[APP_MODULE_COUNT];
  uint32_t module_temp_updated_ms[APP_MODULE_COUNT];

  uint16_t pack_voltage_deci_v;
  uint8_t battery_soc;
  uint8_t imd_signal;
  uint8_t battery_state;
  uint8_t battery_alarm_level;
  uint16_t summary_current_raw;
  uint32_t pack_summary_updated_ms;

  uint16_t cell_voltage_sum_deci_v;
  uint32_t cell_voltage_sum_updated_ms;
  uint8_t imd_diag_payload[8];
  uint32_t imd_diag_updated_ms;

  uint16_t max_cell_voltage_mv;
  uint16_t min_cell_voltage_mv;
  uint8_t max_cell_index_zero_based;
  uint8_t min_cell_index_zero_based;
  uint32_t cell_extrema_updated_ms;

  int16_t max_temp_deci_c;
  int16_t min_temp_deci_c;
  uint8_t max_temp_index_zero_based;
  uint8_t min_temp_index_zero_based;
  uint8_t cooling_control;
  uint32_t temp_extrema_updated_ms;

  uint8_t pos_relay_state;
  uint8_t neg_relay_state;
  uint8_t pre_relay_state;
  uint8_t charge_state;
  uint8_t charge_comm_state;
  uint16_t charge_request_voltage_deci_v;
  uint16_t charge_request_current_deci_a;
  uint16_t precharge_voltage_deci_v;
  uint32_t status_updated_ms;

  int32_t hall_current_ma;
  uint8_t hall_error;
  uint8_t hall_error_code;
  uint16_t hall_sensor_name;
  uint8_t hall_sw_version;
  uint32_t hall_updated_ms;

  uint16_t charger_fb_voltage_deci_v;
  uint16_t charger_fb_current_deci_a;
  uint8_t charger_fb_state;
  uint32_t charger_fb_updated_ms;

  uint16_t can2_pack_voltage_deci_v;
  uint16_t can2_pack_power_raw;
  uint16_t can2_pack_current_raw;
  uint8_t can2_soc;
  int16_t can2_max_temp_deci_c;
  uint32_t can2_power_updated_ms;

  uint8_t can2_battery_state;
  uint8_t can2_alarm_level;
  uint16_t can2_error_rom_low16;
  uint8_t slave_offline_mask;
  uint32_t can2_diag_updated_ms;

  int16_t can2_steering_angle_deci_deg;
  uint16_t can2_apps_open_deci_pct;
  uint16_t can2_oil_pressure_milli_kpa;
  uint32_t can2_datalogger_updated_ms;

  uint16_t can2_gps_speed_kmh;
  uint32_t can2_gps_speed_updated_ms;
  int16_t can2_accel_x_raw;
  int16_t can2_accel_y_raw;
  int16_t can2_accel_z_raw;
  uint32_t can2_accel_updated_ms;
  int16_t can2_yaw_rate_raw;
  uint32_t can2_gyro_updated_ms;
  int16_t can2_yaw_raw;
  uint32_t can2_yaw_updated_ms;

  int32_t ivt_current_ma;
  int32_t ivt_voltage_u1_mv;
  int32_t ivt_power_w;
  int32_t ivt_energy_wh;
  uint8_t ivt_current_state;
  uint8_t ivt_voltage_u1_state;
  uint8_t ivt_power_state;
  uint8_t ivt_energy_state;
  uint8_t ivt_byte_order;
  uint32_t ivt_current_updated_ms;
  uint32_t ivt_voltage_u1_updated_ms;
  uint32_t ivt_power_updated_ms;
  uint32_t ivt_energy_updated_ms;

  int32_t fs_current_ma;
  int32_t fs_voltage_mv;
  int32_t fs_power_w;
  int32_t fs_energy_wh;
  uint8_t fs_status;
  uint8_t fs_msg_counter;
  uint32_t fs_status_updated_ms;
  uint32_t fs_current_updated_ms;
  uint32_t fs_voltage_updated_ms;
  uint32_t fs_power_updated_ms;
  uint32_t fs_energy_updated_ms;
  uint8_t energy_meter_auto_source;

  int16_t motor_torque_0p1pct[APP_MOTOR_COUNT];
  int16_t motor_rpm[APP_MOTOR_COUNT];
  int16_t motor_temp_deci_c[APP_MOTOR_COUNT];
  int16_t motor_inverter_temp_deci_c[APP_MOTOR_COUNT];
  int16_t motor_igbt_temp_deci_c[APP_MOTOR_COUNT];
  uint32_t motor_diagnostic_number[APP_MOTOR_COUNT];
  uint8_t motor_logic_state[APP_MOTOR_COUNT];
  uint32_t motor_torque_updated_ms;
  uint32_t motor_diag_updated_ms;
  uint32_t motor_rpm_updated_ms;
  uint32_t motor_temp_updated_ms;
  uint32_t motor_inverter_temp_updated_ms;
  uint32_t motor_igbt_temp_updated_ms;
  uint32_t motor_logic_state_updated_ms;
  int8_t vehicle_mode_flag;
  uint32_t vehicle_mode_updated_ms;

  uint32_t alarm_fault_code;
  uint8_t hall_fault_active;
  uint8_t imd_fault_active;
  uint32_t battery_fault_code;
  uint32_t alarm_updated_ms;

  AppProtocol protocol;
  uint8_t can1_seen;
  uint8_t can2_seen;
} AppTelemetryState;

typedef struct
{
  uint8_t bus_index;
  uint8_t is_extended_id;
  uint8_t dlc;
  uint32_t id;
  uint8_t data[8];
} AppCanTraceFrame;

typedef struct
{
  uint8_t bus_index;
  uint8_t is_extended_id;
  uint32_t id;
  uint32_t count;
  uint32_t last_ms;
} AppCanIdCounter;

typedef struct
{
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} AppCan2TxFrame;

static volatile AppTelemetryState g_app_state;
static AppTelemetryState g_tx_snapshot;
static fsae_TelemetryFrame g_tx_frame;
static uint8_t g_pb_buffer[fsae_TelemetryFrame_size];
static const fsae_TelemetryFrame g_tx_frame_prototype = fsae_TelemetryFrame_init_zero;
static const fsae_BatteryModule g_battery_module_prototype = fsae_BatteryModule_init_zero;
static const fsae_MotorState g_motor_state_prototype = fsae_MotorState_init_zero;
static uint32_t g_frame_counter;
static uint32_t g_last_base_tx_ms;
static uint32_t g_last_module_tx_ms;
static uint32_t g_last_led_toggle_ms;
static uint32_t g_last_rs485_tx_ms;
static uint8_t g_led_blink_on;
static volatile AppCanTraceFrame g_can_trace[APP_DEBUG_CAN_TRACE_DEPTH];
static volatile uint8_t g_can_trace_write_index;
static volatile uint8_t g_can_trace_count;
static volatile AppCanIdCounter g_can_id_counters[APP_DEBUG_CAN_ID_COUNTER_COUNT];
static volatile uint8_t g_can_id_counter_count;
static volatile uint8_t g_cli_rx_byte;
static volatile char g_cli_rx_buffer[APP_DEBUG_CLI_BUFFER_SIZE];
static volatile uint8_t g_cli_rx_length;
static volatile char g_cli_command_buffer[APP_DEBUG_CLI_BUFFER_SIZE];
static volatile uint8_t g_cli_command_ready;
static volatile uint8_t g_command_rx_byte;
static volatile uint8_t g_command_rx_buffer[APP_COMMAND_RX_BUFFER_SIZE];
static volatile uint8_t g_command_rx_length;
static volatile uint8_t g_command_rx_ready;
static volatile uint8_t g_command_rx_overflow;
static volatile uint32_t g_command_rx_updated_ms;
static uint8_t g_command_pending_value;
static uint8_t g_command_pending;
#if APP_DEBUG_CLI_ENABLE
static uint8_t g_debug_last_command_bytes[APP_COMMAND_RX_BUFFER_SIZE];
static uint8_t g_debug_last_command_length;
static uint8_t g_debug_last_command_parse_ok;
static uint8_t g_debug_last_command_value;
static uint32_t g_debug_command_rx_count;
static uint32_t g_debug_command_parse_fail_count;
static uint32_t g_debug_mode_tx_ok_count;
static uint32_t g_debug_mode_tx_error_count;
static uint8_t g_debug_last_mode_tx_value;
static HAL_StatusTypeDef g_debug_last_mode_tx_status;
static uint32_t g_debug_last_mode_tx_ms;
#endif

static uint8_t App_IvtStateIsUsable(uint8_t state);
#if APP_CAN2_ENERGY_METER_MODE != APP_CAN2_ENERGY_METER_MODE_FS
static uint8_t App_IvtValueIsPlausible(uint32_t std_id, int32_t value);
static uint8_t App_IvtDecodeResult(uint32_t std_id, const uint8_t *data, int32_t *value);
#endif
static void App_SetProtocol(AppProtocol protocol);
static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now);
static void App_UpdateFaultCode(void);
static void App_SetStatusLed(uint8_t on);
static void App_UpdateStatusLed(uint32_t now);
static void App_DebugRecordCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data);
static void App_DebugPollCli(void);
#if APP_DEBUG_CLI_ENABLE
static void App_DebugWrite(const char *text);
static void App_DebugPrintf(const char *format, ...);
static void App_DebugHandleCommand(const char *command);
static void App_DebugPrintHelp(void);
static void App_DebugPrintStatus(void);
static void App_DebugPrintRecentCan(const char *command);
static void App_DebugPrintCanIds(void);
static void App_DebugPrintCommandState(void);
static void App_DebugClear(void);
static void App_DebugRecordCanId(uint8_t bus_index, uint8_t is_extended_id, uint32_t id, uint32_t now);
static void App_DebugRecordCommandRx(const uint8_t *data, uint8_t length, uint8_t parse_ok, uint8_t command_value);
static void App_DebugRecordModeTx(uint8_t command_value, HAL_StatusTypeDef status, uint32_t now);
#endif
static void App_CommandPoll(uint32_t now);
static void App_CommandRxByte(uint8_t rx);
static void App_CommandQueue(uint8_t command_value);
static void App_CommandServiceTx(void);
static void App_CommandRestartRx(void);
static HAL_StatusTypeDef App_SendModeCommand(uint8_t command_value);
static void App_ProcessCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data);
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
static float App_GetHvVoltage(const AppTelemetryState *state, uint32_t now);
static float App_GetHvCurrent(const AppTelemetryState *state, uint32_t now);
static uint8_t App_GetEnergyMeterSource(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetBatterySoc(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetVcuStatus(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetReadyToDrive(const AppTelemetryState *state, uint32_t now);
static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now);
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
static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules);
static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size);

void App_Init(void)
{
  memset((void *)&g_app_state, 0, sizeof(g_app_state));
  g_frame_counter = 0U;
  g_last_base_tx_ms = HAL_GetTick();
  g_last_module_tx_ms = HAL_GetTick();
  g_last_led_toggle_ms = HAL_GetTick();
  g_last_rs485_tx_ms = 0U;
  g_led_blink_on = 0U;
  g_can_trace_write_index = 0U;
  g_can_trace_count = 0U;
  g_can_id_counter_count = 0U;
  g_cli_rx_length = 0U;
  g_cli_command_ready = 0U;
  g_command_rx_length = 0U;
  g_command_rx_ready = 0U;
  g_command_rx_overflow = 0U;
  g_command_rx_updated_ms = 0U;
  g_command_pending_value = 0U;
  g_command_pending = 0U;
#if APP_DEBUG_CLI_ENABLE
  g_debug_last_command_length = 0U;
  g_debug_last_command_parse_ok = 0U;
  g_debug_last_command_value = 0U;
  g_debug_command_rx_count = 0U;
  g_debug_command_parse_fail_count = 0U;
  g_debug_mode_tx_ok_count = 0U;
  g_debug_mode_tx_error_count = 0U;
  g_debug_last_mode_tx_value = 0U;
  g_debug_last_mode_tx_status = HAL_OK;
  g_debug_last_mode_tx_ms = 0U;
#endif

  App_SetStatusLed(0U);

  if (App_CanIo_Init(App_ProcessCanRx) != HAL_OK)
  {
    Error_Handler();
  }

#if APP_DEBUG_CLI_ENABLE
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_cli_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_command_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t include_modules;

  App_UpdateStatusLed(now);
  App_DebugPollCli();
  App_CommandPoll(now);
  App_CommandServiceTx();
  App_CanIo_Service();

  if ((g_app_state.can1_seen == 0U) && (g_app_state.can2_seen == 0U))
  {
    return;
  }

  if ((uint32_t)(now - g_last_base_tx_ms) < APP_BASE_TELEMETRY_PERIOD_MS)
  {
    return;
  }

  include_modules = (uint8_t)(((uint32_t)(now - g_last_module_tx_ms) >= APP_MODULE_TELEMETRY_PERIOD_MS) ? 1U : 0U);
  if (App_SendTelemetry(now, include_modules) == HAL_OK)
  {
    g_last_base_tx_ms = now;
    if (include_modules != 0U)
    {
      g_last_module_tx_ms = now;
    }
  }
}

static uint8_t App_IvtStateIsUsable(uint8_t state)
{
  return ((state & APP_IVT_RESULT_ERROR_MASK) == 0U) ? 1U : 0U;
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

static void App_UpdateFaultCode(void)
{
  g_app_state.battery_fault_code = App_ComputeFaultCode((const AppTelemetryState *)&g_app_state, HAL_GetTick());
}

static void App_SetStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (on != 0U) ? APP_LED_ON_STATE : APP_LED_OFF_STATE);
}

static void App_UpdateStatusLed(uint32_t now)
{
  uint8_t has_fault = (App_ComputeFaultCode((const AppTelemetryState *)&g_app_state, now) != 0U) ? 1U : 0U;
  uint8_t telemetry_active = ((g_last_rs485_tx_ms != 0U) &&
                              ((g_app_state.can1_seen != 0U) || (g_app_state.can2_seen != 0U))) ? 1U : 0U;

  if (has_fault != 0U)
  {
    if ((uint32_t)(now - g_last_led_toggle_ms) >= APP_LED_FAST_BLINK_MS)
    {
      g_last_led_toggle_ms = now;
      g_led_blink_on = (uint8_t)(g_led_blink_on == 0U);
      App_SetStatusLed(g_led_blink_on);
    }
    return;
  }

  if (telemetry_active != 0U)
  {
    g_led_blink_on = 1U;
    App_SetStatusLed(1U);
    return;
  }

  if ((uint32_t)(now - g_last_led_toggle_ms) >= APP_LED_SLOW_BLINK_MS)
  {
    g_last_led_toggle_ms = now;
    g_led_blink_on = (uint8_t)(g_led_blink_on == 0U);
    App_SetStatusLed(g_led_blink_on);
  }
}

static void App_DebugRecordCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data)
{
#if APP_DEBUG_CLI_ENABLE
  uint32_t primask = __get_PRIMASK();
  AppCanTraceFrame *slot = (AppCanTraceFrame *)&g_can_trace[g_can_trace_write_index];
  uint8_t bus_index = (bus == APP_FDCAN_BUS_CAN1) ? 1U :
                      ((bus == APP_FDCAN_BUS_CANB) ? 2U :
                      ((bus == APP_FDCAN_BUS_CANC) ? 3U : 0U));
  uint8_t is_extended_id = (header->IDE == APP_CAN_ID_EXT) ? 1U : 0U;
  uint32_t id = (header->IDE == APP_CAN_ID_EXT) ? header->ExtId : header->StdId;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  slot->bus_index = bus_index;
  slot->is_extended_id = is_extended_id;
  slot->dlc = header->DLC;
  slot->id = id;
  memcpy(slot->data, data, 8U);

  g_can_trace_write_index = (uint8_t)((g_can_trace_write_index + 1U) % APP_DEBUG_CAN_TRACE_DEPTH);
  if (g_can_trace_count < APP_DEBUG_CAN_TRACE_DEPTH)
  {
    g_can_trace_count++;
  }
  App_DebugRecordCanId(bus_index, is_extended_id, id, now);
  if (primask == 0U)
  {
    __enable_irq();
  }
#else
  (void)bus;
  (void)header;
  (void)data;
#endif
}

static void App_DebugPollCli(void)
{
#if APP_DEBUG_CLI_ENABLE
  uint8_t ready;
  char command[APP_DEBUG_CLI_BUFFER_SIZE];

  __disable_irq();
  ready = g_cli_command_ready;
  if (ready != 0U)
  {
    memcpy(command, (const void *)g_cli_command_buffer, APP_DEBUG_CLI_BUFFER_SIZE);
    g_cli_command_ready = 0U;
  }
  __enable_irq();

  if (ready == 0U)
  {
    return;
  }

  App_DebugHandleCommand(command);
#endif
}

#if APP_DEBUG_CLI_ENABLE
static void App_DebugWrite(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), APP_DEBUG_UART_TIMEOUT_MS);
}

static void App_DebugPrintf(const char *format, ...)
{
  char line[160];
  va_list args;
  int written;

  va_start(args, format);
  written = vsnprintf(line, sizeof(line), format, args);
  va_end(args);

  if (written > 0)
  {
    if (written >= (int)sizeof(line))
    {
      written = (int)sizeof(line) - 1;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)written, APP_DEBUG_UART_TIMEOUT_MS);
  }
}

static void App_DebugHandleCommand(const char *command)
{
  if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0))
  {
    App_DebugPrintHelp();
  }
  else if (strcmp(command, "status") == 0)
  {
    App_DebugPrintStatus();
  }
  else if (strcmp(command, "can") == 0)
  {
    App_DebugPrintRecentCan(command);
  }
  else if (strcmp(command, "ids") == 0)
  {
    App_DebugPrintCanIds();
  }
  else if (strcmp(command, "cmd") == 0)
  {
    App_DebugPrintCommandState();
  }
  else if (strcmp(command, "clear") == 0)
  {
    App_DebugClear();
  }
  else
  {
    App_DebugPrintf("unknown command: %s\r\n", command);
    App_DebugPrintHelp();
  }
}

static void App_DebugPrintHelp(void)
{
  App_DebugWrite("commands: help status can ids cmd clear\r\n");
}

static void App_DebugPrintStatus(void)
{
  AppTelemetryState snapshot;
  uint32_t now = HAL_GetTick();
  uint32_t last_rs485_tx_ms = g_last_rs485_tx_ms;
  uint32_t frame_counter = g_frame_counter;

  __disable_irq();
  snapshot = *((const AppTelemetryState *)&g_app_state);
  __enable_irq();

  App_DebugPrintf(
      "status: tick=%lu frames=%lu rs485_last_age=%lu can1=%u can2=%u mode_flag=%d mode_age=%lu fault=0x%08lX\r\n",
      (unsigned long)now,
      (unsigned long)frame_counter,
      (unsigned long)((last_rs485_tx_ms == 0U) ? 0UL : (now - last_rs485_tx_ms)),
      (unsigned int)snapshot.can1_seen,
      (unsigned int)snapshot.can2_seen,
      (int)snapshot.vehicle_mode_flag,
      (unsigned long)((snapshot.vehicle_mode_updated_ms == 0U) ? 0UL : (now - snapshot.vehicle_mode_updated_ms)),
      (unsigned long)App_ComputeFaultCode(&snapshot, now));
}

static void App_DebugPrintRecentCan(const char *command)
{
  AppCanTraceFrame frames[APP_DEBUG_DUMP_COUNT];
  uint8_t count;
  uint8_t write_index;
  uint8_t i;

  __disable_irq();
  count = g_can_trace_count;
  write_index = g_can_trace_write_index;
  if (count > APP_DEBUG_DUMP_COUNT)
  {
    count = APP_DEBUG_DUMP_COUNT;
  }
  for (i = 0U; i < count; ++i)
  {
    uint8_t src_index = (uint8_t)((write_index + APP_DEBUG_CAN_TRACE_DEPTH - count + i) % APP_DEBUG_CAN_TRACE_DEPTH);
    frames[i] = *((const AppCanTraceFrame *)&g_can_trace[src_index]);
  }
  __enable_irq();

  if (count == 0U)
  {
    App_DebugPrintf("cli[%s]: no CAN frames captured yet\r\n", command);
    return;
  }

  App_DebugPrintf("cli[%s]: dumping %u recent CAN frames\r\n", command, (unsigned int)count);

  for (i = 0U; i < count; ++i)
  {
    App_DebugPrintf(
      "CAN%u %s 0x%08lX DLC=%u [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
      (unsigned int)frames[i].bus_index,
      (frames[i].is_extended_id != 0U) ? "EXT" : "STD",
      (unsigned long)frames[i].id,
      (unsigned int)frames[i].dlc,
      (unsigned int)frames[i].data[0],
      (unsigned int)frames[i].data[1],
      (unsigned int)frames[i].data[2],
      (unsigned int)frames[i].data[3],
      (unsigned int)frames[i].data[4],
      (unsigned int)frames[i].data[5],
      (unsigned int)frames[i].data[6],
      (unsigned int)frames[i].data[7]
    );
  }
}

static void App_DebugPrintCanIds(void)
{
  AppCanIdCounter counters[APP_DEBUG_CAN_ID_COUNTER_COUNT];
  uint8_t count;
  uint8_t i;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  count = g_can_id_counter_count;
  memcpy(counters, (const void *)g_can_id_counters, sizeof(counters));
  __enable_irq();

  if (count == 0U)
  {
    App_DebugWrite("ids: none\r\n");
    return;
  }

  App_DebugPrintf("ids: %u tracked\r\n", (unsigned int)count);
  for (i = 0U; i < count; ++i)
  {
    App_DebugPrintf("CAN%u %s 0x%08lX count=%lu age=%lu\r\n",
                    (unsigned int)counters[i].bus_index,
                    (counters[i].is_extended_id != 0U) ? "EXT" : "STD",
                    (unsigned long)counters[i].id,
                    (unsigned long)counters[i].count,
                    (unsigned long)(now - counters[i].last_ms));
  }
}

static void App_DebugPrintCommandState(void)
{
  uint8_t bytes[APP_COMMAND_RX_BUFFER_SIZE];
  uint8_t length;
  uint8_t parse_ok;
  uint8_t command_value = 0U;
  uint32_t rx_count;
  uint32_t fail_count;
  uint32_t tx_ok_count;
  uint32_t tx_error_count;
  uint8_t last_tx_value;
  HAL_StatusTypeDef last_tx_status;
  uint32_t last_tx_ms;
  uint32_t now = HAL_GetTick();
  uint8_t i;

  __disable_irq();
  memcpy(bytes, g_debug_last_command_bytes, sizeof(bytes));
  length = g_debug_last_command_length;
  parse_ok = g_debug_last_command_parse_ok;
  command_value = g_debug_last_command_value;
  rx_count = g_debug_command_rx_count;
  fail_count = g_debug_command_parse_fail_count;
  tx_ok_count = g_debug_mode_tx_ok_count;
  tx_error_count = g_debug_mode_tx_error_count;
  last_tx_value = g_debug_last_mode_tx_value;
  last_tx_status = g_debug_last_mode_tx_status;
  last_tx_ms = g_debug_last_mode_tx_ms;
  __enable_irq();

  App_DebugPrintf("cmd: rx=%lu parse_fail=%lu last_len=%u parse_ok=%u value=%u\r\n",
                  (unsigned long)rx_count,
                  (unsigned long)fail_count,
                  (unsigned int)length,
                  (unsigned int)parse_ok,
                  (unsigned int)command_value);
  App_DebugWrite("cmd bytes:");
  for (i = 0U; i < length; ++i)
  {
    App_DebugPrintf(" %02X", (unsigned int)bytes[i]);
  }
  App_DebugWrite("\r\n");
  App_DebugPrintf("mode_tx: ok=%lu err=%lu last_value=%u last_status=%d age=%lu\r\n",
                  (unsigned long)tx_ok_count,
                  (unsigned long)tx_error_count,
                  (unsigned int)last_tx_value,
                  (int)last_tx_status,
                  (unsigned long)((last_tx_ms == 0U) ? 0UL : (now - last_tx_ms)));
}

static void App_DebugClear(void)
{
  __disable_irq();
  g_can_trace_write_index = 0U;
  g_can_trace_count = 0U;
  g_can_id_counter_count = 0U;
  memset((void *)g_can_id_counters, 0, sizeof(g_can_id_counters));
  g_debug_command_rx_count = 0U;
  g_debug_command_parse_fail_count = 0U;
  g_debug_mode_tx_ok_count = 0U;
  g_debug_mode_tx_error_count = 0U;
  __enable_irq();

  App_DebugWrite("debug counters cleared\r\n");
}

static void App_DebugRecordCanId(uint8_t bus_index, uint8_t is_extended_id, uint32_t id, uint32_t now)
{
  uint8_t i;
  uint8_t count = g_can_id_counter_count;

  for (i = 0U; i < count; ++i)
  {
    if ((g_can_id_counters[i].bus_index == bus_index) &&
        (g_can_id_counters[i].is_extended_id == is_extended_id) &&
        (g_can_id_counters[i].id == id))
    {
      g_can_id_counters[i].count++;
      g_can_id_counters[i].last_ms = now;
      return;
    }
  }

  if (count < APP_DEBUG_CAN_ID_COUNTER_COUNT)
  {
    g_can_id_counters[count].bus_index = bus_index;
    g_can_id_counters[count].is_extended_id = is_extended_id;
    g_can_id_counters[count].id = id;
    g_can_id_counters[count].count = 1U;
    g_can_id_counters[count].last_ms = now;
    g_can_id_counter_count = (uint8_t)(count + 1U);
  }
}

static void App_DebugRecordCommandRx(const uint8_t *data, uint8_t length, uint8_t parse_ok, uint8_t command_value)
{
  if (length > APP_COMMAND_RX_BUFFER_SIZE)
  {
    length = APP_COMMAND_RX_BUFFER_SIZE;
  }

  __disable_irq();
  memcpy(g_debug_last_command_bytes, data, length);
  g_debug_last_command_length = length;
  g_debug_last_command_parse_ok = parse_ok;
  g_debug_last_command_value = command_value;
  g_debug_command_rx_count++;
  if (parse_ok == 0U)
  {
    g_debug_command_parse_fail_count++;
  }
  __enable_irq();
}

static void App_DebugRecordModeTx(uint8_t command_value, HAL_StatusTypeDef status, uint32_t now)
{
  __disable_irq();
  g_debug_last_mode_tx_value = command_value;
  g_debug_last_mode_tx_status = status;
  g_debug_last_mode_tx_ms = now;
  if (status == HAL_OK)
  {
    g_debug_mode_tx_ok_count++;
  }
  else
  {
    g_debug_mode_tx_error_count++;
  }
  __enable_irq();
}
#endif

static void App_CommandPoll(uint32_t now)
{
  uint8_t buffer[APP_COMMAND_RX_BUFFER_SIZE];
  uint8_t length = 0U;
  uint8_t ready = 0U;
  uint8_t command_value = 0U;
  uint8_t parse_ok;

  __disable_irq();
  if ((g_command_rx_ready != 0U) ||
      ((g_command_rx_length > 0U) && ((uint32_t)(now - g_command_rx_updated_ms) >= APP_COMMAND_RX_IDLE_MS)))
  {
    ready = 1U;
    length = g_command_rx_length;
    memcpy(buffer, (const void *)g_command_rx_buffer, length);
    g_command_rx_length = 0U;
    g_command_rx_ready = 0U;
    g_command_rx_overflow = 0U;
  }
  __enable_irq();

  if (ready == 0U)
  {
    return;
  }

  parse_ok = App_CommandTryParse(buffer, length, &command_value);
#if APP_DEBUG_CLI_ENABLE
  App_DebugRecordCommandRx(buffer, length, parse_ok, command_value);
#endif

  if (parse_ok != 0U)
  {
    App_CommandQueue(command_value);
  }
}

static void App_CommandRxByte(uint8_t rx)
{
  g_command_rx_updated_ms = HAL_GetTick();

  if ((rx == '\r') || (rx == '\n') || (rx == 0U))
  {
    if (g_command_rx_length > 0U)
    {
      g_command_rx_ready = 1U;
    }
    return;
  }

  if (g_command_rx_ready != 0U)
  {
    return;
  }

  if (g_command_rx_length < APP_COMMAND_RX_BUFFER_SIZE)
  {
    g_command_rx_buffer[g_command_rx_length] = rx;
    g_command_rx_length++;
  }
  else
  {
    g_command_rx_length = 0U;
    g_command_rx_overflow = 1U;
  }
}

static void App_CommandQueue(uint8_t command_value)
{
  __disable_irq();
  g_command_pending_value = command_value;
  g_command_pending = 1U;
  __enable_irq();
}

static void App_CommandServiceTx(void)
{
  uint8_t command_value;

  __disable_irq();
  if (g_command_pending == 0U)
  {
    __enable_irq();
    return;
  }
  command_value = g_command_pending_value;
  g_command_pending = 0U;
  __enable_irq();

  if (App_SendModeCommand(command_value) != HAL_OK)
  {
    App_CommandQueue(command_value);
  }
}

static void App_CommandRestartRx(void)
{
  (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_command_rx_byte, 1U);
}

static HAL_StatusTypeDef App_SendModeCommand(uint8_t command_value)
{
  uint8_t tx_data[1];
  HAL_StatusTypeDef status;

  if ((command_value < APP_MODE_CMD_STRAIGHT) || (command_value > APP_MODE_CMD_ENDURANCE))
  {
#if APP_DEBUG_CLI_ENABLE
    App_DebugRecordModeTx(command_value, HAL_ERROR, HAL_GetTick());
#endif
    return HAL_ERROR;
  }

  tx_data[0] = command_value;

  status = App_CanIo_SendCanBStd(APP_CAN2_MODE_COMMAND_ID, tx_data, 1U);
#if APP_DEBUG_CLI_ENABLE
  App_DebugRecordModeTx(command_value, status, HAL_GetTick());
#endif
  return status;
}

static void App_ProcessCanRx(AppFdcanBus bus, const AppCanRxHeader *header, const uint8_t *data)
{
  App_DebugRecordCanRx(bus, header, data);

  if (bus == APP_FDCAN_BUS_CAN1)
  {
    App_ProcessCan1Rx(header, data);
  }
  else if (bus == APP_FDCAN_BUS_CANB)
  {
    App_ProcessCan2Rx(header, data);
  }
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
    App_UpdateFaultCode();
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
  App_UpdateFaultCode();
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
  App_UpdateFaultCode();
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

static uint32_t App_ComputeFaultCode(const AppTelemetryState *state, uint32_t now)
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

static uint32_t App_GetFaultCode(const AppTelemetryState *state, uint32_t now)
{
  return App_ComputeFaultCode(state, now);
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

static HAL_StatusTypeDef App_SendTelemetry(uint32_t now, uint8_t include_modules)
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
  *snapshot = *((const AppTelemetryState *)&g_app_state);
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
  frame->fault_code = App_GetFaultCode(snapshot, now);
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

  return App_RS485_Transmit(g_pb_buffer, (uint16_t)stream.bytes_written);
}

static HAL_StatusTypeDef App_RS485_Transmit(const uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  uint32_t start_ms;

  (void)HAL_UART_AbortReceive(&huart2);

  status = HAL_UART_Transmit(&huart2, (uint8_t *)data, size, APP_RS485_TX_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    App_CommandRestartRx();
    return status;
  }

  start_ms = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) > APP_RS485_TX_TIMEOUT_MS)
    {
      App_CommandRestartRx();
      return HAL_TIMEOUT;
    }
  }

  App_CommandRestartRx();
  g_last_rs485_tx_ms = HAL_GetTick();
  return HAL_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if APP_DEBUG_CLI_ENABLE
  if (huart->Instance == USART1)
  {
    uint8_t rx = g_cli_rx_byte;

    if ((rx == '\r') || (rx == '\n'))
    {
      if ((g_cli_rx_length > 0U) && (g_cli_command_ready == 0U))
      {
        memcpy((void *)g_cli_command_buffer, (const void *)g_cli_rx_buffer, g_cli_rx_length);
        g_cli_command_buffer[g_cli_rx_length] = '\0';
        g_cli_command_ready = 1U;
      }
      g_cli_rx_length = 0U;
    }
    else if (g_cli_rx_length < (APP_DEBUG_CLI_BUFFER_SIZE - 1U))
    {
      g_cli_rx_buffer[g_cli_rx_length] = (char)rx;
      g_cli_rx_length++;
    }
    else
    {
      g_cli_rx_length = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_cli_rx_byte, 1U);
  }
#endif

  if (huart->Instance == USART2)
  {
    App_CommandRxByte(g_command_rx_byte);
    App_CommandRestartRx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    App_CommandRestartRx();
  }
}
