#ifndef __APP_STATE_H__
#define __APP_STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_config.h"

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

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_H__ */
