/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef INCLUDE_MAVLINK_TELEMETRY_H_
#define INCLUDE_MAVLINK_TELEMETRY_H_

#include <array>
#include "core/core.h"
#include "global_defs/global_defs.h"
#include "gnss/gnss.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

class MavLinkTelemetry {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void sys_id(const uint8_t sys_id) {sys_id_ = sys_id;}
  /* Update and message handler methods */
  void Update();
  void MsgHandler(const mavlink_message_t &ref);
  /* System and component ID getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* Config data stream rates */
  inline void raw_sens_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_RAW_SENS_STREAM] = val;
  }
  inline int32_t raw_sens_stream_period_ms() const {
    return data_stream_period_ms_[SRx_RAW_SENS_STREAM];
  }
  inline void ext_status_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_EXT_STAT_STREAM] = val;
  }
  inline int32_t ext_status_stream_period_ms() const {
    return data_stream_period_ms_[SRx_EXT_STAT_STREAM];
  }
  inline void rc_chan_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_RC_CHAN_STREAM] = val;
  }
  inline int32_t rc_chan_stream_period_ms() const {
    return data_stream_period_ms_[SRx_RC_CHAN_STREAM];
  }
  inline void pos_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_POSITION_STREAM] = val;
  }
  inline int32_t pos_stream_period_ms() const {
    return data_stream_period_ms_[SRx_POSITION_STREAM];
  }
  inline void extra1_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_EXTRA1_STREAM] = val;
  }
  inline int32_t extra1_stream_period_ms() const {
    return data_stream_period_ms_[SRx_EXTRA1_STREAM];
  }
  inline void extra2_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_EXTRA2_STREAM] = val;
  }
  inline int32_t extra2_stream_period_ms() const {
    return data_stream_period_ms_[SRx_EXTRA2_STREAM];
  }
  inline void extra3_stream_period_ms(const int32_t val) {
    data_stream_period_ms_[SRx_EXTRA3_STREAM] = val;
  }
  inline int32_t extra3_stream_period_ms() const {
    return data_stream_period_ms_[SRx_EXTRA3_STREAM];
  }
  /* System */
  inline void sys_time_us(const uint64_t val) {sys_time_us_ = val;}
  inline void cpu_load(uint32_t frame_time_us, uint32_t frame_period_us) {
    frame_time_us_ = frame_time_us;
    frame_period_us_ = frame_period_us;
  }
  /* Installed sensors */
  inline void gyro_installed(const bool val) {gyro_installed_ = val;}
  inline void accel_installed(const bool val) {accel_installed_ = val;}
  inline void mag_installed(const bool val) {mag_installed_ = val;}
  inline void static_pres_installed(const bool val) {
    static_pres_installed_ = val;
  }
  inline void diff_pres_installed(const bool val) {diff_pres_installed_ = val;}
  inline void gnss_installed(const bool val) {gnss_installed_ = val;}
  inline void inceptor_installed(const bool val) {inceptor_installed_ = val;}
  inline void gyro_healthy(const bool val) {gyro_healthy_ = val;}
  inline void accel_healthy(const bool val) {accel_healthy_ = val;}
  inline void mag_healthy(const bool val) {mag_healthy_ = val;}
  inline void static_pres_healthy(const bool val) {static_pres_healthy_ = val;}
  inline void diff_pres_healthy(const bool val) {diff_pres_healthy_ = val;}
  inline void gnss_healthy(const bool val) {gnss_healthy_ = val;}
  inline void inceptor_healthy(const bool val) {inceptor_healthy_ = val;}
  /* Battery */
  inline void battery_volt(const float val) {
    battery_volt_.set = true;
    battery_volt_.val = val;
  }
  /* IMU data */
  inline void imu_accel_x_mps2(const float val) {imu_accel_x_mps2_ = val;}
  inline void imu_accel_y_mps2(const float val) {imu_accel_y_mps2_ = val;}
  inline void imu_accel_z_mps2(const float val) {imu_accel_z_mps2_ = val;}
  inline void imu_gyro_x_radps(const float val) {imu_gyro_x_radps_ = val;}
  inline void imu_gyro_y_radps(const float val) {imu_gyro_y_radps_ = val;}
  inline void imu_gyro_z_radps(const float val) {imu_gyro_z_radps_ = val;}
  inline void imu_mag_x_ut(const float val) {imu_mag_x_ut_ = val;}
  inline void imu_mag_y_ut(const float val) {imu_mag_y_ut_ = val;}
  inline void imu_mag_z_ut(const float val) {imu_mag_z_ut_ = val;}
  inline void imu_die_temp_c(const float val) {
    imu_die_temp_c_.set = true;
    imu_die_temp_c_.val = val;
  }
  /* Airdata */
  inline void static_pres_pa(const float val) {static_pres_pa_ = val;}
  inline void diff_pres_pa(const float val) {diff_pres_pa_ = val;}
  inline void static_pres_die_temp_c(const float val) {
    static_pres_die_temp_c_ = val;
  }
  inline void diff_pres_die_temp_c(const float val) {
    diff_pres_die_temp_c_.set = true;
    diff_pres_die_temp_c_.val = val;
  }
  /* GNSS data */
  inline void gnss_fix(const int8_t val) {gnss_fix_ = val;}
  inline void gnss_num_sats(const uint8_t val) {
    gnss_num_sv_.set = true;
    gnss_num_sv_.val = val;
  }
  inline void gnss_lat_rad(const double val) {gnss_lat_rad_ = val;}
  inline void gnss_lon_rad(const double val) {gnss_lon_rad_ = val;}
  inline void gnss_alt_msl_m(const float val) {gnss_alt_msl_m_ = val;}
  inline void gnss_alt_wgs84_m(const float val) {gnss_alt_wgs84_m_ = val;}
  inline void gnss_hdop(const float val) {
    gnss_hdop_.set = true;
    gnss_hdop_.val = val;
  }
  inline void gnss_vdop(const float val) {
    gnss_vdop_.set = true;
    gnss_vdop_.val = val;
  }
  inline void gnss_track_rad(const float val) {
    gnss_track_rad_.set = true;
    gnss_track_rad_.val = val;
  }
  inline void gnss_spd_mps(const float val) {
    gnss_vel_mps_.set = true;
    gnss_vel_mps_.val = val;
  }
  inline void gnss_horz_acc_m(const float val) {gnss_horz_acc_m_ = val;}
  inline void gnss_vert_acc_m(const float val) {gnss_vert_acc_m_ = val;}
  inline void gnss_vel_acc_mps(const float val) {gnss_vel_acc_mps_ = val;}
  inline void gnss_track_acc_rad(const float val) {gnss_track_acc_rad_ = val;}
  /* Estimation data */
  inline void nav_lat_rad(const double val) {nav_lat_rad_ = val;}
  inline void nav_lon_rad(const double val) {nav_lon_rad_ = val;}
  inline void nav_alt_msl_m(const float val) {nav_alt_msl_m_ = val;}
  inline void nav_alt_agl_m(const float val) {nav_alt_agl_m_ = val;}
  inline void nav_north_pos_m(const float val) {nav_north_pos_m_ = val;}
  inline void nav_east_pos_m(const float val) {nav_east_pos_m_ = val;}
  inline void nav_down_pos_m(const float val) {nav_down_pos_m_ = val;}
  inline void nav_north_vel_mps(const float val) {nav_north_vel_mps_ = val;}
  inline void nav_east_vel_mps(const float val) {nav_east_vel_mps_ = val;}
  inline void nav_down_vel_mps(const float val) {nav_down_vel_mps_ = val;}
  inline void nav_gnd_spd_mps(const float val) {nav_gnd_spd_mps_ = val;}
  inline void nav_ias_mps(const float val) {nav_ias_mps_ = val;}
  inline void nav_pitch_rad(const float val) {nav_pitch_rad_ = val;}
  inline void nav_roll_rad(const float val) {nav_roll_rad_ = val;}
  inline void nav_hdg_rad(const float val) {
    nav_hdg_rad_.set = true;
    nav_hdg_rad_.val = val;
  }
  inline void nav_gyro_x_radps(const float val) {nav_gyro_x_radps_ = val;}
  inline void nav_gyro_y_radps(const float val) {nav_gyro_y_radps_ = val;}
  inline void nav_gyro_z_radps(const float val) {nav_gyro_z_radps_ = val;}
  /* Effector */
  inline void effector(const std::array<float, 16> &ref) {effector_ = ref;}
  /* Inceptor */
  inline void inceptor(const std::array<float, 16> &ref) {inceptor_ = ref;}
  inline void throttle_ch(const uint8_t val) {throttle_ch_ = val;}

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  static constexpr uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Data */
  template<typename T>
  struct CondData {
    T val = static_cast<T>(0);
    bool set = false;
  };
  /* System */
  uint64_t sys_time_us_ = 0;
  uint32_t frame_time_us_ = 0;
  uint32_t frame_period_us_ = 0;
  bool gyro_installed_ = false;
  bool accel_installed_ = false;
  bool mag_installed_ = false;
  bool static_pres_installed_ = false;
  bool diff_pres_installed_ = false;
  bool gnss_installed_ = false;
  bool inceptor_installed_ = false;
  bool gyro_healthy_ = false;
  bool accel_healthy_ = false;
  bool mag_healthy_ = false;
  bool static_pres_healthy_ = false;
  bool diff_pres_healthy_ = false;
  bool gnss_healthy_ = false;
  bool inceptor_healthy_ = false;
  CondData<float> battery_volt_;
  /* IMU */
  float imu_accel_x_mps2_ = 0.0f;
  float imu_accel_y_mps2_ = 0.0f;
  float imu_accel_z_mps2_ = 0.0f;
  float imu_gyro_x_radps_ = 0.0f;
  float imu_gyro_y_radps_ = 0.0f;
  float imu_gyro_z_radps_ = 0.0f;
  float imu_mag_x_ut_ = 0.0f;
  float imu_mag_y_ut_ = 0.0f;
  float imu_mag_z_ut_ = 0.0f;
  CondData<float> imu_die_temp_c_;
  /* Airdata */
  float static_pres_pa_ = 0;
  float diff_pres_pa_ = 0;
  float static_pres_die_temp_c_ = 0;
  CondData<float> diff_pres_die_temp_c_;
  /* GNSS */
  int8_t gnss_fix_ = GNSS_FIX_NONE;
  double gnss_lat_rad_ = 0.0;
  double gnss_lon_rad_ = 0.0;
  float gnss_alt_msl_m_ = 0.0f;
  float gnss_alt_wgs84_m_ = 0.0f;
  float gnss_horz_acc_m_ = 0.0f;
  float gnss_vert_acc_m_ = 0.0f;
  float gnss_vel_acc_mps_ = 0.0f;
  float gnss_track_acc_rad_ = 0.0f;
  CondData<uint8_t> gnss_num_sv_;
  CondData<float> gnss_hdop_;
  CondData<float> gnss_vdop_;
  CondData<float> gnss_vel_mps_;
  CondData<float> gnss_track_rad_;
  /* Nav */
  double nav_lat_rad_ = 0.0f;
  float nav_lon_rad_ = 0.0f;
  float nav_alt_msl_m_ = 0.0f;
  float nav_alt_agl_m_ = 0.0f;
  float nav_north_pos_m_ = 0.0f;
  float nav_east_pos_m_ = 0.0f;
  float nav_down_pos_m_ = 0.0f;
  float nav_north_vel_mps_ = 0.0f;
  float nav_east_vel_mps_ = 0.0f;
  float nav_down_vel_mps_ = 0.0f;
  float nav_gnd_spd_mps_ = 0.0f;
  float nav_ias_mps_ = 0.0f;
  float nav_pitch_rad_ = 0.0f;
  float nav_roll_rad_ = 0.0f;
  float nav_gyro_x_radps_ = 0.0f;
  float nav_gyro_y_radps_ = 0.0f;
  float nav_gyro_z_radps_ = 0.0f;
  CondData<float> nav_hdg_rad_;
  /* Effector */
  std::array<float, 16> effector_ = {0.0f};
  /* RC Input */
  std::array<float, 16> inceptor_ = {0.0f};
  uint8_t throttle_ch_ = 0;
  /* Telemetry Messages */
  /* SRx_ALL */
  void SRx_ALL();
  /* SRx_EXT_STAT */
  void SRx_EXT_STAT();
  void SendSysStatus();
  void SendBatteryStatus();
  /* SRx_EXTRA1 */
  void SRx_EXTRA1();
  void SendAttitude();
  /* SRx_EXTRA2 */
  void SRx_EXTRA2();
  void SendVfrHud();
  /* SRx_EXTRA3 */
  void SRx_EXTRA3();
  /* SRx_POSITION */
  void SRx_POSITION();
  void SendLocalPositionNed();
  void SendGlobalPositionInt();
  /* SRx_RAW_SENS */
  void SRx_RAW_SENS();
  void SendScaledImu();
  void SendGpsRawInt();
  void SendScaledPressure();
  /* SRx_RC_CHAN */
  void SRx_RC_CHAN();
  void SendServoOutputRaw();
  void SendRcChannels();
  /* SRx_RAW_CTRL */
  void SRx_RAW_CTRL();
  /* SRx_EMPTY */
  void SRx_EMPTY();
  /* Timing */
  static constexpr int16_t NUM_DATA_STREAMS_ = 13;
  int16_t data_stream_period_ms_[NUM_DATA_STREAMS_] = {-1, -1, -1, -1, -1, -1,
                                                       -1, -1, -1, -1, -1, -1,
                                                       -1};
  elapsedMillis data_stream_timer_ms_[NUM_DATA_STREAMS_];
  /* Data streams */
  static constexpr int8_t SRx_RAW_SENS_STREAM = 1;
  static constexpr int8_t SRx_EXT_STAT_STREAM = 2;
  static constexpr int8_t SRx_RC_CHAN_STREAM = 3;
  static constexpr int8_t SRx_RAW_CTRL_STREAM = 4;
  static constexpr int8_t SRx_POSITION_STREAM = 6;
  static constexpr int8_t SRx_EXTRA1_STREAM = 10;
  static constexpr int8_t SRx_EXTRA2_STREAM = 11;
  static constexpr int8_t SRx_EXTRA3_STREAM = 12;
  typedef void (MavLinkTelemetry::*DataStream)(void);
  DataStream streams_[NUM_DATA_STREAMS_] = {
    &MavLinkTelemetry::SRx_ALL,
    &MavLinkTelemetry::SRx_RAW_SENS,
    &MavLinkTelemetry::SRx_EXT_STAT,
    &MavLinkTelemetry::SRx_RC_CHAN,
    &MavLinkTelemetry::SRx_RAW_CTRL,
    &MavLinkTelemetry::SRx_EMPTY,
    &MavLinkTelemetry::SRx_POSITION,
    &MavLinkTelemetry::SRx_EMPTY,
    &MavLinkTelemetry::SRx_EMPTY,
    &MavLinkTelemetry::SRx_EMPTY,
    &MavLinkTelemetry::SRx_EXTRA1,
    &MavLinkTelemetry::SRx_EXTRA2,
    &MavLinkTelemetry::SRx_EXTRA3
  };
  /* Request data stream */
  void ParseRequestDataStream(const mavlink_request_data_stream_t &ref);
  /*** Message data ***/
  mavlink_request_data_stream_t request_stream_;
  uint32_t sys_time_ms_;
  /* System status */
  static constexpr int16_t current_battery_ = -1;
  static constexpr int8_t battery_remaining_ = -1;
  static constexpr uint16_t drop_rate_comm_ = 0;
  static constexpr uint16_t errors_comm_ = 0;
  static constexpr uint16_t errors_count_[4] = {0};
  uint32_t sensors_present_;
  uint32_t sensors_healthy_;
  uint16_t load_;
  uint16_t voltage_battery_ = UINT16_MAX;
  /* GNSS raw */
  static constexpr uint16_t yaw_cdeg_ = 0;
  uint8_t fix_;
  int32_t lat_dege7_;
  int32_t lon_dege7_;
  int32_t alt_msl_mm_;
  uint16_t eph_ = UINT16_MAX, epv_ = UINT16_MAX;
  uint16_t vel_cmps_ = UINT16_MAX;
  uint16_t track_cdeg_ = UINT16_MAX;
  uint8_t num_sv_ = 255;
  int32_t alt_wgs84_mm_;
  uint32_t h_acc_mm_;
  uint32_t v_acc_mm_;
  uint32_t vel_acc_mmps_;
  uint32_t hdg_acc_dege5_;
  /* Attitude */
  float yaw_rad_ = 0;
  /* VFR HUD */
  int16_t hdg_deg_ = 0;
  uint16_t throttle_;
  float climb_mps_;
  /* Battery status */
  static constexpr uint8_t id_ = 0;
  static constexpr uint8_t battery_function_ = MAV_BATTERY_FUNCTION_UNKNOWN;
  static constexpr uint8_t type_ = MAV_BATTERY_TYPE_LIPO;
  static constexpr int16_t temp_ = INT16_MAX;
  static constexpr int16_t current_ = -1;
  static constexpr int32_t current_consumed_ = -1;
  static constexpr int32_t energy_consumed_ = -1;
  static constexpr int32_t time_remaining_ = 0;
  static constexpr uint8_t charge_state_ = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
  static constexpr uint8_t battery_mode_ = MAV_BATTERY_MODE_UNKNOWN;
  static constexpr uint32_t fault_bitmask_ = 0;
  uint16_t volt_[14] = {
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
    UINT16_MAX, UINT16_MAX};
  /* Global position */
  int32_t alt_agl_mm_;
  int16_t vx_cmps_;
  int16_t vy_cmps_;
  int16_t vz_cmps_;
  uint16_t hdg_cdeg_ = UINT16_MAX;
  /* IMU */
  int16_t accel_x_mg_;
  int16_t accel_y_mg_;
  int16_t accel_z_mg_;
  int16_t gyro_x_mradps_;
  int16_t gyro_y_mradps_;
  int16_t gyro_z_mradps_;
  int16_t mag_x_mgauss_;
  int16_t mag_y_mgauss_;
  int16_t mag_z_mgauss_;
  int16_t temp_cc_ = 0;
  /* Scaled pressure */
  float static_pres_hpa_;
  float diff_pres_hpa_;
  int16_t static_temp_cc_;
  int16_t diff_temp_cc_ = 0;
  /* Servo output */
  static constexpr uint8_t port_ = 0;
  uint16_t servo_raw_[16] = {0};
  /* RC input */
  static constexpr uint8_t chancount_ = 16;
  static constexpr uint8_t rssi_ = 255;
  uint16_t chan_[18] = {0};
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_TELEMETRY_H_
