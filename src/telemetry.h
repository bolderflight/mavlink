/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#ifndef MAVLINK_SRC_TELEMETRY_H_  // NOLINT
#define MAVLINK_SRC_TELEMETRY_H_

#if defined(ARDUINO)
#include "Arduino.h"
#include "optional.hpp"
#else
#include <optional>
#include "core/core.h"
#endif
#include <array>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"


namespace bfs {

#if defined(ARDUINO)
using nonstd::optional;
#else
using std::optional;
#endif

enum GnssFix : int8_t {
  GNSS_FIX_NONE = 1,
  GNSS_FIX_2D = 2,
  GNSS_FIX_3D = 3,
  GNSS_FIX_DGNSS = 4,
  GNSS_FIX_RTK_FLOAT = 5,
  GNSS_FIX_RTK_FIXED = 6
};

class MavLinkTelemetry {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void sys_id(const uint8_t sys_id) {sys_id_ = sys_id;}
  inline void comp_id(const uint8_t comp_id) {comp_id_ = comp_id;}
  /* Update and message handler methods */
  void Update();
  void MsgHandler(const mavlink_message_t &ref);
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
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
  inline void battery_current_ma(const float val) {
    battery_current_ma_.set = true;
    battery_current_ma_.val = val;
  }
  inline void battery_consumed_mah(const float val) {
    battery_consumed_mah_.set = true;
    battery_consumed_mah_.val = val;
  }
  inline void battery_remaining_prcnt(const float val) {
    battery_remaining_prcnt_.set = true;
    battery_remaining_prcnt_.val = val;
  }
  inline void battery_remaining_time_s(const float val) {
    battery_remaining_time_s_.set = true;
    battery_remaining_time_s_.val = val;
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
  inline void gnss_num_sats(const int8_t val) {
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
  inline void effector(const std::array<int16_t, 16> &ref) {
    use_raw_effector_ = true;
    effector_raw_ = ref;
  }
  /* Inceptor */
  inline void inceptor(const std::array<float, 16> &ref) {inceptor_ = ref;}
  inline void inceptor(const std::array<int16_t, 16> &ref) {
    use_raw_inceptor_ = true;
    inceptor_raw_ = ref;
  }
  inline void throttle_ch(const int8_t val) {throttle_ch_ = val;}
  inline void throttle_prcnt(const float val) {
    use_throttle_prcnt_ = true;
    throttle_prcnt_ = val;
  }
  /* Wind Covariance */
  inline void wind_north_vel_mps(const float val) {wind_x_mps_ = val;}
  inline void wind_east_vel_mps(const float val) {wind_y_mps_ = val;}
  inline void wind_down_vel_mps(const float val) {wind_z_mps_ = val;}
  inline void wind_var_horz_mps(const float val) {wind_var_horz_mps_ = val;}
  inline void wind_var_vert_mps(const float val) {wind_var_vert_mps_ = val;}
  inline void wind_meas_alt_m(const float val) {wind_alt_m_ = val;}
  inline void wind_horz_acc_mps(const float val) {wind_horz_acc_mps_ = val;}
  inline void wind_vert_acc_mps(const float val) {wind_vert_acc_mps_ = val;}
  /* Unix time */
  inline void unix_time_us(const uint64_t val) {unix_time_us_ = val;}
  /* Home position */
  inline void home_lat_rad(const double val) {home_lat_rad_ = val;}
  inline void home_lon_rad(const double val) {home_lon_rad_ = val;}
  inline void home_alt_m(const float val) {home_alt_m_ = val;}
  void SendHomePos();
  /* Receive home position */
  inline optional<double> home_lat_rad() {
    optional<double> ret = rx_home_lat_rad_;
    rx_home_lat_rad_.reset();
    return ret;
  }
  inline optional<double> home_lon_rad() {
    optional<double> ret = rx_home_lon_rad_;
    rx_home_lon_rad_.reset();
    return ret;
  }
  inline optional<float> home_alt_m() {
    optional<float> ret = rx_home_alt_m_;
    rx_home_alt_m_.reset();
    return ret;
  }
  /* Receive IMU data */
  inline optional<float> imu_accel_x_mps2() {
    optional<float> ret = rx_imu_accel_x_mps2_;
    rx_imu_accel_x_mps2_.reset();
    return ret;
  }
  inline optional<float> imu_accel_y_mps2() {
    optional<float> ret = rx_imu_accel_y_mps2_;
    rx_imu_accel_y_mps2_.reset();
    return ret;
  }
  inline optional<float> imu_accel_z_mps2() {
    optional<float> ret = rx_imu_accel_z_mps2_;
    rx_imu_accel_z_mps2_.reset();
    return ret;
  }
  inline optional<float> imu_gyro_x_radps() {
    optional<float> ret = rx_imu_gyro_x_radps_;
    rx_imu_gyro_x_radps_.reset();
    return ret;
  }
  inline optional<float> imu_gyro_y_radps() {
    optional<float> ret = rx_imu_gyro_y_radps_;
    rx_imu_gyro_y_radps_.reset();
    return ret;
  }
  inline optional<float> imu_gyro_z_radps() {
    optional<float> ret = rx_imu_gyro_z_radps_;
    rx_imu_gyro_z_radps_.reset();
    return ret;
  }
  inline optional<float> imu_mag_x_ut() {
    optional<float> ret = rx_imu_mag_x_ut_;
    rx_imu_mag_x_ut_.reset();
    return ret;
  }
  inline optional<float> imu_mag_y_ut() {
    optional<float> ret = rx_imu_mag_y_ut_;
    rx_imu_mag_y_ut_.reset();
    return ret;
  }
  inline optional<float> imu_mag_z_ut() {
    optional<float> ret = rx_imu_mag_z_ut_;
    rx_imu_mag_z_ut_.reset();
    return ret;
  }
  inline optional<float> imu_die_temp_c() {
    optional<float> ret = rx_imu_die_temp_c_;
    rx_imu_die_temp_c_.reset();
    return ret;
  }
  /* Air data */
  inline optional<float> static_pres_pa() {
    optional<float> ret = rx_static_pres_pa_;
    rx_static_pres_pa_.reset();
    return ret;
  }
  inline optional<float> diff_pres_pa() {
    optional<float> ret = rx_diff_pres_pa_;
    rx_diff_pres_pa_.reset();
    return ret;
  }
  inline optional<float> static_pres_die_temp_c() {
    optional<float> ret = rx_static_pres_die_temp_c_;
    rx_static_pres_die_temp_c_.reset();
    return ret;
  }
  inline optional<float> diff_pres_die_temp_c() {
    optional<float> ret = rx_diff_pres_die_temp_c_;
    rx_diff_pres_die_temp_c_.reset();
    return ret;
  }
  /* GNSS data */
  inline optional<int8_t> gnss_fix() {
    optional<int8_t> ret = rx_gnss_fix_;
    rx_gnss_fix_.reset();
    return ret;
  }
  inline optional<int8_t> gnss_num_sats() {
    optional<int8_t> ret = rx_gnss_num_sats_;
    rx_gnss_num_sats_.reset();
    return ret;
  }
  inline optional<double> gnss_lat_rad() {
    optional<double> ret = rx_gnss_lat_rad_;
    rx_gnss_lat_rad_.reset();
    return ret;
  }
  inline optional<double> gnss_lon_rad() {
    optional<double> ret = rx_gnss_lon_rad_;
    rx_gnss_lon_rad_.reset();
    return ret;
  }
  inline optional<float> gnss_alt_msl_m() {
    optional<float> ret = rx_gnss_alt_msl_m_;
    rx_gnss_alt_msl_m_.reset();
    return ret;
  }
  inline optional<float> gnss_alt_wgs84_m() {
    optional<float> ret = rx_gnss_alt_wgs84_m_;
    rx_gnss_alt_wgs84_m_.reset();
    return ret;
  }
  inline optional<float> gnss_hdop() {
    optional<float> ret = rx_gnss_hdop_;
    rx_gnss_hdop_.reset();
    return ret;
  }
  inline optional<float> gnss_vdop() {
    optional<float> ret = rx_gnss_vdop_;
    rx_gnss_vdop_.reset();
    return ret;
  }
  inline optional<float> gnss_track_rad() {
    optional<float> ret = rx_gnss_track_rad_;
    rx_gnss_track_rad_.reset();
    return ret;
  }
  inline optional<float> gnss_spd_mps() {
    optional<float> ret = rx_gnss_spd_mps_;
    rx_gnss_spd_mps_.reset();
    return ret;
  }
  inline optional<float> gnss_horz_acc_m() {
    optional<float> ret = rx_gnss_horz_acc_m_;
    rx_gnss_horz_acc_m_.reset();
    return ret;
  }
  inline optional<float> gnss_vert_acc_m() {
    optional<float> ret = rx_gnss_vert_acc_m_;
    rx_gnss_vert_acc_m_.reset();
    return ret;
  }
  inline optional<float> gnss_vel_acc_mps() {
    optional<float> ret = rx_gnss_vel_acc_mps_;
    rx_gnss_vel_acc_mps_.reset();
    return ret;
  }
  inline optional<float> gnss_track_acc_rad() {
    optional<float> ret = rx_gnss_track_acc_rad_;
    rx_gnss_track_acc_rad_.reset();
    return ret;
  }
  inline optional<float> gnss_yaw_rad() {
    optional<float> ret = rx_gnss_yaw_;
    rx_gnss_yaw_.reset();
    return ret;
  }
  /* Navigation filter data */
  inline optional<double> nav_lat_rad() {
    optional<double> ret = rx_nav_lat_rad_;
    rx_nav_lat_rad_.reset();
    return ret;
  }
  inline optional<double> nav_lon_rad() {
    optional<double> ret = rx_nav_lon_rad_;
    rx_nav_lon_rad_.reset();
    return ret;
  }
  inline optional<float> nav_alt_msl_m() {
    optional<float> ret = rx_nav_alt_msl_m_;
    rx_nav_alt_msl_m_.reset();
    return ret;
  }
  inline optional<float> nav_alt_agl_m() {
    optional<float> ret = rx_nav_alt_agl_m_;
    rx_nav_alt_agl_m_.reset();
    return ret;
  }
  inline optional<float> nav_north_pos_m() {
    optional<float> ret = rx_nav_north_pos_m_;
    rx_nav_north_pos_m_.reset();
    return ret;
  }
  inline optional<float> nav_east_pos_m() {
    optional<float> ret = rx_nav_east_pos_m_;
    rx_nav_east_pos_m_.reset();
    return ret;
  }
  inline optional<float> nav_down_pos_m() {
    optional<float> ret = rx_nav_down_pos_m_;
    rx_nav_down_pos_m_.reset();
    return ret;
  }
  inline optional<float> nav_north_vel_mps() {
    optional<float> ret = rx_nav_north_vel_mps_;
    rx_nav_north_vel_mps_.reset();
    return ret;
  }
  inline optional<float> nav_east_vel_mps() {
    optional<float> ret = rx_nav_east_vel_mps_;
    rx_nav_east_vel_mps_.reset();
    return ret;
  }
  inline optional<float> nav_down_vel_mps() {
    optional<float> ret = rx_nav_down_vel_mps_;
    rx_nav_down_vel_mps_.reset();
    return ret;
  }
  inline optional<float> nav_gnd_spd_mps() {
    optional<float> ret = rx_nav_gnd_spd_mps_;
    rx_nav_gnd_spd_mps_.reset();
    return ret;
  }
  inline optional<float> nav_ias_mps() {
    optional<float> ret = rx_nav_ias_mps_;
    rx_nav_ias_mps_.reset();
    return ret;
  }
  inline optional<float> nav_pitch_rad() {
    optional<float> ret = rx_nav_pitch_rad;
    rx_nav_pitch_rad.reset();
    return ret;
  }
  inline optional<float> nav_roll_rad() {
    optional<float> ret = rx_nav_roll_rad;
    rx_nav_roll_rad.reset();
    return ret;
  }
  inline optional<float> nav_hdg_rad() {
    optional<float> ret = rx_nav_hdg_rad;
    rx_nav_hdg_rad.reset();
    return ret;
  }
  inline optional<float> nav_gyro_x_radps() {
    optional<float> ret = rx_nav_gyro_x_radps;
    rx_nav_gyro_x_radps.reset();
    return ret;
  }
  inline optional<float> nav_gyro_y_radps() {
    optional<float> ret = rx_nav_gyro_y_radps;
    rx_nav_gyro_y_radps.reset();
    return ret;
  }
  inline optional<float> nav_gyro_z_radps() {
    optional<float> ret = rx_nav_gyro_z_radps;
    rx_nav_gyro_z_radps.reset();
    return ret;
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
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
  /* Home */
  double home_lat_rad_, home_lon_rad_;
  float home_alt_m_;
  mavlink_home_position_t home_pos_;
  optional<double> rx_home_lat_rad_, rx_home_lon_rad_;
  optional<float> rx_home_alt_m_;
  void ParseHomePosition(const mavlink_home_position_t &ref);
  /* System */
  uint64_t sys_time_us_ = 0;
  uint64_t unix_time_us_ = 0;
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
  CondData<float> battery_current_ma_;
  CondData<float> battery_consumed_mah_;
  CondData<float> battery_remaining_prcnt_;
  CondData<float> battery_remaining_time_s_;
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
  CondData<int8_t> gnss_num_sv_;
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
  std::array<float, 16> effector_;
  bool use_raw_effector_ = false;
  std::array<int16_t, 16> effector_raw_;
  /* RC Input */
  std::array<float, 16> inceptor_;
  bool use_raw_inceptor_ = false;
  std::array<int16_t, 16> inceptor_raw_;
  int8_t throttle_ch_ = 0;
  float throttle_prcnt_;
  bool use_throttle_prcnt_ = false;
  /* Wind */
  float wind_x_mps_ = 0.0f;
  float wind_y_mps_ = 0.0f;
  float wind_z_mps_ = 0.0f;
  float wind_var_horz_mps_ = 0.0f;
  float wind_var_vert_mps_ = 0.0f;
  float wind_alt_m_ = 0.0f;
  float wind_horz_acc_mps_ = 0.0f;
  float wind_vert_acc_mps_ = 0.0f;
  /* RX Data - IMU */
  mavlink_scaled_imu_t scaled_imu_;
  optional<float> rx_imu_accel_x_mps2_;
  optional<float> rx_imu_accel_y_mps2_;
  optional<float> rx_imu_accel_z_mps2_;
  optional<float> rx_imu_gyro_x_radps_;
  optional<float> rx_imu_gyro_y_radps_;
  optional<float> rx_imu_gyro_z_radps_;
  optional<float> rx_imu_mag_x_ut_;
  optional<float> rx_imu_mag_y_ut_;
  optional<float> rx_imu_mag_z_ut_;
  optional<float> rx_imu_die_temp_c_;
  void ParseScaledImu(const mavlink_scaled_imu_t &ref);
  /* RX Data - air data */
  mavlink_scaled_pressure_t scaled_pres_;
  optional<float> rx_static_pres_pa_;
  optional<float> rx_diff_pres_pa_;
  optional<float> rx_static_pres_die_temp_c_;
  optional<float> rx_diff_pres_die_temp_c_;
  void ParseScaledPres(const mavlink_scaled_pressure_t &ref);
  /* RX Data - GNSS */
  mavlink_gps_raw_int_t gps_raw_int_;
  optional<int8_t> rx_gnss_fix_;
  optional<int8_t> rx_gnss_num_sats_;
  optional<double> rx_gnss_lat_rad_;
  optional<double> rx_gnss_lon_rad_;
  optional<float> rx_gnss_alt_msl_m_;
  optional<float> rx_gnss_alt_wgs84_m_;
  optional<float> rx_gnss_hdop_;
  optional<float> rx_gnss_vdop_;
  optional<float> rx_gnss_track_rad_;
  optional<float> rx_gnss_spd_mps_;
  optional<float> rx_gnss_horz_acc_m_;
  optional<float> rx_gnss_vert_acc_m_;
  optional<float> rx_gnss_vel_acc_mps_;
  optional<float> rx_gnss_track_acc_rad_;
  optional<float> rx_gnss_yaw_;
  void ParseGpsRawInt(const mavlink_gps_raw_int_t &ref);
  /* RX Data - navigation filter */
  mavlink_attitude_t attitude_;
  optional<float> rx_nav_pitch_rad;
  optional<float> rx_nav_roll_rad;
  optional<float> rx_nav_hdg_rad;
  optional<float> rx_nav_gyro_x_radps;
  optional<float> rx_nav_gyro_y_radps;
  optional<float> rx_nav_gyro_z_radps;
  void ParseAttitude(const mavlink_attitude_t &ref);
  mavlink_vfr_hud_t vfr_hud_;
  optional<float> rx_nav_ias_mps_;
  optional<float> rx_nav_gnd_spd_mps_;
  void ParseVfrHud(const mavlink_vfr_hud_t &ref);
  mavlink_local_position_ned_t local_pos_ned_;
  optional<float> rx_nav_north_pos_m_;
  optional<float> rx_nav_east_pos_m_;
  optional<float> rx_nav_down_pos_m_;
  void ParseLocalPosNed(const mavlink_local_position_ned_t &ref);
  mavlink_global_position_int_t global_pos_int_;
  optional<double> rx_nav_lat_rad_;
  optional<double> rx_nav_lon_rad_;
  optional<float> rx_nav_alt_msl_m_;
  optional<float> rx_nav_alt_agl_m_;
  optional<float> rx_nav_north_vel_mps_;
  optional<float> rx_nav_east_vel_mps_;
  optional<float> rx_nav_down_vel_mps_;
  void ParseGlobalPosInt(const mavlink_global_position_int_t &ref);
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
  void SendWindCov();
  void SendSystemTime();
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
  static constexpr uint16_t drop_rate_comm_ = 0;
  static constexpr uint16_t errors_comm_ = 0;
  static constexpr uint16_t errors_count_[4] = {0};
  static constexpr uint32_t onboard_control_sensors_present_ext_ = 0;
  uint32_t sensors_present_;
  uint32_t sensors_healthy_;
  uint16_t load_;
  uint16_t voltage_battery_ = UINT16_MAX;
  int16_t current_battery_ = -1;
  int8_t battery_remaining_ = -1;
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
  static constexpr int32_t energy_consumed_ = -1;
  static constexpr uint8_t charge_state_ = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
  static constexpr uint8_t battery_mode_ = MAV_BATTERY_MODE_UNKNOWN;
  static constexpr uint32_t fault_bitmask_ = 0;
  uint16_t volt_[14] = {
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
    UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
    UINT16_MAX, UINT16_MAX};
  int16_t current_ = -1;
  int32_t current_consumed_ = -1;
  int32_t time_remaining_ = 0;
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

#endif  // MAVLINK_SRC_TELEMETRY_H_ NOLINT
