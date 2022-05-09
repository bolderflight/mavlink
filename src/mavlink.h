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

#ifndef MAVLINK_SRC_MAVLINK_H_  // NOLINT
#define MAVLINK_SRC_MAVLINK_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <string>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "heartbeat.h"  // NOLINT
#include "telemetry.h"  // NOLINT
#include "parameter.h"  // NOLINT
#include "mission.h"  // NOLINT
#include "rtcm.h"  // NOLINT
#include "utm.h"  // NOLINT

namespace bfs {

template<std::size_t N, std::size_t M>
class MavLink {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {
    bus_ = bus;
    heartbeat_.hardware_serial(bus);
    telem_.hardware_serial(bus);
    param_.hardware_serial(bus);
    mission_.hardware_serial(bus);
    util_.hardware_serial(bus);
    rtcm_.hardware_serial(bus);
    utm_.hardware_serial(bus);
  }
  inline void aircraft_type(const int8_t type) {
    heartbeat_.aircraft_type(type);
  }
  inline void sys_id(const uint8_t sys_id) {
    sys_id_ = sys_id;
    heartbeat_.sys_id(sys_id);
    telem_.sys_id(sys_id);
    param_.sys_id(sys_id);
    mission_.sys_id(sys_id);
    util_.sys_id(sys_id);
    utm_.sys_id(sys_id);
  }
  inline void mission(MissionItem * const mission,
                      const std::size_t mission_size,
                      MissionItem * const temp) {
    mission_.mission(mission, mission_size, temp);
  }
  inline void fence(MissionItem * const fence, const std::size_t fence_size) {
    mission_.fence(fence, fence_size);
    fence_supported_ = true;
  }
  inline void rally(MissionItem * const rally, const std::size_t rally_size) {
    mission_.rally(rally, rally_size);
    rally_supported_ = true;
  }
  void Begin(uint32_t baud) {
    bus_->begin(baud);
  }
  void Update() {
    /* Update child classes */
    heartbeat_.Update();
    telem_.Update();
    param_.Update();
    mission_.Update();
    utm_.Update();
    /* Check for received messages */
    while (bus_->available()) {
      if (mavlink_frame_char(chan_, bus_->read(), &msg_, &status_)
          != MAVLINK_FRAMING_INCOMPLETE) {
        /* Items handled by this class */
        rx_sys_id_ = msg_.sysid;
        rx_comp_id_ = msg_.compid;
        switch (msg_.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_msg_heartbeat_decode(&msg_, &heartbeat_msg_);
            HeartbeatHandler(heartbeat_msg_);
          }
          case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_msg_command_long_decode(&msg_, &cmd_long_);
            CommandLongHandler(cmd_long_);
            break;
          }
        }
        /* Message handling */
        telem_.MsgHandler(msg_);
        param_.MsgHandler(msg_);
        mission_.MsgHandler(msg_);
        rtcm_.MsgHandler(msg_);
        utm_.MsgHandler(msg_);
      }
    }
  }
  /* Vehicle type, system and component ID getters */
  inline int8_t aircraft_type() const {
    return heartbeat_.aircraft_type();
  }
  inline uint8_t sys_id() const {return heartbeat_.sys_id();}
  inline uint8_t comp_id() const {return heartbeat_.comp_id();}
  /* Link established */
  inline void gcs_lost_link_timeout_ms(const int32_t val) {
    gcs_lost_link_timeout_ms_ = val;
  }
  inline int32_t gcs_lost_link_timeout_ms() const {
    return gcs_lost_link_timeout_ms_;
  }
  inline bool gcs_link_established() const {return gcs_link_established_;}
  inline bool gcs_link_lost() const {
    if (gcs_link_established_) {
      return (gcs_link_timer_ms_ > gcs_lost_link_timeout_ms_);
    } else {
      return false;
    }
  }
  /* Pass pointer to GNSS serial to provide RTCM corrections */
  inline void gnss_serial(HardwareSerial *bus) {rtcm_.gnss_serial(bus);}
  /* 
  * Setters and getters for the throttle enabled flag, vehicle mode,
  * and vehicle state
  */
  inline void throttle_enabled(const bool val) {
    heartbeat_.throttle_enabled(val);
  }
  inline void aircraft_mode(const int8_t val) {
    heartbeat_.aircraft_mode(val);
  }
  inline void aircraft_state(const int8_t val) {
    heartbeat_.aircraft_state(val);
  }
  /* Config data stream rates */
  inline void raw_sens_stream_period_ms(const int16_t val) {
    telem_.raw_sens_stream_period_ms(val);
  }
  inline int16_t raw_sens_stream_period_ms() const {
    return telem_.raw_sens_stream_period_ms();
  }
  inline void ext_status_stream_period_ms(const int16_t val) {
    telem_.ext_status_stream_period_ms(val);
  }
  inline int16_t ext_status_stream_period_ms() const {
    return telem_.ext_status_stream_period_ms();
  }
  inline void rc_chan_stream_period_ms(const int16_t val) {
    telem_.rc_chan_stream_period_ms(val);
  }
  inline int16_t rc_chan_stream_period_ms() const {
    return telem_.rc_chan_stream_period_ms();
  }
  inline void pos_stream_period_ms(const int16_t val) {
    telem_.pos_stream_period_ms(val);
  }
  inline int16_t pos_stream_period_ms() const {
    return telem_.pos_stream_period_ms();
  }
  inline void extra1_stream_period_ms(const int16_t val) {
    telem_.extra1_stream_period_ms(val);
  }
  inline int16_t extra1_stream_period_ms() const {
    return telem_.extra1_stream_period_ms();
  }
  inline void extra2_stream_period_ms(const int16_t val) {
    telem_.extra2_stream_period_ms(val);
  }
  inline int16_t extra2_stream_period_ms() const {
    return telem_.extra2_stream_period_ms();
  }
  inline void extra3_stream_period_ms(const int16_t val) {
    telem_.extra3_stream_period_ms(val);
  }
  inline int16_t extra3_stream_period_ms() const {
    return telem_.extra3_stream_period_ms();
  }
  /* System */
  inline void sys_time_us(const uint64_t val) {telem_.sys_time_us(val);}
  inline void cpu_load(uint32_t frame_time_us, uint32_t frame_period_us) {
    telem_.cpu_load(frame_time_us, frame_period_us);
  }
  /* Installed sensors */
  inline void gyro_installed(const bool val) {telem_.gyro_installed(val);}
  inline void accel_installed(const bool val) {telem_.accel_installed(val);}
  inline void mag_installed(const bool val) {telem_.mag_installed(val);}
  inline void static_pres_installed(const bool val) {
    telem_.static_pres_installed(val);
  }
  inline void diff_pres_installed(const bool val) {
    telem_.diff_pres_installed(val);
  }
  inline void gnss_installed(const bool val) {telem_.gnss_installed(val);}
  inline void inceptor_installed(const bool val) {
    telem_.inceptor_installed(val);
  }
  inline void gyro_healthy(const bool val) {telem_.gyro_healthy(val);}
  inline void accel_healthy(const bool val) {telem_.accel_healthy(val);}
  inline void mag_healthy(const bool val) {telem_.mag_healthy(val);}
  inline void static_pres_healthy(const bool val) {
    telem_.static_pres_healthy(val);
  }
  inline void diff_pres_healthy(const bool val) {telem_.diff_pres_healthy(val);}
  inline void gnss_healthy(const bool val) {telem_.gnss_healthy(val);}
  inline void inceptor_healthy(const bool val) {telem_.inceptor_healthy(val);}
  /* Battery */
  inline void battery_volt(const float val) {telem_.battery_volt(val);}
  inline void battery_current_ma(const float val) {
    telem_.battery_current_ma(val);
  }
  inline void battery_consumed_mah(const float val) {
    telem_.battery_consumed_mah(val);
  }
  inline void battery_remaining_prcnt(const float val) {
    telem_.battery_remaining_prcnt(val);
  }
  inline void battery_remaining_time_s(const float val) {
    telem_.battery_remaining_time_s(val);
  }
  /* IMU data */
  inline void imu_accel_x_mps2(const float val) {telem_.imu_accel_x_mps2(val);}
  inline void imu_accel_y_mps2(const float val) {telem_.imu_accel_y_mps2(val);}
  inline void imu_accel_z_mps2(const float val) {telem_.imu_accel_z_mps2(val);}
  inline void imu_gyro_x_radps(const float val) {telem_.imu_gyro_x_radps(val);}
  inline void imu_gyro_y_radps(const float val) {telem_.imu_gyro_y_radps(val);}
  inline void imu_gyro_z_radps(const float val) {telem_.imu_gyro_z_radps(val);}
  inline void imu_mag_x_ut(const float val) {telem_.imu_mag_x_ut(val);}
  inline void imu_mag_y_ut(const float val) {telem_.imu_mag_y_ut(val);}
  inline void imu_mag_z_ut(const float val) {telem_.imu_mag_z_ut(val);}
  inline void imu_die_temp_c(const float val) {telem_.imu_die_temp_c(val);}
  /* Airdata */
  inline void static_pres_pa(const float val) {telem_.static_pres_pa(val);}
  inline void diff_pres_pa(const float val) {telem_.diff_pres_pa(val);}
  inline void static_pres_die_temp_c(const float val) {
    telem_.static_pres_die_temp_c(val);
  }
  inline void diff_pres_die_temp_c(const float val) {
    telem_.diff_pres_die_temp_c(val);
  }
  /* GNSS data */
  inline void gnss_fix(const int8_t val) {telem_.gnss_fix(val);}
  inline void gnss_num_sats(const int8_t val) {telem_.gnss_num_sats(val);}
  inline void gnss_lat_rad(const double val) {telem_.gnss_lat_rad(val);}
  inline void gnss_lon_rad(const double val) {telem_.gnss_lon_rad(val);}
  inline void gnss_alt_msl_m(const float val) {telem_.gnss_alt_msl_m(val);}
  inline void gnss_alt_wgs84_m(const float val) {telem_.gnss_alt_wgs84_m(val);}
  inline void gnss_hdop(const float val) {telem_.gnss_hdop(val);}
  inline void gnss_vdop(const float val) {telem_.gnss_vdop(val);}
  inline void gnss_track_rad(const float val) {telem_.gnss_track_rad(val);}
  inline void gnss_spd_mps(const float val) {telem_.gnss_spd_mps(val);}
  inline void gnss_horz_acc_m(const float val) {telem_.gnss_horz_acc_m(val);}
  inline void gnss_vert_acc_m(const float val) {telem_.gnss_vert_acc_m(val);}
  inline void gnss_vel_acc_mps(const float val) {telem_.gnss_vel_acc_mps(val);}
  inline void gnss_track_acc_rad(const float val) {
    telem_.gnss_track_acc_rad(val);
  }
  /* Estimation data */
  inline void nav_lat_rad(const double val) {telem_.nav_lat_rad(val);}
  inline void nav_lon_rad(const double val) {telem_.nav_lon_rad(val);}
  inline void nav_alt_msl_m(const float val) {telem_.nav_alt_msl_m(val);}
  inline void nav_alt_agl_m(const float val) {telem_.nav_alt_agl_m(val);}
  inline void nav_north_pos_m(const float val) {telem_.nav_north_pos_m(val);}
  inline void nav_east_pos_m(const float val) {telem_.nav_east_pos_m(val);}
  inline void nav_down_pos_m(const float val) {telem_.nav_down_pos_m(val);}
  inline void nav_north_vel_mps(const float val) {
    telem_.nav_north_vel_mps(val);
  }
  inline void nav_east_vel_mps(const float val) {telem_.nav_east_vel_mps(val);}
  inline void nav_down_vel_mps(const float val) {telem_.nav_down_vel_mps(val);}
  inline void nav_gnd_spd_mps(const float val) {telem_.nav_gnd_spd_mps(val);}
  inline void nav_ias_mps(const float val) {telem_.nav_ias_mps(val);}
  inline void nav_pitch_rad(const float val) {telem_.nav_pitch_rad(val);}
  inline void nav_roll_rad(const float val) {telem_.nav_roll_rad(val);}
  inline void nav_hdg_rad(const float val) {telem_.nav_hdg_rad(val);}
  inline void nav_gyro_x_radps(const float val) {telem_.nav_gyro_x_radps(val);}
  inline void nav_gyro_y_radps(const float val) {telem_.nav_gyro_y_radps(val);}
  inline void nav_gyro_z_radps(const float val) {telem_.nav_gyro_z_radps(val);}
  /* Effector */
  inline void effector(const std::array<float, 16> &ref) {
    telem_.effector(ref);
  }
  inline void effector(const std::array<int16_t, 16> &ref) {
    telem_.effector(ref);
  }
  /* Inceptor */
  inline void inceptor(const std::array<float, 16> &ref) {
    telem_.inceptor(ref);
  }
  inline void inceptor(const std::array<int16_t, 16> &ref) {
    telem_.inceptor(ref);
  }
  inline void throttle_ch(const uint8_t val) {telem_.throttle_ch(val);}
  inline void throttle_prcnt(const float val) {
    telem_.throttle_prcnt(val);
  }
  /* Wind */
  inline void wind_north_vel_mps(const float val) {
    telem_.wind_north_vel_mps(val);
  }
  inline void wind_east_vel_mps(const float val) {
    telem_.wind_east_vel_mps(val);
  }
  inline void wind_down_vel_mps(const float val) {
    telem_.wind_down_vel_mps(val);
  }
  inline void wind_var_horz_mps(const float val) {
    telem_.wind_var_horz_mps(val);
  }
  inline void wind_var_vert_mps(const float val) {
    telem_.wind_var_vert_mps(val);
  }
  inline void wind_meas_alt_m(const float val) {telem_.wind_meas_alt_m(val);}
  inline void wind_horz_acc_mps(const float val) {
    telem_.wind_horz_acc_mps(val);
  }
  inline void wind_vert_acc_mps(const float val) {
    telem_.wind_vert_acc_mps(val);
  }
  /* Unix Time */
  inline void unix_time_us(const uint64_t val) {telem_.unix_time_us(val);}
  /* Home position */
  inline void home_lat_rad(const double val) {telem_.home_lat_rad(val);}
  inline void home_lon_rad(const double val) {telem_.home_lon_rad(val);}
  inline void home_alt_m(const float val) {telem_.home_alt_m_(val);}
  inline void SendHomePos() {telem_.SendHomePos();}
  /* RX IMU */
  inline optional<float> imu_accel_x_mps2() {return telem_.imu_accel_x_mps2();}
  inline optional<float> imu_accel_y_mps2() {return telem_.imu_accel_y_mps2();}
  inline optional<float> imu_accel_z_mps2() {return telem_.imu_accel_z_mps2();}
  inline optional<float> imu_gyro_x_radps() {return telem_.imu_gyro_x_radps();}
  inline optional<float> imu_gyro_y_radps() {return telem_.imu_gyro_y_radps();}
  inline optional<float> imu_gyro_z_radps() {return telem_.imu_gyro_z_radps();}
  inline optional<float> imu_mag_x_ut() {return telem_.imu_mag_x_ut();}
  inline optional<float> imu_mag_y_ut() {return telem_.imu_mag_y_ut();}
  inline optional<float> imu_mag_z_ut() {return telem_.imu_mag_z_ut();}
  inline optional<float> imu_die_temp_c() {return telem_.imu_die_temp_c();}
  /* RX air data */
  inline optional<float> static_pres_pa() {return telem_.static_pres_pa();}
  inline optional<float> diff_pres_pa() {return telem_.diff_pres_pa();}
  inline optional<float> static_pres_die_temp_c() {
    return telem_.static_pres_die_temp_c();
  }
  inline optional<float> diff_pres_die_temp_c() {
    return telem_.diff_pres_die_temp_c();
  }
  /* RX GNSS */
  inline optional<int8_t> gnss_fix() {return telem_.gnss_fix();}
  inline optional<int8_t> gnss_num_sats() {return telem_.gnss_num_sats();}
  inline optional<double> gnss_lat_rad() {return telem_.gnss_lat_rad();}
  inline optional<double> gnss_lon_rad() {return telem_.gnss_lon_rad();}
  inline optional<float> gnss_alt_msl_m() {return telem_.gnss_alt_msl_m();}
  inline optional<float> gnss_alt_wgs84_m() {return telem_.gnss_alt_wgs84_m();}
  inline optional<float> gnss_hdop() {return telem_.gnss_hdop();}
  inline optional<float> gnss_vdop() {return telem_.gnss_vdop();}
  inline optional<float> gnss_track_rad() {return telem_.gnss_track_rad();}
  inline optional<float> gnss_spd_mps() {return telem_.gnss_spd_mps();}
  inline optional<float> gnss_horz_acc_m() {return telem_.gnss_horz_acc_m();}
  inline optional<float> gnss_vert_acc_m() {return telem_.gnss_vert_acc_m();}
  inline optional<float> gnss_vel_acc_mps() {return telem_.gnss_vel_acc_mps();}
  inline optional<float> gnss_track_acc_rad() {
    return telem_.gnss_track_acc_rad();
  }
  inline optional<float> gnss_yaw_rad() {return telem_.gnss_yaw_rad();}
  /* RX navigation filter */
  inline optional<double> nav_lat_rad() {return telem_.nav_lat_rad();}
  inline optional<double> nav_lon_rad() {return telem_.nav_lon_rad();}
  inline optional<float> nav_alt_msl_m() {return telem_.nav_alt_msl_m();}
  inline optional<float> nav_alt_agl_m() {return telem_.nav_alt_agl_m();}
  inline optional<float> nav_north_pos_m() {return telem_.nav_north_pos_m();}
  inline optional<float> nav_east_pos_m() {return telem_.nav_east_pos_m();}
  inline optional<float> nav_down_pos_m() {return telem_.nav_down_pos_m();}
  inline optional<float> nav_north_vel_mps() {
    return telem_.nav_north_vel_mps();
  }
  inline optional<float> nav_east_vel_mps() {return telem_.nav_east_vel_mps();}
  inline optional<float> nav_down_vel_mps() {return telem_.nav_down_vel_mps();}
  inline optional<float> nav_gnd_spd_mps() {return telem_.nav_gnd_spd_mps();}
  inline optional<float> nav_ias_mps() {return telem_.nav_ias_mps();}
  inline optional<float> nav_pitch_rad() {return telem_.nav_pitch_rad();}
  inline optional<float> nav_roll_rad() {return telem_.nav_roll_rad();}
  inline optional<float> nav_hdg_rad() {return telem_.nav_hdg_rad();}
  inline optional<float> nav_gyro_x_radps() {return telem_.nav_gyro_x_radps();}
  inline optional<float> nav_gyro_y_radps() {return telem_.nav_gyro_y_radps();}
  inline optional<float> nav_gyro_z_radps() {return telem_.nav_gyro_z_radps();}
  /* Parameters */
  static constexpr std::size_t params_size() {return N;}
  inline void params(const std::array<float, N> &val) {param_.params(val);}
  inline std::array<float, N> params() const {return param_.params();}
  inline void param(const int32_t i, const float val) {param_.param(i, val);}
  inline float param(const int32_t idx) const {return param_.param(idx);}
  inline int32_t updated_param() {return param_.updated_param();}
  template<std::size_t NCHAR>
  inline void param_id(const int32_t idx, char const (&name)[NCHAR]) {
    param_.param_id(idx, name);
  }
  inline std::string param_id(const int32_t idx) const {
    return param_.param_id(idx);
  }
  /* Mission */
  inline bool mission_updated() {return mission_.mission_updated();}
  inline int32_t active_mission_item() const {
    return mission_.active_mission_item();
  }
  inline std::size_t num_mission_items() const {
    return mission_.num_mission_items();
  }
  void AdvanceMissionItem() {mission_.AdvanceMissionItem();}
  /* Fence */
  inline bool fence_updated() {return mission_.fence_updated();}
  inline std::size_t num_fence_items() const {
    return mission_.num_fence_items();
  }
  /* Rally */
  inline bool rally_points_updated() {
    return mission_.rally_points_updated();
  }
  inline std::size_t num_rally_points() const {
    return mission_.num_rally_points();
  }
  /* Status text */
  template<std::size_t NCHAR>
  void SendStatusText(Severity severity, const char(&msg)[NCHAR]) {
    util_.SendStatusText(severity, msg);
  }
  /* UTM */
  static constexpr std::size_t UTM_UAS_ID_LEN = 18;
  inline void utm_update_period_s(const float val) {utm_.update_period_s(val);}
  inline void utm_unix_time_us(const uint64_t val) {utm_.unix_time_us(val);}
  inline void utm_uas_id(const std::array<uint8_t, UTM_UAS_ID_LEN> &id) {
    utm_.uas_id(id);
  }
  inline void utm_lat_rad(const double val) {utm_.lat_rad(val);}
  inline void utm_lon_rad(const double val) {utm_.lon_rad(val);}
  inline void utm_alt_wgs84_m(const float val) {utm_.alt_wgs84_m(val);}
  inline void utm_rel_alt_m(const float val) {utm_.rel_alt_m(val);}
  inline void utm_north_vel_mps(const float val) {utm_.north_vel_mps(val);}
  inline void utm_east_vel_mps(const float val) {utm_.east_vel_mps(val);}
  inline void utm_down_vel_mps(const float val) {utm_.down_vel_mps(val);}
  inline void utm_horz_acc_m(const float val) {utm_.horz_acc_m(val);}
  inline void utm_vert_acc_m(const float val) {utm_.vert_acc_m(val);}
  inline void utm_vel_acc_mps(const float val) {utm_.vel_acc_mps(val);}
  inline void utm_next_lat_rad(const double val) {utm_.next_lat_rad(val);}
  inline void utm_next_lon_rad(const double val) {utm_.nex_lon_rad(val);}
  inline void utm_next_alt_wgs84_m(const float val) {
    utm_.next_alt_wgs84_m(val);
  }
  inline void utm_flight_state(const FlightState val) {utm_.flight_state(val);}
  inline std::size_t utm_num_rx() const {return utm_.num_rx();}
  inline void UtmClearRx() {utm_.ClearRx();}
  inline optional<uint64_t> utm_unix_time_us(const std::size_t idx) const {
    return utm_.unix_time_us(idx);
  }
  inline optional<std::array<uint8_t, UTM_UAS_ID_LEN>> utm_uas_id(
                                                  const std::size_t idx) const {
    return utm_.uas_id(idx);
  }
  inline optional<double> utm_lat_rad(const std::size_t idx) const {
    return utm_.lat_rad(idx);
  }
  inline optional<double> utm_lon_rad(const std::size_t idx) const {
    return utm_.lon_rad(idx);
  }
  inline optional<float> utm_alt_wgs84_m(const std::size_t idx) const {
    return utm_.alt_wgs84_m(idx);
  }
  inline optional<float> utm_rel_alt_m(const std::size_t idx) const {
    return utm_.rel_alt_m(idx);
  }
  inline optional<float> utm_north_vel_mps(const std::size_t idx) const {
    return utm_.north_vel_mps(idx);
  }
  inline optional<float> utm_east_vel_mps(const std::size_t idx) const {
    return utm_.east_vel_mps(idx);
  }
  inline optional<float> utm_down_vel_mps(const std::size_t idx) const {
    return utm_.down_vel_mps(idx);
  }
  inline optional<float> utm_horz_acc_m(const std::size_t idx) const {
    return utm_.horz_acc_m(idx);
  }
  inline optional<float> utm_vert_acc_m(const std::size_t idx) const {
    return utm_.vert_acc_m(idx);
  }
  inline optional<float> utm_vel_acc_mps(const std::size_t idx) const {
    return utm_.vel_acc_mps(idx);
  }
  inline optional<double> utm_next_lat_rad(const std::size_t idx) const {
    return utm_.next_lat_rad(idx);
  }
  inline optional<double> utm_next_lon_rad(const std::size_t idx) const {
    return utm_.next_lon_rad(idx);
  }
  inline optional<float> utm_next_alt_wgs84_m(const std::size_t idx) const {
    return utm_.next_alt_wgs84_m(idx);
  }
  inline float utm_update_period_s(const std::size_t idx) const {
    return utm_.update_period_s(idx);
  }
  inline FlightState utm_flight_state(const std::size_t idx) const {
    return utm_.utm_flight_state(idx);
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  bool fence_supported_ = false;
  bool rally_supported_ = false;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  mavlink_status_t status_;
  uint8_t chan_ = 0;
  /* Child classes */
  MavLinkUtil util_;
  MavLinkHeartbeat heartbeat_;
  MavLinkTelemetry telem_;
  MavLinkParameter<N> param_;
  MavLinkMission mission_;
  MavLinkRtcm rtcm_;
  MavLinkUtm<M> utm_;
  /* Message handlers */
  uint8_t rx_sys_id_, rx_comp_id_;
  uint16_t cmd_;
  /* Message data */
  mavlink_command_long_t cmd_long_;
  uint32_t flight_sw_version_ = 0;
  uint32_t middleware_sw_version_ = 0;
  uint32_t os_sw_version_ = 0;
  uint32_t board_version_ = 0;
  uint8_t flight_custom_version_[8];
  uint8_t middleware_custom_version_[8];
  uint8_t os_custom_version_[8];
  uint16_t vendor_id_ = 0;
  uint16_t product_id_ = 0;
  uint64_t uid_ = 0;
  uint8_t uid2_[18];
  uint8_t spec_version_hash_[8];
  uint8_t library_versions_hash_[8];
  uint16_t version_ = 230;
  uint16_t min_version_ = 100;
  uint64_t capabilities_;
  static constexpr int32_t result_param2_ = 0;
  /* GCS link status */
  mavlink_heartbeat_t heartbeat_msg_;
  bool gcs_link_latch_ = false;
  bool gcs_link_established_ = false;
  elapsedMillis gcs_link_timer_ms_;
  int32_t gcs_lost_link_timeout_ms_ = 5000;
  void HeartbeatHandler(const mavlink_heartbeat_t &ref) {
    if (ref.type == MAV_TYPE_GCS) {
      /* Established contact with GCS */
      if (!gcs_link_latch_) {
        gcs_link_established_ = true;
        gcs_link_latch_ = true;
      }
      /* Reset timer */
      gcs_link_timer_ms_ = 0;
    }
  }
  void CommandLongHandler(const mavlink_command_long_t &ref) {
    if ((cmd_long_.target_system == sys_id_) &&
        (cmd_long_.target_component == comp_id_)) {
      cmd_ = cmd_long_.command;
      switch (cmd_long_.command) {
        case MAV_CMD_REQUEST_PROTOCOL_VERSION: {
          SendCmdAck(MAV_RESULT_ACCEPTED, 255);
          SendProtocolVersion();
          break;
        }
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
          SendCmdAck(MAV_RESULT_ACCEPTED, 255);
          SendAutopilotVersion();
          break;
        }
      }
    }
  }
  /* Emitters */
  void SendProtocolVersion() {
    msg_len_ = mavlink_msg_protocol_version_pack(sys_id_, comp_id_, &msg_,
                                                 version_, min_version_,
                                                 version_,
                                                 spec_version_hash_,
                                                 library_versions_hash_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }
  void SendAutopilotVersion() {
    capabilities_ =
      MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
      MAV_PROTOCOL_CAPABILITY_MISSION_INT |
      MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    if (fence_supported_) {
      capabilities_ |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
    }
    if (rally_supported_) {
      capabilities_ |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
    }
    msg_len_ = mavlink_msg_autopilot_version_pack(sys_id_, comp_id_, &msg_,
                                                  capabilities_,
                                                  flight_sw_version_,
                                                  middleware_sw_version_,
                                                  os_sw_version_,
                                                  board_version_,
                                                  flight_custom_version_,
                                                  middleware_custom_version_,
                                                  os_custom_version_,
                                                  vendor_id_, product_id_,
                                                  uid_, uid2_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }
  void SendCmdAck(const uint8_t result, const uint8_t progress) {
    msg_len_ = mavlink_msg_command_ack_pack(sys_id_, comp_id_, &msg_,
                                            cmd_, result, progress,
                                            result_param2_, rx_sys_id_,
                                            rx_comp_id_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }
};

}  // namespace bfs

#endif  // MAVLINK_SRC_MAVLINK_H_ NOLINT
