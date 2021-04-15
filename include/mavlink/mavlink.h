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

#ifndef INCLUDE_MAVLINK_MAVLINK_H_
#define INCLUDE_MAVLINK_MAVLINK_H_

#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/heartbeat.h"
#include "mavlink/telemetry.h"
#include "mavlink/parameter.h"
// #include "mavlink/mission.h"
// #include "mavlink/cmd.h"

namespace bfs {

template<std::size_t NPARAM>
class MavLink {
 public:
  MavLink(HardwareSerial *bus, const VehicleType type) :
          bus_(bus), heartbeat_(bus, type), telem_(bus),
          param_(bus) {}
  MavLink(HardwareSerial *bus, const VehicleType type, const uint8_t sys_id) :
          bus_(bus), heartbeat_(bus, type, sys_id), telem_(bus, sys_id),
          param_(bus, sys_id) {}
  void Begin(uint32_t baud) {
    bus_->begin(baud);
  }
  void Update() {
    /* Update child classes */
    heartbeat_.Update();
    telem_.Update();
    param_.Update();
    /* Check for received messages */
    while (bus_->available()) {
      if (mavlink_frame_char(chan_, bus_->read(), &msg_, &status_)
          != MAVLINK_FRAMING_INCOMPLETE) {
        /* Message handling */
        telem_.MsgHandler(msg_);
        param_.MsgHandler(msg_);
      }
    }
  }
  /* Vehicle type, system and component ID getters */
  inline constexpr VehicleType vehicle_type() const {
    return heartbeat_.vehicle_type();
  }
  inline constexpr uint8_t sys_id() const {return heartbeat_.sys_id();}
  inline constexpr uint8_t comp_id() const {return heartbeat_.comp_id();}
  /* 
  * Setters and getters for the throttle enabled flag, vehicle mode,
  * and vehicle state
  */
  inline void throttle_enabled(const bool val) {
    heartbeat_.throttle_enabled(val);
  }
  inline void vehicle_mode(const VehicleMode val) {
    heartbeat_.vehicle_mode(val);
  }
  inline void vehicle_state(const VehicleState val) {
    heartbeat_.vehicle_state(val);
  }
  /* Config data stream rates */
  inline void raw_sens_stream_period_ms(const int val) {
    telem_.raw_sens_stream_period_ms(val);
  }
  inline int raw_sens_stream_period_ms() const {
    return telem_.raw_sens_stream_period_ms();
  }
  inline void ext_status_stream_period_ms(const int val) {
    telem_.ext_status_stream_period_ms(val);
  }
  inline int ext_status_stream_period_ms() const {
    return telem_.ext_status_stream_period_ms();
  }
  inline void rc_chan_stream_period_ms(const int val) {
    telem_.rc_chan_stream_period_ms(val);
  }
  inline int rc_chan_stream_period_ms() const {
    return telem_.rc_chan_stream_period_ms();
  }
  inline void pos_stream_period_ms(const int val) {
    telem_.pos_stream_period_ms(val);
  }
  inline int pos_stream_period_ms() const {
    return telem_.pos_stream_period_ms();
  }
  inline void extra1_stream_period_ms(const int val) {
    telem_.extra1_stream_period_ms(val);
  }
  inline int extra1_stream_period_ms() const {
    return telem_.extra1_stream_period_ms();
  }
  inline void extra2_stream_period_ms(const int val) {
    telem_.extra2_stream_period_ms(val);
  }
  inline int extra2_stream_period_ms() const {
    return telem_.extra2_stream_period_ms();
  }
  inline void extra3_stream_period_ms(const int val) {
    telem_.extra3_stream_period_ms(val);
  }
  inline int extra3_stream_period_ms() const {
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
  /* GNSS data */
  inline void gnss_fix(const GnssFix val) {telem_.gnss_fix(val);}
  inline void gnss_num_sats(const uint8_t val) {telem_.gnss_num_sats(val);}
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
  /* Inceptor */
  inline void inceptor(const std::array<float, 16> &ref) {
    telem_.inceptor(ref);
  }
  inline void throttle_ch(const uint8_t val) {telem_.throttle_ch(val);}
  /* Parameters */
  static constexpr std::size_t params_size() {return NPARAM;}
  inline std::array<float, NPARAM> params() const {return param_.params();}
  inline float param(const int32_t idx) const {return param_.param(idx);}
  inline int32_t updated_param() const {return param_.updated_param();}
  template<std::size_t NCHAR>
  inline void SetParamId(const int32_t idx, char const (&name)[NCHAR]) {param_.SetParamId(idx, name);}
  inline std::string param_id(const int32_t idx) const {return param_.param_id(idx);}

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Child classes */
  MavLinkHeartbeat heartbeat_;
  MavLinkTelemetry telem_;
  MavLinkParameter<NPARAM> param_;
  /* Message buffer */
  mavlink_message_t msg_;
  mavlink_status_t status_;
  uint8_t chan_ = 0;
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_MAVLINK_H_
