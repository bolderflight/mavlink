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
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/heartbeat.h"
#include "mavlink/telemetry.h"
#include "mavlink/parameter.h"
#include "mavlink/mission.h"
#include "mavlink/cmd.h"

namespace bfs {

template<std::size_t PARAM_N, std::size_t MISSION_N, std::size_t FENCE_N, std::size_t RALLY_N>
class MavLink {
 public:
  MavLink(HardwareSerial *bus, const VehicleType type) :
          bus_(bus), heartbeat_(bus, type), telem_(bus),
          param_(bus), mission_(bus) {}
  MavLink(HardwareSerial *bus, const VehicleType type, const uint8_t sys_id) :
          bus_(bus), heartbeat_(bus, type, sys_id), telem_(bus, sys_id),
          param_(bus, sys_id), mission_(bus, sys_id) {}
  void Begin(uint32_t baud) {
    bus_->begin(baud);
  }
  void Update() {
    /* Update child classes */
    heartbeat_.Update();
    telem_.Update();
    param_.Update();
    mission_.Update();
    /* Check for received messages */
    while (bus_->available()) {
      if (mavlink_frame_char(chan_, bus_->read(), &msg_, &status_) != MAVLINK_FRAMING_INCOMPLETE) {
        /* Message handling */
        telem_.MsgHandler(msg_);
        param_.MsgHandler(msg_);
        mission_.MsgHandler(msg_);
      }
    }
  }
  /* Config */
  inline void SetExtStatusPeriod_ms(int val) {telem_.SetExtStatusPeriod_ms(val);}
  inline void SetExtra1Period_ms(int val) {telem_.SetExtra1Period_ms(val);}
  inline void SetExtra2Period_ms(int val) {telem_.SetExtra2Period_ms(val);}
  inline void SetExtra3Period_ms(int val) {telem_.SetExtra3Period_ms(val);}
  inline void SetPosPeriod_ms(int val) {telem_.SetPosPeriod_ms(val);}
  inline void SetRawSensPeriod_ms(int val) {telem_.SetRawSensPeriod_ms(val);}
  inline void SetRcChanPeriod_ms(int val) {telem_.SetRcChanPeriod_ms(val);}
  /*** Getters ***/
  inline constexpr VehicleType vehicle_type() const {return heartbeat_.vehicle_type();}
  inline constexpr uint8_t sys_id() const {return heartbeat_.sys_id();}
  inline constexpr uint8_t comp_id() const {return heartbeat_.comp_id();}
  inline bool throttle_enabled() const {return heartbeat_.throttle_enabled();}
  inline VehicleMode vehicle_mode() const {return heartbeat_.vehicle_mode();}
  inline VehicleState vehicle_state() const {return heartbeat_.vehicle_state();}
 /*** Setters ***/
  /* System */
  inline void throttle_enabled(const bool val) {heartbeat_.throttle_enabled(val);}
  inline void vehicle_mode(const VehicleMode val) {heartbeat_.vehicle_mode(val);}
  inline void vehicle_state(const VehicleState val) {heartbeat_.vehicle_state(val);}
  inline void sys_time_s(const double val) {telem_.sys_time_s(val);}
  inline void cpu_load(const float val) {telem_.cpu_load(val);}
  inline void diff_pres_installed(const bool val) {telem_.diff_pres_installed(val);}
  inline void imu_healthy(const bool val) {telem_.imu_healthy(val);}
  inline void static_pres_healthy(const bool val) {telem_.static_pres_healthy(val);}
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
  inline void static_pres_die_temp_c(const float val) {telem_.static_pres_die_temp_c(val);}
  /* GNSS data */
  inline void gnss_fix(const GnssFix val) {telem_.gnss_fix(val);}
  inline void gnss_num_sats(const uint8_t val) {telem_.gnss_num_sats(val);}
  inline void gnss_lat_rad(const double val) {telem_.gnss_lat_rad(val);}
  inline void gnss_lon_rad(const double val) {telem_.gnss_lon_rad(val);}
  inline void gnss_alt_msl_m(const float val) {telem_.gnss_alt_msl_m(val);}
  inline void gnss_alt_wgs84_m(const float val) {telem_.gnss_alt_wgs84_m(val);}
  inline void gnss_north_vel_mps(const float val) {telem_.gnss_north_vel_mps(val);}
  inline void gnss_east_vel_mps(const float val) {telem_.gnss_east_vel_mps(val);}
  inline void gnss_down_vel_mps(const float val) {telem_.gnss_down_vel_mps(val);}
  inline void gnss_horz_acc_m(const float val) {telem_.gnss_horz_acc_m(val);}
  inline void gnss_vert_acc_m(const float val) {telem_.gnss_vert_acc_m(val);}
  inline void gnss_vel_acc_mps(const float val) {telem_.gnss_vel_acc_mps(val);}
  /* Estimation data */
  inline void nav_lat_rad(const double val) {telem_.nav_lat_rad(val);}
  inline void nav_lon_rad(const double val) {telem_.nav_lon_rad(val);}
  inline void nav_alt_msl_m(const float val) {telem_.nav_alt_msl_m(val);}
  inline void nav_alt_agl_m(const float val) {telem_.nav_alt_agl_m(val);}
  inline void nav_north_vel_mps(const float val) {telem_.nav_north_vel_mps(val);}
  inline void nav_east_vel_mps(const float val) {telem_.nav_east_vel_mps(val);}
  inline void nav_down_vel_mps(const float val) {telem_.nav_down_vel_mps(val);}
  inline void nav_ias_mps(const float val) {telem_.nav_ias_mps(val);}
  inline void nav_pitch_rad(const float val) {telem_.nav_pitch_rad(val);}
  inline void nav_roll_rad(const float val) {telem_.nav_roll_rad(val);}
  inline void nav_hdg_rad(const float val) {telem_.nav_hdg_rad(val);}
  inline void nav_gyro_x_radps(const float val) {telem_.nav_gyro_x_radps(val);}
  inline void nav_gyro_y_radps(const float val) {telem_.nav_gyro_y_radps(val);}
  inline void nav_gyro_z_radps(const float val) {telem_.nav_gyro_z_radps(val);}
  /* Effector */
  inline void effector(const std::array<float, 16> &ref) {telem_.effector(ref);}
  /* Inceptor */
  inline void inceptor(const std::array<float, 16> &ref) {telem_.inceptor(ref);}
  inline void throttle_ch(const uint8_t val) {telem_.throttle_ch(val);}
  /* Get parameters */
  static constexpr std::size_t size_params() {return PARAM_N;}
  inline std::array<float, PARAM_N> parameters() const {return param_.parameters();}

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Child classes */
  MavLinkHeartbeat heartbeat_;
  MavLinkTelemetry telem_;
  MavLinkParameter<PARAM_N> param_;
  MavLinkMission<MISSION_N, FENCE_N, RALLY_N> mission_;
  /* Message buffer */
  mavlink_message_t msg_;
  mavlink_status_t status_;
  uint8_t chan_ = 0;
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_MAVLINK_H_
