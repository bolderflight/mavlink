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

#include <array>
#include "mavlink/telemetry.h"
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "units/units.h"
#include "navigation/navigation.h"

namespace bfs {

void MavLinkTelemetry::Update() {
  /*
  * Data stream periods are set by default to -1 to disable them. Check for
  * enabled streams and, if their timer is greater than the period, send
  * them and reset the timer.
  */
  for (std::size_t stream_num = 0; stream_num < NUM_DATA_STREAMS_; stream_num++) {
    if (data_stream_period_ms_[stream_num] > 0) {
      if (data_stream_timer_ms_[stream_num] > data_stream_period_ms_[stream_num]) {
        (this->*(streams_[stream_num]))();
        data_stream_timer_ms_[stream_num] = 0;
      }
    }
  }
}
void MavLinkTelemetry::MsgHandler(const mavlink_message_t &ref) {
  switch(ref.msgid) {
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
      mavlink_request_data_stream_t request_stream;
      mavlink_msg_request_data_stream_decode(&ref, &request_stream);
      ParseRequestDataStream(request_stream);
      break;
    }
  }
}
void MavLinkTelemetry::SRx_ALL() {
  SRx_RAW_SENS();
  SRx_EXT_STAT();
  SRx_RC_CHAN();
  SRx_RAW_CTRL();
  SRx_POSITION();
  SRx_EXTRA1();
  SRx_EXTRA2();
  SRx_EXTRA3();
}
void MavLinkTelemetry::SRx_EXT_STAT() {
  SendSysStatus();
  SendGpsRawInt();
}
void MavLinkTelemetry::SendSysStatus() {
  /* 
  * Assume IMU, static pressure, GNSS, and RC receiver installed,
  * check differential pressure.
  */
  uint32_t sensors_present = 0;
  sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
  sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
  sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
  sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  if (diff_pres_installed_) {
    sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
  }
  /* All installed sensors are enabled */
  uint32_t sensors_enabled = sensors_present;
  /* Check sensor health */
  uint32_t sensors_healthy = 0;
  if (imu_healthy_) {
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_3D_MAG;
  }
  if (static_pres_healthy_) {
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  }
  if (diff_pres_healthy_) {
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
  }
  if (gnss_healthy_) {
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_GPS;
  }
  if (inceptor_healthy_) {
    sensors_healthy |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  }
  uint16_t load = static_cast<uint16_t>(cpu_load_ * 10.0f);
  uint16_t voltage_battery = UINT16_MAX;
  if (battery_volt_.set) {
    voltage_battery = static_cast<uint16_t>(battery_volt_.val * 1000.0f);
  }
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;
  uint16_t drop_rate_comm = 0;
  uint16_t errors_comm = 0;
  uint16_t errors_count[4] = {0};
  msg_len_ = mavlink_msg_sys_status_pack(sys_id_, comp_id_, &msg_,
                                         sensors_present, sensors_enabled,
                                         sensors_healthy, load,
                                         voltage_battery, current_battery,
                                         battery_remaining, drop_rate_comm,
                                         errors_comm, errors_count[0],
                                         errors_count[1], errors_count[2],
                                         errors_count[3]);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendGpsRawInt() {
  uint64_t sys_time_us = static_cast<uint64_t>(sys_time_s_ * 1e6);
  uint8_t fix = static_cast<uint8_t>(gnss_fix_);
  int32_t lat_dege7 = static_cast<int32_t>(rad2deg(gnss_lat_rad_) * 1e7);
  int32_t lon_dege7 = static_cast<int32_t>(rad2deg(gnss_lon_rad_) * 1e7);
  int32_t alt_msl_mm = static_cast<int32_t>(gnss_alt_msl_m_ * 1000.0f);
  uint16_t eph = UINT16_MAX, epv = UINT16_MAX;
  float gnss_gs_mps = std::sqrt(gnss_north_vel_mps_ * gnss_north_vel_mps_ +
                                gnss_east_vel_mps_ * gnss_east_vel_mps_);
  uint16_t vel_cmps = static_cast<uint16_t>(100.0f * gnss_gs_mps);
  float gnss_track_rad = std::atan2(gnss_east_vel_mps_, gnss_north_vel_mps_);
  uint16_t track_cdeg = static_cast<uint16_t>(
                        rad2deg(Constrain2Pi(gnss_track_rad)) * 100.0f);
  uint8_t num_sv = 255;
  if (gnss_num_sv_.set) {
    num_sv = gnss_num_sv_.val;
  }
  int32_t alt_wgs84 = static_cast<int32_t>(gnss_alt_wgs84_m_ * 1000.0f);
  uint32_t h_acc_mm = static_cast<uint32_t>(gnss_horz_acc_m_ * 1000.0f);
  uint32_t v_acc_mm = static_cast<uint32_t>(gnss_vert_acc_m_ * 1000.0f);
  uint32_t vel_acc_mmps = static_cast<uint32_t>(gnss_vel_acc_mps_ * 1000.0f);
  /* Not currently supporting heading accuracy */
  uint32_t hdg_acc_dege5 = 0;
  /* Not currently supporting GNSS yaw */
  uint16_t yaw_cdeg = 0;
  msg_len_ = mavlink_msg_gps_raw_int_pack(sys_id_, comp_id_, &msg_,
                                          sys_time_us, fix, lat_dege7,
                                          lon_dege7, alt_msl_mm, eph, epv,
                                          vel_cmps, track_cdeg, num_sv,
                                          alt_wgs84, h_acc_mm, v_acc_mm,
                                          vel_acc_mmps, hdg_acc_dege5,
                                          yaw_cdeg);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA1() {
  SendAttitude();
}
void MavLinkTelemetry::SendAttitude() {
  uint32_t sys_time_ms = static_cast<uint32_t>(sys_time_s_ * 1e3);
  float yaw_rad = 0;
  if (nav_hdg_rad_.set) {
    yaw_rad = ConstrainPi(nav_hdg_rad_.val);
  }
  msg_len_ = mavlink_msg_attitude_pack(sys_id_, comp_id_, &msg_,
                                       sys_time_ms, nav_roll_rad_,
                                       nav_pitch_rad_, yaw_rad,
                                       nav_gyro_x_radps_, nav_gyro_y_radps_,
                                       nav_gyro_z_radps_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA2() {
  SendVfrHud();
}
void MavLinkTelemetry::SendVfrHud() {
  float nav_gnd_speed_mps = std::sqrt(nav_north_vel_mps_ * nav_north_vel_mps_ +
                                      nav_east_vel_mps_ * nav_east_vel_mps_);
  int16_t hdg_deg = 0;
  if (nav_hdg_rad_.set) {
    hdg_deg = static_cast<int16_t>(
      rad2deg(Constrain2Pi(nav_hdg_rad_.val)));
  }
  uint16_t throttle = static_cast<uint16_t>(inceptor_[throttle_ch_] * 100.0f);
  float climb_mps = -1.0f * nav_down_vel_mps_;
  msg_len_ = mavlink_msg_vfr_hud_pack(sys_id_, comp_id_, &msg_,
                                      nav_ias_mps_, nav_gnd_speed_mps,
                                      hdg_deg, throttle, nav_alt_msl_m_,
                                      climb_mps);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA3() {
  SendBatteryStatus();
}
void MavLinkTelemetry::SendBatteryStatus() {
  uint8_t id = 0;
  uint8_t battery_function = MAV_BATTERY_FUNCTION_UNKNOWN;
  uint8_t type = MAV_BATTERY_TYPE_LIPO;
  int16_t temp = INT16_MAX;
  uint16_t volt[14] = {};
  for (std::size_t i = 0; i < 14; i++) {
    volt[i] = UINT16_MAX;
  }
  if (battery_volt_.set) {
    volt[0] = static_cast<uint16_t>(battery_volt_.val * 1000.0f);
  }
  int16_t current = -1;
  int32_t current_consumed = -1;
  int32_t energy_consumed = -1;
  int8_t battery_remaining = -1;
  int32_t time_remaining = 0;
  uint8_t charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
  msg_len_ = mavlink_msg_battery_status_pack(sys_id_, comp_id_, &msg_,
                                             id, battery_function, type, temp,
                                             &volt[0], current,
                                             current_consumed, energy_consumed,
                                             battery_remaining, time_remaining,
                                             charge_state, &volt[10]);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_POSITION() {
  SendGlobalPositionInt();
}
void MavLinkTelemetry::SendGlobalPositionInt() {
  uint32_t sys_time_ms = static_cast<uint32_t>(sys_time_s_ * 1e3);
  int32_t lat_dege7 = static_cast<int32_t>(rad2deg(nav_lat_rad_) * 1e7);
  int32_t lon_dege7 = static_cast<int32_t>(rad2deg(nav_lon_rad_) * 1e7);
  int32_t alt_msl_mm = static_cast<int32_t>(nav_alt_msl_m_ * 1000.0f);
  int32_t alt_agl_mm = static_cast<int32_t>(nav_alt_agl_m_ * 1000.0f);
  int16_t vx_cmps = static_cast<int16_t>(nav_north_vel_mps_ * 100.0f);
  int16_t vy_cmps = static_cast<int16_t>(nav_east_vel_mps_ * 100.0f);
  int16_t vz_cmps = static_cast<int16_t>(nav_down_vel_mps_ * 100.0f);
  uint16_t hdg_cdeg = UINT16_MAX;
  if (nav_hdg_rad_.set) {
    hdg_cdeg = static_cast<uint16_t>(
      rad2deg(Constrain2Pi(nav_hdg_rad_.val)) * 100.0f);
  }
  msg_len_ = mavlink_msg_global_position_int_pack(sys_id_, comp_id_, &msg_,
                                                  sys_time_ms, lat_dege7,
                                                  lon_dege7, alt_msl_mm,
                                                  alt_agl_mm, vx_cmps,
                                                  vy_cmps, vz_cmps, hdg_cdeg);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RAW_SENS() {
  SendRawImu();
  SendScaledPressure();
}
void MavLinkTelemetry::SendRawImu() {
  uint64_t sys_time_us = static_cast<uint64_t>(sys_time_s_ * 1e6);
  int16_t accel_x_mg = static_cast<int16_t>(convacc(imu_accel_x_mps2_,
                       LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  int16_t accel_y_mg = static_cast<int16_t>(convacc(imu_accel_y_mps2_,
                       LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  int16_t accel_z_mg = static_cast<int16_t>(convacc(imu_accel_z_mps2_,
                       LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  int16_t gyro_x_mradps = static_cast<int16_t>(imu_gyro_x_radps_ * 1000.0f);
  int16_t gyro_y_mradps = static_cast<int16_t>(imu_gyro_y_radps_ * 1000.0f);
  int16_t gyro_z_mradps = static_cast<int16_t>(imu_gyro_z_radps_ * 1000.0f);
  int16_t mag_x_mgauss = static_cast<int16_t>(imu_mag_x_ut_ * 10.0f);
  int16_t mag_y_mgauss = static_cast<int16_t>(imu_mag_y_ut_ * 10.0f);
  int16_t mag_z_mgauss = static_cast<int16_t>(imu_mag_z_ut_ * 10.0f);
  int16_t temp_cc = 0;
  if (imu_die_temp_c_.set) {
    temp_cc = static_cast<int16_t>(imu_die_temp_c_.val * 100.0f);
    if (temp_cc == 0) {
      temp_cc = 1;
    }
  }
  uint8_t imu_id = 0;
  msg_len_ = mavlink_msg_raw_imu_pack(sys_id_, comp_id_, &msg_,
                                      sys_time_us,
                                      accel_x_mg, accel_y_mg, accel_z_mg,
                                      gyro_x_mradps, gyro_y_mradps,
                                      gyro_z_mradps, mag_x_mgauss,
                                      mag_y_mgauss, mag_z_mgauss,
                                      imu_id, temp_cc);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendScaledPressure() {
  uint32_t sys_time_ms = static_cast<uint32_t>(sys_time_s_ * 1e3);
  float static_pres_hpa = static_pres_pa_ / 100.0f;
  float diff_pres_hpa = diff_pres_pa_ / 100.0f;
  int16_t static_temp_cc =
    static_cast<int16_t>(static_pres_die_temp_c_ * 100.0f);
  msg_len_ = mavlink_msg_scaled_pressure_pack(sys_id_, comp_id_, &msg_,
                                              sys_time_ms,
                                              static_pres_hpa,
                                              diff_pres_hpa,
                                              static_temp_cc);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RC_CHAN() {
  SendServoOutputRaw();
  SendRcChannels();
}
void MavLinkTelemetry::SendServoOutputRaw() {
  uint32_t sys_time_us = static_cast<uint32_t>(sys_time_s_ * 1e6);
  uint8_t port = 0;
  uint16_t servo_raw[16] = {0};
  /* Transform percent [0 - 1] to PWM value */
  for (std::size_t i = 0; i < 16; i++) {
    servo_raw[i] = static_cast<uint16_t>(effector_[i] * 1000.0f + 1000.0f);
  }
  msg_len_ = mavlink_msg_servo_output_raw_pack(sys_id_, comp_id_, &msg_,
                                               sys_time_us, port,
                                               servo_raw[0], servo_raw[1],
                                               servo_raw[2], servo_raw[3],
                                               servo_raw[4], servo_raw[5],
                                               servo_raw[6], servo_raw[7],
                                               servo_raw[8], servo_raw[9],
                                               servo_raw[10], servo_raw[11],
                                               servo_raw[12], servo_raw[13],
                                               servo_raw[14], servo_raw[15]);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendRcChannels() {
  uint32_t sys_time_ms = static_cast<uint32_t>(sys_time_s_ * 1e3);
  uint8_t chancount = 16;
  uint16_t chan[18] = {0};
  /* RSSI not currently supported */
  uint8_t rssi = 255;
  /* Transform percent [0 - 1] to PWM value */
  for (std::size_t i = 0; i < chancount; i++) {
    chan[i] = static_cast<uint16_t>(inceptor_[i] * 1000.0f + 1000.0f);
  }
  msg_len_ = mavlink_msg_rc_channels_pack(sys_id_, comp_id_, &msg_,
                                          sys_time_ms, chancount,
                                          chan[0], chan[1],
                                          chan[2], chan[3],
                                          chan[4], chan[5],
                                          chan[6], chan[7],
                                          chan[8], chan[9],
                                          chan[10], chan[11],
                                          chan[12], chan[13],
                                          chan[14], chan[15],
                                          chan[16], chan[17], rssi);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RAW_CTRL() {}
void MavLinkTelemetry::SRx_EMPTY() {}
void MavLinkTelemetry::ParseRequestDataStream(const mavlink_request_data_stream_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    if (ref.start_stop) {
      float msg_period_s = 1.0f / static_cast<float>(ref.req_message_rate);
      data_stream_period_ms_[ref.req_stream_id] =
        static_cast<int>(msg_period_s * 1000.0f);
    } else {
      data_stream_period_ms_[ref.req_stream_id] = -1;
    }
  }
}

}  // namespace bfs
