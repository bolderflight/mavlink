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

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <array>
#include <cmath>
#include "telemetry.h"  // NOLINT
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "units.h"  // NOLINT

namespace bfs {

namespace {

/* Converts a +/- 180 value to a 0 - 360 value */
template<typename T>
T WrapTo2Pi(T ang) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  ang = std::fmod(ang, BFS_2PI<T>);
  if (ang < static_cast<T>(0)) {
    ang += BFS_2PI<T>;
  }
  return ang;
}

/* Converts a 0 - 360 value to a +/- 180 value */
template<typename T>
T WrapToPi(T ang) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  if (ang > BFS_PI<T>) {
    ang -= BFS_2PI<T>;
  }
  if (ang < -BFS_PI<T>) {
    ang += BFS_2PI<T>;
  }
  return ang;
}

}  // namespace

void MavLinkTelemetry::Update() {
  /*
  * Data stream periods are set by default to -1 to disable them. Check for
  * enabled streams and, if their timer is greater than the period, send
  * them and reset the timer.
  */
  for (std::size_t stream_num = 0; stream_num < NUM_DATA_STREAMS_;
       stream_num++) {
    if (data_stream_period_ms_[stream_num] > 0) {
      if (data_stream_timer_ms_[stream_num] >
          data_stream_period_ms_[stream_num]) {
        (this->*(streams_[stream_num]))();
        data_stream_timer_ms_[stream_num] = 0;
      }
    }
  }
}
void MavLinkTelemetry::MsgHandler(const mavlink_message_t &ref) {
  switch (ref.msgid) {
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
      mavlink_msg_request_data_stream_decode(&ref, &request_stream_);
      ParseRequestDataStream(request_stream_);
      break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION: {
      mavlink_msg_home_position_decode(&ref, &home_pos_);
      ParseHomePosition(home_pos_);
      break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU: {
      mavlink_msg_scaled_imu_decode(&ref, &scaled_imu_);
      ParseScaledImu(scaled_imu_);
      break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
      mavlink_msg_scaled_pressure_decode(&ref, &scaled_pres_);
      ParseScaledPres(scaled_pres_);
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_msg_gps_raw_int_decode(&ref, &gps_raw_int_);
      ParseGpsRawInt(gps_raw_int_);
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_msg_attitude_decode(&ref, &attitude_);
      ParseAttitude(attitude_);
      break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_msg_vfr_hud_decode(&ref, &vfr_hud_);
      ParseVfrHud(vfr_hud_);
      break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_msg_local_position_ned_decode(&ref, &local_pos_ned_);
      ParseLocalPosNed(local_pos_ned_);
      break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_msg_global_position_int_decode(&ref, &global_pos_int_);
      ParseGlobalPosInt(global_pos_int_);
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
  SendBatteryStatus();
}
void MavLinkTelemetry::SendSysStatus() {
  sensors_present_ = 0;
  if (gyro_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
  }
  if (accel_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  }
  if (mag_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_3D_MAG;
  }
  if (static_pres_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  }
  if (diff_pres_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
  }
  if (gnss_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_GPS;
  }
  if (inceptor_installed_) {
    sensors_present_ |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  }
  /* Check sensor health */
  sensors_healthy_ = 0;
  if (gyro_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
  }
  if (accel_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  }
  if (mag_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_3D_MAG;
  }
  if (static_pres_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  }
  if (diff_pres_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
  }
  if (gnss_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_GPS;
  }
  if (inceptor_healthy_) {
    sensors_healthy_ |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  }
  load_ = static_cast<uint16_t>(1000.0f * static_cast<float>(frame_time_us_) /
          static_cast<float>(frame_period_us_));
  if (battery_volt_.set) {
    voltage_battery_ = static_cast<uint16_t>(battery_volt_.val * 1000.0f);
  }
  if (battery_current_ma_.set) {
    current_battery_ = static_cast<int16_t>(battery_current_ma_.val * 0.1f);
  }
  if (battery_remaining_prcnt_.set) {
    battery_remaining_ = static_cast<int8_t>(battery_remaining_prcnt_.val);
  }
  msg_len_ = mavlink_msg_sys_status_pack(sys_id_, comp_id_, &msg_,
                                         sensors_present_, sensors_present_,
                                         sensors_healthy_, load_,
                                         voltage_battery_, current_battery_,
                                         battery_remaining_, drop_rate_comm_,
                                         errors_comm_, errors_count_[0],
                                         errors_count_[1], errors_count_[2],
                                         errors_count_[3],
                                         onboard_control_sensors_present_ext_,
                                         onboard_control_sensors_present_ext_,
                                         onboard_control_sensors_present_ext_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendGpsRawInt() {
  switch (gnss_fix_) {
    case GNSS_FIX_NONE: {
      fix_ = GPS_FIX_TYPE_NO_FIX;
      break;
    }
    case GNSS_FIX_2D: {
      fix_ = GPS_FIX_TYPE_2D_FIX;
      break;
    }
    case GNSS_FIX_3D: {
      fix_ = GPS_FIX_TYPE_3D_FIX;
      break;
    }
    case GNSS_FIX_DGNSS: {
      fix_ = GPS_FIX_TYPE_DGPS;
      break;
    }
    case GNSS_FIX_RTK_FLOAT: {
      fix_ = GPS_FIX_TYPE_RTK_FLOAT;
      break;
    }
    case GNSS_FIX_RTK_FIXED: {
      fix_ = GPS_FIX_TYPE_RTK_FIXED;
      break;
    }
  }
  lat_dege7_ = static_cast<int32_t>(rad2deg(gnss_lat_rad_) * 1e7);
  lon_dege7_ = static_cast<int32_t>(rad2deg(gnss_lon_rad_) * 1e7);
  alt_msl_mm_ = static_cast<int32_t>(gnss_alt_msl_m_ * 1000.0f);
  if (gnss_hdop_.set) {
    eph_ = static_cast<uint16_t>(100.0f * gnss_hdop_.val);
  }
  if (gnss_vdop_.set) {
    epv_ = static_cast<uint16_t>(100.0f * gnss_vdop_.val);
  }
  if (gnss_vel_mps_.set) {
    vel_cmps_ = static_cast<uint16_t>(100.0f * gnss_vel_mps_.val);
  }
  if (gnss_track_rad_.set) {
    track_cdeg_ = static_cast<uint16_t>(100.0f *
                 rad2deg(WrapTo2Pi(gnss_track_rad_.val)));
  }
  if (gnss_num_sv_.set) {
    num_sv_ = gnss_num_sv_.val;
  }
  alt_wgs84_mm_ = static_cast<int32_t>(gnss_alt_wgs84_m_ * 1000.0f);
  h_acc_mm_ = static_cast<uint32_t>(gnss_horz_acc_m_ * 1000.0f);
  v_acc_mm_ = static_cast<uint32_t>(gnss_vert_acc_m_ * 1000.0f);
  vel_acc_mmps_ = static_cast<uint32_t>(gnss_vel_acc_mps_ * 1000.0f);
  hdg_acc_dege5_ = static_cast<uint32_t>(100000.0f *
                                                 rad2deg(gnss_track_acc_rad_));
  msg_len_ = mavlink_msg_gps_raw_int_pack(sys_id_, comp_id_, &msg_,
                                          sys_time_us_, fix_, lat_dege7_,
                                          lon_dege7_, alt_msl_mm_, eph_, epv_,
                                          vel_cmps_, track_cdeg_, num_sv_,
                                          alt_wgs84_mm_, h_acc_mm_, v_acc_mm_,
                                          vel_acc_mmps_, hdg_acc_dege5_,
                                          yaw_cdeg_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA1() {
  SendAttitude();
}
void MavLinkTelemetry::SendAttitude() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  if (nav_hdg_rad_.set) {
    yaw_rad_ = WrapToPi(nav_hdg_rad_.val);
  }
  msg_len_ = mavlink_msg_attitude_pack(sys_id_, comp_id_, &msg_,
                                       sys_time_ms_, nav_roll_rad_,
                                       nav_pitch_rad_, yaw_rad_,
                                       nav_gyro_x_radps_, nav_gyro_y_radps_,
                                       nav_gyro_z_radps_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA2() {
  SendVfrHud();
}
void MavLinkTelemetry::SendVfrHud() {
  if (nav_hdg_rad_.set) {
    hdg_deg_ = static_cast<int16_t>(
      rad2deg(WrapTo2Pi(nav_hdg_rad_.val)));
  }
  if (!use_throttle_prcnt_) {
    throttle_ = static_cast<uint16_t>(inceptor_[throttle_ch_] * 100.0f);
  } else {
    throttle_ = throttle_prcnt_;
  }
  climb_mps_ = -1.0f * nav_down_vel_mps_;
  msg_len_ = mavlink_msg_vfr_hud_pack(sys_id_, comp_id_, &msg_,
                                      nav_ias_mps_, nav_gnd_spd_mps_,
                                      hdg_deg_, throttle_, nav_alt_msl_m_,
                                      climb_mps_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_EXTRA3() {
  SendWindCov();
  SendSystemTime();
}

void MavLinkTelemetry::SendWindCov() {
  msg_len_ = mavlink_msg_wind_cov_pack(sys_id_, comp_id_, &msg_, sys_time_us_,
                                       wind_x_mps_, wind_y_mps_, wind_z_mps_,
                                       wind_var_horz_mps_, wind_var_vert_mps_,
                                       wind_alt_m_, wind_horz_acc_mps_,
                                       wind_vert_acc_mps_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

void MavLinkTelemetry::SendSystemTime() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  msg_len_ = mavlink_msg_system_time_pack(sys_id_, comp_id_, &msg_,
                                          unix_time_us_, sys_time_ms_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

void MavLinkTelemetry::SendBatteryStatus() {
  if (battery_volt_.set) {
    volt_[0] = static_cast<uint16_t>(battery_volt_.val * 1000.0f);
  }
  if (battery_current_ma_.set) {
    current_ = static_cast<int16_t>(battery_current_ma_.val * 0.1f);
  }
  if (battery_consumed_mah_.set) {
    current_consumed_ = static_cast<int32_t>(battery_consumed_mah_.val);
  }
  if (battery_remaining_prcnt_.set) {
    battery_remaining_ = static_cast<int8_t>(battery_remaining_prcnt_.val);
  }
  if (battery_remaining_time_s_.set) {
    time_remaining_ = static_cast<int32_t>(battery_remaining_time_s_.val);
  }
  msg_len_ = mavlink_msg_battery_status_pack(sys_id_, comp_id_, &msg_,
                                             id_, battery_function_, type_,
                                             temp_, &volt_[0], current_,
                                             current_consumed_,
                                             energy_consumed_,
                                             battery_remaining_,
                                             time_remaining_,
                                             charge_state_, &volt_[10],
                                             battery_mode_, fault_bitmask_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_POSITION() {
  SendLocalPositionNed();
  SendGlobalPositionInt();
}
void MavLinkTelemetry::SendLocalPositionNed() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  msg_len_ = mavlink_msg_local_position_ned_pack(sys_id_, comp_id_, &msg_,
                                                 sys_time_ms_,
                                                 nav_north_pos_m_,
                                                 nav_east_pos_m_,
                                                 nav_down_pos_m_,
                                                 nav_north_vel_mps_,
                                                 nav_east_vel_mps_,
                                                 nav_down_vel_mps_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendGlobalPositionInt() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  lat_dege7_ = static_cast<int32_t>(rad2deg(nav_lat_rad_) * 1e7);
  lon_dege7_ = static_cast<int32_t>(rad2deg(nav_lon_rad_) * 1e7);
  alt_msl_mm_ = static_cast<int32_t>(nav_alt_msl_m_ * 1000.0f);
  alt_agl_mm_ = static_cast<int32_t>(nav_alt_agl_m_ * 1000.0f);
  vx_cmps_ = static_cast<int16_t>(nav_north_vel_mps_ * 100.0f);
  vy_cmps_ = static_cast<int16_t>(nav_east_vel_mps_ * 100.0f);
  vz_cmps_ = static_cast<int16_t>(nav_down_vel_mps_ * 100.0f);
  if (nav_hdg_rad_.set) {
    hdg_cdeg_ = static_cast<uint16_t>(100.0f *
      rad2deg(WrapTo2Pi(nav_hdg_rad_.val)));
  }
  msg_len_ = mavlink_msg_global_position_int_pack(sys_id_, comp_id_, &msg_,
                                                  sys_time_ms_, lat_dege7_,
                                                  lon_dege7_, alt_msl_mm_,
                                                  alt_agl_mm_, vx_cmps_,
                                                  vy_cmps_, vz_cmps_,
                                                  hdg_cdeg_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RAW_SENS() {
  SendScaledImu();
  SendGpsRawInt();
  SendScaledPressure();
}
void MavLinkTelemetry::SendScaledImu() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  accel_x_mg_ = static_cast<int16_t>(convacc(imu_accel_x_mps2_,
                LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  accel_y_mg_ = static_cast<int16_t>(convacc(imu_accel_y_mps2_,
                LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  accel_z_mg_ = static_cast<int16_t>(convacc(imu_accel_z_mps2_,
                LinAccUnit::MPS2, LinAccUnit::G) * 1000.0f);
  gyro_x_mradps_ = static_cast<int16_t>(imu_gyro_x_radps_ * 1000.0f);
  gyro_y_mradps_ = static_cast<int16_t>(imu_gyro_y_radps_ * 1000.0f);
  gyro_z_mradps_ = static_cast<int16_t>(imu_gyro_z_radps_ * 1000.0f);
  mag_x_mgauss_ = static_cast<int16_t>(imu_mag_x_ut_ * 10.0f);
  mag_y_mgauss_ = static_cast<int16_t>(imu_mag_y_ut_ * 10.0f);
  mag_z_mgauss_ = static_cast<int16_t>(imu_mag_z_ut_ * 10.0f);
  if (imu_die_temp_c_.set) {
    temp_cc_ = static_cast<int16_t>(imu_die_temp_c_.val * 100.0f);
    if (temp_cc_ == 0) {
      temp_cc_ = 1;
    }
  }
  msg_len_ = mavlink_msg_scaled_imu_pack(sys_id_, comp_id_, &msg_,
                                         sys_time_ms_,
                                         accel_x_mg_, accel_y_mg_, accel_z_mg_,
                                         gyro_x_mradps_, gyro_y_mradps_,
                                         gyro_z_mradps_, mag_x_mgauss_,
                                         mag_y_mgauss_, mag_z_mgauss_,
                                         temp_cc_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendScaledPressure() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  static_pres_hpa_ = static_pres_pa_ / 100.0f;
  diff_pres_hpa_ = diff_pres_pa_ / 100.0f;
  static_temp_cc_ =
    static_cast<int16_t>(static_pres_die_temp_c_ * 100.0f);
  if (diff_pres_die_temp_c_.set) {
    diff_temp_cc_ = static_cast<int16_t>(diff_pres_die_temp_c_.val * 100.0f);
    if (diff_temp_cc_ == 0) {
      diff_temp_cc_ = 1;
    }
  }
  msg_len_ = mavlink_msg_scaled_pressure_pack(sys_id_, comp_id_, &msg_,
                                              sys_time_ms_,
                                              static_pres_hpa_,
                                              diff_pres_hpa_,
                                              static_temp_cc_,
                                              diff_temp_cc_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RC_CHAN() {
  SendServoOutputRaw();
  SendRcChannels();
}
void MavLinkTelemetry::SendServoOutputRaw() {
  /* Transform percent [0 - 1] to PWM value */
  if (!use_raw_effector_) {
    for (std::size_t i = 0; i < 16; i++) {
      servo_raw_[i] = static_cast<uint16_t>(effector_[i] * 1000.0f + 1000.0f);
    }
  } else {
    for (std::size_t i = 0; i < 16; i++) {
      servo_raw_[i] = effector_[i];
    }
  }
  msg_len_ = mavlink_msg_servo_output_raw_pack(sys_id_, comp_id_, &msg_,
                                               sys_time_us_, port_,
                                               servo_raw_[0], servo_raw_[1],
                                               servo_raw_[2], servo_raw_[3],
                                               servo_raw_[4], servo_raw_[5],
                                               servo_raw_[6], servo_raw_[7],
                                               servo_raw_[8], servo_raw_[9],
                                               servo_raw_[10], servo_raw_[11],
                                               servo_raw_[12], servo_raw_[13],
                                               servo_raw_[14], servo_raw_[15]);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SendRcChannels() {
  sys_time_ms_ = static_cast<uint32_t>(sys_time_us_ / 1000);
  /* Transform percent [0 - 1] to PWM value */
  if (!use_raw_inceptor_) {
    for (std::size_t i = 0; i < chancount_; i++) {
      chan_[i] = static_cast<uint16_t>(inceptor_[i] * 1000.0f + 1000.0f);
    }
  } else {
    for (std::size_t i = 0; i < chancount_; i++) {
      chan_[i] = inceptor_[i];
    }
  }
  msg_len_ = mavlink_msg_rc_channels_pack(sys_id_, comp_id_, &msg_,
                                          sys_time_ms_, chancount_,
                                          chan_[0], chan_[1],
                                          chan_[2], chan_[3],
                                          chan_[4], chan_[5],
                                          chan_[6], chan_[7],
                                          chan_[8], chan_[9],
                                          chan_[10], chan_[11],
                                          chan_[12], chan_[13],
                                          chan_[14], chan_[15],
                                          chan_[16], chan_[17], rssi_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkTelemetry::SRx_RAW_CTRL() {}
void MavLinkTelemetry::SRx_EMPTY() {}
void MavLinkTelemetry::ParseRequestDataStream(
                       const mavlink_request_data_stream_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    if (ref.start_stop) {
      data_stream_period_ms_[ref.req_stream_id] =
        static_cast<int32_t>(1000.0f /
        static_cast<float>(ref.req_message_rate));
    } else {
      data_stream_period_ms_[ref.req_stream_id] = -1;
    }
  }
}
void MavLinkTelemetry::ParseScaledImu(const mavlink_scaled_imu_t &ref) {
  rx_imu_accel_x_mps2_ = convacc(static_cast<float>(ref.xacc) / 1000.0f,
                                 LinAccUnit::G, LinAccUnit::MPS2);
  rx_imu_accel_y_mps2_ = convacc(static_cast<float>(ref.yacc) / 1000.0f,
                                LinAccUnit::G, LinAccUnit::MPS2);
  rx_imu_accel_z_mps2_ = convacc(static_cast<float>(ref.zacc) / 1000.0f,
                                 LinAccUnit::G, LinAccUnit::MPS2);
  rx_imu_gyro_x_radps_ = static_cast<float>(ref.xgyro) / 1000.0f;
  rx_imu_gyro_y_radps_ = static_cast<float>(ref.ygyro) / 1000.0f;
  rx_imu_gyro_z_radps_ = static_cast<float>(ref.zgyro) / 1000.0f;
  rx_imu_mag_x_ut_ = static_cast<float>(ref.xmag) / 10.0f;
  rx_imu_mag_y_ut_ = static_cast<float>(ref.ymag) / 10.0f;
  rx_imu_mag_z_ut_ = static_cast<float>(ref.zmag) / 10.0f;
  if (ref.temperature != 0) {
    rx_imu_die_temp_c_ = static_cast<float>(ref.temperature) / 100.0f;
  }
}
void MavLinkTelemetry::ParseScaledPres(const mavlink_scaled_pressure_t &ref) {
  rx_static_pres_pa_ = convpres(ref.press_abs, PresUnit::HPA, PresUnit::PA);
  rx_diff_pres_pa_ = convpres(ref.press_diff, PresUnit::HPA, PresUnit::PA);
  rx_static_pres_die_temp_c_ = static_cast<float>(ref.temperature) / 100.0f;
  if (ref.temperature_press_diff != 0) {
    rx_diff_pres_die_temp_c_ = static_cast<float>(ref.temperature_press_diff) /
                               100.0f;
  }
}
void MavLinkTelemetry::ParseGpsRawInt(const mavlink_gps_raw_int_t &ref) {
  rx_gnss_fix_ = ref.fix_type;
  rx_gnss_lat_rad_ = static_cast<double>(ref.lat) / 1e7;
  rx_gnss_lon_rad_ = static_cast<double>(ref.lon) / 1e7;
  rx_gnss_alt_msl_m_ = static_cast<float>(ref.alt) / 1000.0f;
  if (ref.eph != UINT16_MAX) {
    rx_gnss_hdop_ = static_cast<float>(ref.eph) / 100.0f;
  }
  if (ref.epv != UINT16_MAX) {
    rx_gnss_vdop_ = static_cast<float>(ref.epv) / 100.0f;
  }
  if (ref.vel != UINT16_MAX) {
    rx_gnss_spd_mps_ = static_cast<float>(ref.vel) / 100.0f;
  }
  if (ref.cog != UINT16_MAX) {
    rx_gnss_track_rad_ = deg2rad(static_cast<float>(ref.cog) / 100.0f);
  }
  if (ref.satellites_visible != UINT8_MAX) {
    rx_gnss_num_sats_ = ref.satellites_visible;
  }
  rx_gnss_alt_wgs84_m_ = static_cast<float>(ref.alt_ellipsoid) / 1000.0f;
  rx_gnss_horz_acc_m_ = static_cast<float>(ref.h_acc) / 1000.0f;
  rx_gnss_vert_acc_m_ = static_cast<float>(ref.v_acc) / 1000.0f;
  rx_gnss_vel_acc_mps_ = static_cast<float>(ref.vel_acc) / 1000.0f;
  rx_gnss_track_acc_rad_ = deg2rad(static_cast<float>(ref.hdg_acc) / 100000.0f);
  if ((ref.yaw != 0) && (ref.yaw != UINT16_MAX)) {
    rx_gnss_yaw_ = deg2rad(static_cast<float>(ref.yaw) / 100.0f);
  }
}
void MavLinkTelemetry::ParseAttitude(const mavlink_attitude_t &ref) {
  rx_nav_pitch_rad = ref.pitch;
  rx_nav_roll_rad = ref.roll;
  rx_nav_hdg_rad = ref.yaw;
  rx_nav_gyro_x_radps = ref.rollspeed;
  rx_nav_gyro_y_radps = ref.pitchspeed;
  rx_nav_gyro_z_radps = ref.yawspeed;
}
void MavLinkTelemetry::ParseVfrHud(const mavlink_vfr_hud_t &ref) {
  rx_nav_ias_mps_ = ref.airspeed;
  rx_nav_gnd_spd_mps_ = ref.groundspeed;
}
void MavLinkTelemetry::ParseLocalPosNed(
                                      const mavlink_local_position_ned_t &ref) {
  rx_nav_north_pos_m_ = ref.x;
  rx_nav_east_pos_m_ = ref.y;
  rx_nav_down_pos_m_ = ref.z;
  rx_nav_north_vel_mps_ = ref.vx;
  rx_nav_east_vel_mps_ = ref.vy;
  rx_nav_down_vel_mps_ = ref.vz;
}
void MavLinkTelemetry::ParseGlobalPosInt(
                                     const mavlink_global_position_int_t &ref) {
  rx_nav_lat_rad_ = deg2rad(static_cast<double>(ref.lat) / 1e7);
  rx_nav_lon_rad_ = deg2rad(static_cast<double>(ref.lon) / 1e7);
  rx_nav_alt_msl_m_ = static_cast<float>(ref.alt) / 1000.0f;
  rx_nav_alt_agl_m_ = static_cast<float>(ref.relative_alt) / 1000.0f;
  rx_nav_north_vel_mps_ = static_cast<float>(ref.vx) / 100.0f;
  rx_nav_east_vel_mps_ = static_cast<float>(ref.vy) / 100.0f;
  rx_nav_down_vel_mps_ = static_cast<float>(ref.vz) /100.0f;
}

void MavLinkTelemetry::SendHomePos() {
  lat_dege7_ = static_cast<int32_t>(rad2deg(home_lat_rad_) * 1e7);
  lon_dege7_ = static_cast<int32_t>(rad2deg(home_lon_rad_) * 1e7);
  alt_msl_mm_ = static_cast<int32_t>(home_alt_m_ * 1000.0f);
  float q[4];
  msg_len_ = mavlink_msg_home_position_pack(sys_id_, comp_id_, &msg_,
                                            lat_dege7_, lon_dege7_,
                                            alt_msl_mm_, 0, 0, 0,
                                            q, 0, 0, 0, 0);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

void MavLinkTelemetry::ParseHomePosition(const mavlink_home_position_t &ref) {
  rx_home_lat_rad_ = deg2rad(static_cast<double>(ref.latitude) / 1e7);
  rx_home_lon_rad_ = deg2rad(static_cast<double>(ref.longitude) / 1e7);
  rx_home_alt_m_ = static_cast<float>(ref.altitude) / 1000.0f;
}

}  // namespace bfs
