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

#include "mavlink/mavlink.h"
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "ardupilotmega/ardupilotmega.h"
#include "units/units.h"
#include "navigation/navigation.h"

namespace bfs {

void MavLink::Update() {
  /* Send heartbeat */
  if (heartbeat_timer_ms_ > HEARTBEAT_PERIOD_MS_) {
    SendHeartbeat();
    heartbeat_timer_ms_ = 0;
  }
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
  /*
  * Stream parameters, one per PARAM_PERIOD_MS. The param_index is initially
  * set to -1. Upon receiving a PARAM_REQUEST_LIST message, we set the index
  * to 0 to start the stream, which will occur until we get through the entire
  * set of parameters.
  */
  if ((param_index_ >= 0) && (param_index_ < NUM_PARAMS)) {
    if (param_timer_ms_ > PARAM_PERIOD_MS_) {
      Serial.print("PARAM INDEX ");
      Serial.println(param_index_);
      SendParam(param_index_++);
      param_timer_ms_ = 0;
    }
  }
  /*
  * Request mission items, one per MISSION_PERIOD_MS. The mission_upload_index_
  * is initial set -1. Upon receiving a MISSION_COUNT message, we set the count
  * to the maximum value and the index to 0 to start the stream, which will
  * occur until all mission items are recieved.
  */
 if ((mission_upload_index_ >= 0) &&
     (mission_upload_index_ < mission_count_upload_)) {
    if (mission_timer_ms_ > MISSION_PERIOD_MS_) {
      Serial.print("MISSION INDEX ");
      Serial.println(mission_upload_index_);
      SendMissionRequestInt(mission_upload_index_);
      mission_timer_ms_ = 0;
    }
  }

  /*
  *
  */

  /* Check for received messages */
  while (bus_->available()) {
    if (mavlink_frame_char(chan_, bus_->read(), &msg_, &status_) != MAVLINK_FRAMING_INCOMPLETE) {
      /* The sys id and component id of the request */
      sys_id_req_ = msg_.sysid;
      comp_id_req_ = msg_.compid;
      switch (msg_.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
          mavlink_param_request_read_t request_read;
          mavlink_msg_param_request_read_decode(&msg_, &request_read);
          ParamRequestRead(request_read);
          break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          mavlink_param_request_list_t request_list;
          mavlink_msg_param_request_list_decode(&msg_, &request_list);
          ParamRequestList(request_list);
          break;
        }
        case MAVLINK_MSG_ID_PARAM_SET: {
          mavlink_param_set_t param_set;
          mavlink_msg_param_set_decode(&msg_, &param_set);
          ParamSet(param_set);
          break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM: {
          mission_upload_index_++;
          Serial.println("MISSION ITEM");
          mavlink_mission_item_t mission_item;
          mavlink_msg_mission_item_decode(&msg_, &mission_item);
          Serial.print(mission_item.seq);
          Serial.print("\t");
          Serial.print(mission_item.frame);
          Serial.print("\t");
          Serial.print(mission_item.command);
          Serial.print("\t");
          Serial.print(mission_item.current);
          Serial.print("\t");
          Serial.print(mission_item.autocontinue);
          Serial.print("\t");
          Serial.print(mission_item.param1);
          Serial.print("\t");
          Serial.print(mission_item.param2);
          Serial.print("\t");
          Serial.print(mission_item.param3);
          Serial.print("\t");
          Serial.print(mission_item.param4);
          Serial.print("\t");
          Serial.print(mission_item.x);
          Serial.print("\t");
          Serial.print(mission_item.y);
          Serial.print("\t");
          Serial.println(mission_item.z);

          mavlink_msg_mission_ack_pack(sys_id_, comp_id_, &msg_, sys_id_req_, comp_id_req_, 0, MAV_MISSION_TYPE_MISSION);
          mavlink_msg_to_send_buffer(msg_buf_, &msg_);
          bus_->write(msg_buf_, msg_len_);

          break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST: {
          mavlink_mission_request_t mission_request;
          mavlink_msg_mission_request_decode(&msg_, &mission_request);
          MissionRequest(mission_request);
          break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
          mavlink_mission_request_int_t mission_request_int;
          mavlink_msg_mission_request_int_decode(&msg_, &mission_request_int);
          MissionRequestInt(mission_request_int);
          break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
          mavlink_mission_request_list_t mission_request_list;
          mavlink_msg_mission_request_list_decode(&msg_,
                                                  &mission_request_list);
          if ((mission_request_list.target_system == sys_id_) &&
              (mission_request_list.target_component == comp_id_)) {
            SendMissionCount();
          }
          break;
        }
        case MAVLINK_MSG_ID_MISSION_COUNT: {
          mavlink_mission_count_t mission_count;
          mavlink_msg_mission_count_decode(&msg_, &mission_count);
          SetMissionCount(mission_count);
          break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
          Serial.println("MISSION ITEM INT");
          break;
        }
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
          mavlink_mission_set_current_t set_current;
          mavlink_msg_mission_set_current_decode(&msg_, &set_current);
          MissionSetCurrent(set_current);
          break;
        }
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
          Serial.println("MISSION CLEAR ALL");
          break;
        }
        case MAVLINK_MSG_ID_MISSION_ACK: {
          Serial.println("MISSION ACK");
          break;
        }
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
          mavlink_request_data_stream_t request_stream;
          mavlink_msg_request_data_stream_decode(&msg_, &request_stream);
          ParseRequestDataStream(request_stream);
          break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
          // mavlink_command_long_t cmd;
          // mavlink_msg_command_long_decode(&msg_, &cmd);
          // Serial.print(cmd.command);
          // Serial.print("\t");
          // Serial.print(cmd.param1);
          // Serial.print("\t");
          // Serial.print(cmd.param2);
          // Serial.print("\t");
          // Serial.print(cmd.param3);
          // Serial.print("\t");
          // Serial.print(cmd.param4);
          // Serial.print("\t");
          // Serial.print(cmd.param5);
          // Serial.print("\t");
          // Serial.print(cmd.param6);
          // Serial.print("\t");
          // Serial.print(cmd.param7);
          // Serial.print("\n");
        }
        case MAVLINK_MSG_ID_RADIO_STATUS: {
          break;
        }
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST: {
          // send autopilot version response


          break;
        }

        case MAVLINK_MSG_ID_STATUSTEXT: {
          /* OK, MP is just telling us its version number */


          // mavlink_statustext_t status_text;
          // mavlink_msg_statustext_decode(&msg_, &status_text);
          // Serial.println("STATUS TEXT");
          // Serial.print("SEVERITY: ");
          // Serial.println(status_text.severity);
          // Serial.print("TEXT: ");
          // Serial.println(status_text.text);
          break;
        }

        default: {
          Serial.print("MSG ID: ");
          Serial.print(msg_.msgid);
          Serial.print("\tStatus: ");
          Serial.println(status_.parse_error);

        }
      }
    }
  }
}

void MavLink::SendHeartbeat() {
  uint8_t type = static_cast<uint8_t>(vehicle_type_);
  uint8_t mode = 0;
  if (!throttle_enabled_) {
    mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  switch (vehicle_mode_) {
    case VehicleMode::MANUAL: {
      mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    }
    case VehicleMode::STABALIZED: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case VehicleMode::ATTITUDE: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case VehicleMode::AUTO: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
      break;
    }
    case VehicleMode::TEST: {
      mode |= MAV_MODE_FLAG_TEST_ENABLED;
      break;
    }
  }
  uint32_t custom_mode = 0;
  uint8_t state = static_cast<uint8_t>(vehicle_state_);
  msg_len_ = mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg_,
                                        type, autopilot_, mode,
                                        custom_mode, state);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLink::SRx_ALL() {
  SRx_RAW_SENS();
  SRx_EXT_STAT();
  SRx_RC_CHAN();
  SRx_RAW_CTRL();
  SRx_POSITION();
  SRx_EXTRA1();
  SRx_EXTRA2();
  SRx_EXTRA3();
}
void MavLink::SRx_EXT_STAT() {
  SendSysStatus();
  SendGpsRawInt();
}
void MavLink::SendSysStatus() {
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
void MavLink::SendGpsRawInt() {
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
void MavLink::SRx_EXTRA1() {
  SendAttitude();
}
void MavLink::SendAttitude() {
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
void MavLink::SRx_EXTRA2() {
  SendVfrHud();
}
void MavLink::SendVfrHud() {
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
void MavLink::SRx_EXTRA3() {
  SendBatteryStatus();
}
void MavLink::SendBatteryStatus() {
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
void MavLink::SRx_POSITION() {
  SendGlobalPositionInt();
}
void MavLink::SendGlobalPositionInt() {
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
void MavLink::SRx_RAW_SENS() {
  SendRawImu();
  SendScaledPressure();
}
void MavLink::SendRawImu() {
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
void MavLink::SendScaledPressure() {
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
void MavLink::SRx_RC_CHAN() {
  SendServoOutputRaw();
  SendRcChannels();
}
void MavLink::SendServoOutputRaw() {
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
void MavLink::SendRcChannels() {
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
void MavLink::SRx_RAW_CTRL() {}
void MavLink::SRx_EMPTY() {}
void MavLink::ParseRequestDataStream(const mavlink_request_data_stream_t &ref) {
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
void MavLink::SendParam(const std::size_t i) {
  /* Index bounds checking */
  if ((i < 0) || (i > NUM_PARAMS - 1)) {return;}
  msg_len_ = mavlink_msg_param_value_pack(sys_id_, comp_id_, &msg_,
                                          params_[i].param_id.c_str(),
                                          params_[i].val,
                                          params_[i].param_type,
                                          params_[i].param_count,
                                          params_[i].param_index
  );
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLink::ParamRequestRead(const mavlink_param_request_read_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    /*
    * Param index is set to -1 if param id should be used instead. So, check
    * if param index is valid and use that, otherwise use param id.
    */
    if (ref.param_index > -1) {
      SendParam(ref.param_index);
    } else {
      for (std::size_t i = 0; i < NUM_PARAMS; i++) {
        if (std::string(ref.param_id) == params_[i].param_id) {
          SendParam(i);
          return;
        }
      }
    }
  }
}
void MavLink::ParamRequestList(const mavlink_param_request_list_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    /* Set param_index_ to zero to start streaming a full list of parameters */
    param_index_ = 0;
  }
}
void MavLink::ParamSet(const mavlink_param_set_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    for (std::size_t i = 0; i < NUM_PARAMS; i++) {
      if (std::string(ref.param_id) == params_[i].param_id) {
        params_[i].val = ref.param_value;
        SendParam(i);
        return;
      }
    }
  }
}
void MavLink::MissionRequest(const mavlink_mission_request_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    Serial.println("MISSION REQUEST");

  }
}
void MavLink::MissionRequestInt(const mavlink_mission_request_int_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    Serial.println("MISSION REQUEST INT");
  }
}
void MavLink::SendMissionCount() {
  uint8_t mission_type = static_cast<uint8_t>(MissionType::MISSION);
  msg_len_ = mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg_,
                                            sys_id_req_, comp_id_req_,
                                            mission_count_[0],
                                            mission_type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLink::MissionSetCurrent(const mavlink_mission_set_current_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    /* Range check */
    if (InRange(ref.seq, mission_count_[0])) {
      mission_current_[0] = ref.seq;
      SendMissionCurrent();
    } else {
      // SEND ERROR
    }
  }
}
void MavLink::SendMissionCurrent() {
  msg_len_ = mavlink_msg_mission_current_pack(sys_id_, comp_id_req_, &msg_,
                                              mission_current_[0]);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}


void MavLink::SetMissionCount(const mavlink_mission_count_t &ref) {
  if ((ref.target_system == sys_id_) &&
      (ref.target_component == comp_id_)) {
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        mission_count_upload_ = ref.count;
        mission_upload_index_ = 0;
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        break;
      }
    }
  }
}

void MavLink::SendMissionRequestInt(const std::size_t i) {
  msg_len_ = mavlink_msg_mission_request_int_pack(sys_id_, comp_id_, &msg_,
                                                  sys_id_req_, comp_id_req_,
                                                  i, MAV_MISSION_TYPE_MISSION);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // namespace bfs
