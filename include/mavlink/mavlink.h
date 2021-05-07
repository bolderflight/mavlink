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

#include <string>
#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/heartbeat.h"
#include "mavlink/telemetry.h"
#include "mavlink/parameter.h"
#include "mavlink/mission.h"

namespace bfs {

template<std::size_t NPARAM>
class MavLink {
 public:
  MavLink(HardwareSerial *bus, const AircraftType type,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const temp) :
          bus_(bus), util_(bus), heartbeat_(bus, type), telem_(bus),
          param_(bus), mission_(bus, mission, mission_size, temp) {}
  MavLink(HardwareSerial *bus, const AircraftType type, const uint8_t sys_id,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const temp) :
          bus_(bus), sys_id_(sys_id), util_(bus, sys_id),
          heartbeat_(bus, type, sys_id), telem_(bus, sys_id),
          param_(bus, sys_id), mission_(bus, sys_id, mission, mission_size,
                                        temp) {}
  MavLink(HardwareSerial *bus, const AircraftType type,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const fence, const std::size_t fence_size,
          MissionItem * const temp) :
          bus_(bus), util_(bus), heartbeat_(bus, type), telem_(bus),
          param_(bus), mission_(bus, mission, mission_size, fence, fence_size,
                                temp),
          fence_supported_(true) {}
  MavLink(HardwareSerial *bus, const AircraftType type, const uint8_t sys_id,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const fence, const std::size_t fence_size,
          MissionItem * const temp) :
          bus_(bus), sys_id_(sys_id), util_(bus, sys_id),
          heartbeat_(bus, type, sys_id), telem_(bus, sys_id),
          param_(bus, sys_id), mission_(bus, sys_id, mission, mission_size,
                                        fence, fence_size, temp),
          fence_supported_(true) {}
  MavLink(HardwareSerial *bus, const AircraftType type,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const fence, const std::size_t fence_size,
          MissionItem * const rally, const std::size_t rally_size,
          MissionItem * const temp) :
          bus_(bus), util_(bus), heartbeat_(bus, type), telem_(bus),
          param_(bus), mission_(bus, mission, mission_size, fence, fence_size,
                                rally, rally_size, temp),
          fence_supported_(true), rally_supported_(true) {}
  MavLink(HardwareSerial *bus, const AircraftType type, const uint8_t sys_id,
          MissionItem * const mission, const std::size_t mission_size,
          MissionItem * const fence, const std::size_t fence_size,
          MissionItem * const rally, const std::size_t rally_size,
          MissionItem * const temp) :
          bus_(bus), sys_id_(sys_id), util_(bus, sys_id),
          heartbeat_(bus, type, sys_id), telem_(bus, sys_id),
          param_(bus, sys_id), mission_(bus, sys_id, mission, mission_size,
                                        fence, fence_size, rally, rally_size,
                                        temp),
          fence_supported_(true), rally_supported_(true) {}
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
      if (mavlink_frame_char(chan_, bus_->read(), &msg_, &status_)
          != MAVLINK_FRAMING_INCOMPLETE) {
        /* Items handled by this class */
        rx_sys_id_ = msg_.sysid;
        rx_comp_id_ = msg_.compid;
        switch (msg_.msgid) {
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
      }
    }
  }
  /* Vehicle type, system and component ID getters */
  inline constexpr AircraftType aircraft_type() const {
    return heartbeat_.aircraft_type();
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
  inline void aircraft_mode(const AircraftMode val) {
    heartbeat_.aircraft_mode(val);
  }
  inline void aircraft_state(const AircraftState val) {
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
  void SendStatusText(Severity severity, char const *msg) {
    util_.SendStatusText(severity, msg);
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  const uint8_t sys_id_ = 1;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  const bool fence_supported_ = false;
  const bool rally_supported_ = false;
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
  MavLinkParameter<NPARAM> param_;
  MavLinkMission mission_;
  /* Message handlers */
  uint8_t rx_sys_id_, rx_comp_id_;
  uint16_t cmd_;
  /* Message data */
  mavlink_command_long_t cmd_long_;
  static constexpr uint32_t flight_sw_version_ = 0;
  static constexpr uint32_t middleware_sw_version_ = 0;
  static constexpr uint32_t os_sw_version_ = 0;
  static constexpr uint32_t board_version_ = 0;
  static constexpr uint8_t flight_custom_version_[8] = {0};
  static constexpr uint8_t middleware_custom_version_[8] = {0};
  static constexpr uint8_t os_custom_version_[8] = {0};
  static constexpr uint16_t vendor_id_ = 0;
  static constexpr uint16_t product_id_ = 0;
  static constexpr uint64_t uid_ = 0;
  static constexpr uint8_t uid2_[18] = {0};
  static constexpr uint8_t spec_version_hash_[8] = {8};
  static constexpr uint8_t library_versions_hash_[8] = {0};
  static constexpr uint16_t version_ = 230;
  static constexpr uint16_t min_version_ = 100;
  uint64_t capabilities_;
  static constexpr int32_t result_param2_ = 0;
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

#endif  // INCLUDE_MAVLINK_MAVLINK_H_
