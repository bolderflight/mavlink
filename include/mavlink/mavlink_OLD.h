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

#include <array>
#include <string>
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

class MavLink {
 public:
  enum class VehicleType {
    FIXED_WING = 1,
    QUADROTOR = 2,
    COAXIAL_HELICOPTER = 3,
    HELICOPTER = 4,
    HEXAROTOR = 13,
    OCTOROTOR = 14,
    TRICOPTER = 15,
    VTOL = 21,
    DODECAROTOR = 29,
    DECAROTOR = 35
  };
  enum class VehicleMode {
    MANUAL,
    STABALIZED,
    ATTITUDE,
    AUTO,
    TEST
  };
  enum class VehicleState {
    STANDBY = 3,
    ACTIVE = 4,
    CAUTION = 5,
    EMERGENCY = 6,
    FTS = 8
  };
  enum class GnssFix : uint8_t {
    NO_GNSS = 0,
    FIX_NONE = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGPS = 4,
    FIX_RTK_FLOAT = 5,
    FIX_RTK_FIXED = 6,
    FIX_STATIC = 7,
    FIX_PPP = 8
  };
  MavLink(HardwareSerial *bus, const VehicleType type) : bus_(bus),
    vehicle_type_(type) {}
  MavLink(HardwareSerial *bus, const VehicleType type, const uint8_t sys_id) :
    bus_(bus), vehicle_type_(type), sys_id_(sys_id) {}
  void Begin(uint32_t baud) {bus_->begin(baud);}
  void Update();
  /* System */
  inline void throttle_enabled(const bool val) {throttle_enabled_ = val;}
  inline void vehicle_mode(const VehicleMode val) {vehicle_mode_ = val;}
  inline void vehicle_state(const VehicleState val) {vehicle_state_ = val;}
  inline void sys_time_s(const double val) {sys_time_s_ = val;}
  inline void cpu_load(const float val) {cpu_load_ = val;}
  inline void diff_pres_installed(const bool val) {diff_pres_installed_ = val;}
  inline void imu_healthy(const bool val) {imu_healthy_ = val;}
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
  inline void diff_pres_pa(const float val) {static_pres_pa_ = val;}
  inline void static_pres_die_temp_c(const float val) {static_pres_die_temp_c_ = val;}
  /* GNSS data */
  inline void gnss_fix(const GnssFix val) {gnss_fix_ = val;}
  inline void gnss_num_sats(const uint8_t val) {
    gnss_num_sv_.set = true;
    gnss_num_sv_.val = val;
  }
  inline void gnss_lat_rad(const double val) {gnss_lat_rad_ = val;}
  inline void gnss_lon_rad(const double val) {gnss_lon_rad_ = val;}
  inline void gnss_alt_msl_m(const float val) {gnss_alt_msl_m_= val;}
  inline void gnss_alt_wgs84_m(const float val) {gnss_alt_wgs84_m_ = val;}
  inline void gnss_north_vel_mps(const float val) {gnss_north_vel_mps_ = val;}
  inline void gnss_east_vel_mps(const float val) {gnss_east_vel_mps_ = val;}
  inline void gnss_down_vel_mps(const float val) {gnss_down_vel_mps_ = val;}
  inline void gnss_horz_acc_m(const float val) {gnss_horz_acc_m_ = val;}
  inline void gnss_vert_acc_m(const float val) {gnss_vert_acc_m_ = val;}
  inline void gnss_vel_acc_mps(const float val) {gnss_vel_acc_mps_ = val;}
  /* Estimation data */
  inline void nav_lat_rad(const double val) {nav_lat_rad_ = val;}
  inline void nav_lon_rad(const double val) {nav_lon_rad_ = val;}
  inline void nav_alt_msl_m(const float val) {nav_alt_msl_m_ = val;}
  inline void nav_alt_agl_m(const float val) {nav_alt_agl_m_ = val;}
  inline void nav_north_vel_mps(const float val) {nav_north_vel_mps_ = val;}
  inline void nav_east_vel_mps(const float val) {nav_east_vel_mps_ = val;}
  inline void nav_down_vel_mps(const float val) {nav_down_vel_mps_ = val;}
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
  /* Parameters */
  static constexpr std::size_t NUM_PARAMS = 24;
  inline std::array<float, NUM_PARAMS> parameters() const {
    std::array<float, NUM_PARAMS> ret;
    for (std::size_t i = 0; i < NUM_PARAMS; i++) {
      ret[i] = params_[i].val;
    }
    return ret;
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  const uint8_t sys_id_ = 1;
  const VehicleType vehicle_type_;
  const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  const uint8_t autopilot_ = MAV_AUTOPILOT_ARDUPILOTMEGA;
  uint8_t sys_id_req_;
  uint8_t comp_id_req_;
  /* Message buffer */
  mavlink_message_t msg_;
  mavlink_status_t status_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  uint8_t chan_ = 0;
  /* Data */
  template<typename T>
  struct CondData {
    T val;
    bool set = false;
  };
  /* System */
  double sys_time_s_ = 0;
  bool throttle_enabled_ = false;
  VehicleMode vehicle_mode_ = VehicleMode::MANUAL;
  VehicleState vehicle_state_ = VehicleState::STANDBY;
  float cpu_load_ = 0;
  bool diff_pres_installed_ = false;
  bool imu_healthy_ = false;
  bool static_pres_healthy_ = false;
  bool diff_pres_healthy_ = false;
  bool gnss_healthy_ = false;
  bool inceptor_healthy_ = false;
  CondData<float> battery_volt_;
  /* IMU */
  float imu_accel_x_mps2_ = 0, imu_accel_y_mps2_ = 0, imu_accel_z_mps2_ = 0;
  float imu_gyro_x_radps_ = 0, imu_gyro_y_radps_ = 0, imu_gyro_z_radps_ = 0;
  float imu_mag_x_ut_ = 0, imu_mag_y_ut_ = 0, imu_mag_z_ut_ = 0;
  CondData<float> imu_die_temp_c_;
  /* Airdata */
  float static_pres_pa_ = 0;
  float diff_pres_pa_ = 0;
  float static_pres_die_temp_c_ = 0;
  /* GNSS */
  CondData<uint8_t> gnss_num_sv_;
  GnssFix gnss_fix_ = GnssFix::NO_GNSS;
  double gnss_lat_rad_ = 0, gnss_lon_rad_ = 0;
  float gnss_alt_msl_m_ = 0, gnss_alt_wgs84_m_ = 0;
  float gnss_north_vel_mps_ = 0, gnss_east_vel_mps_ = 0;
  float gnss_down_vel_mps_ = 0;
  float gnss_horz_acc_m_ = 0, gnss_vert_acc_m_ = 0;
  float gnss_vel_acc_mps_ = 0;
  /* Nav */
  double nav_lat_rad_ = 0, nav_lon_rad_ = 0;
  float nav_alt_msl_m_ = 0, nav_alt_agl_m_ = 0;
  float nav_north_vel_mps_ = 0, nav_east_vel_mps_ = 0, nav_down_vel_mps_ = 0;
  float nav_ias_mps_ = 0;
  float nav_pitch_rad_ = 0, nav_roll_rad_ = 0;
  CondData<float> nav_hdg_rad_;
  float nav_gyro_x_radps_ = 0, nav_gyro_y_radps_ = 0, nav_gyro_z_radps_ = 0;
  /* Effector */
  std::array<float, 16> effector_ = {0};
  /* RC Input */
  std::array<float, 16> inceptor_ = {0};
  uint8_t throttle_ch_ = 0;
  /* Telemetry Messages */
  void SendHeartbeat();
  /* SRx_ALL */
  void SRx_ALL();
  /* SRx_EXT_STAT */
  void SRx_EXT_STAT();
  void SendSysStatus();
  void SendGpsRawInt();
  /* SRx_EXTRA1 */
  void SRx_EXTRA1();
  void SendAttitude();
  /* SRx_EXTRA2 */
  void SRx_EXTRA2();
  void SendVfrHud();
  /* SRx_EXTRA3 */
  void SRx_EXTRA3();
  void SendBatteryStatus();
  /* SRx_POSITION */
  void SRx_POSITION();
  void SendGlobalPositionInt();
  /* SRx_RAW_SENS */
  void SRx_RAW_SENS();
  void SendRawImu();
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
  static constexpr int HEARTBEAT_PERIOD_MS_ = 1000;
  static constexpr int NUM_DATA_STREAMS_ = 13;
  int data_stream_period_ms_[NUM_DATA_STREAMS_] = {-1, -1, -1, -1, -1, -1, -1,
                                                   -1, -1, -1, -1, -1, -1};
  elapsedMillis heartbeat_timer_ms_;
  elapsedMillis data_stream_timer_ms_[NUM_DATA_STREAMS_];
  /* Data streams */
  typedef void (MavLink::*DataStream)(void);
  DataStream streams_[NUM_DATA_STREAMS_] = {
    &MavLink::SRx_ALL,
    &MavLink::SRx_RAW_SENS,
    &MavLink::SRx_EXT_STAT,
    &MavLink::SRx_RC_CHAN,
    &MavLink::SRx_RAW_CTRL,
    &MavLink::SRx_EMPTY,
    &MavLink::SRx_POSITION,
    &MavLink::SRx_EMPTY,
    &MavLink::SRx_EMPTY,
    &MavLink::SRx_EMPTY,
    &MavLink::SRx_EXTRA1,
    &MavLink::SRx_EXTRA2,
    &MavLink::SRx_EXTRA3
  };
  /* Request data stream */
  void ParseRequestDataStream(const mavlink_request_data_stream_t &ref);
  /* Parameter Microservice */
  struct Param {
    const std::string param_id;
    float val;
    /* Currently only supporting float params */
    static const uint8_t param_type = MAV_PARAM_TYPE_REAL32;
    static const uint16_t param_count = NUM_PARAMS;
    const uint16_t param_index;
  };
  /* Array of params, fixed name and index */
  Param params_[NUM_PARAMS] = {
    {.param_id = "PARAM_00", .param_index = 0},
    {.param_id = "PARAM_01", .param_index = 1},
    {.param_id = "PARAM_02", .param_index = 2},
    {.param_id = "PARAM_03", .param_index = 3},
    {.param_id = "PARAM_04", .param_index = 4},
    {.param_id = "PARAM_05", .param_index = 5},
    {.param_id = "PARAM_06", .param_index = 6},
    {.param_id = "PARAM_07", .param_index = 7},
    {.param_id = "PARAM_08", .param_index = 8},
    {.param_id = "PARAM_09", .param_index = 9},
    {.param_id = "PARAM_10", .param_index = 10},
    {.param_id = "PARAM_11", .param_index = 11},
    {.param_id = "PARAM_12", .param_index = 12},
    {.param_id = "PARAM_13", .param_index = 13},
    {.param_id = "PARAM_14", .param_index = 14},
    {.param_id = "PARAM_15", .param_index = 15},
    {.param_id = "PARAM_16", .param_index = 16},
    {.param_id = "PARAM_17", .param_index = 17},
    {.param_id = "PARAM_18", .param_index = 18},
    {.param_id = "PARAM_19", .param_index = 19},
    {.param_id = "PARAM_20", .param_index = 20},
    {.param_id = "PARAM_21", .param_index = 21},
    {.param_id = "PARAM_22", .param_index = 22},
    {.param_id = "PARAM_23", .param_index = 23}
  };
  /* Sends parameter given an index value */
  void SendParam(const std::size_t i);
  void ParamRequestRead(const mavlink_param_request_read_t &ref);
  void ParamRequestList(const mavlink_param_request_list_t &ref);
  void ParamSet(const mavlink_param_set_t &ref);
  /*
  * Sets up parameter streaming in response to a PARAM_REQUEST_LIST
  * message. One parameter will be sent each PARAM_PERIOD_MS. Default
  * to a param index of -1; when a PARAM_REQUEST_LIST is received,
  * update param index to 0 to start stream and continue until all
  * parameters are sent.
  */
  int param_index_ = -1;
  elapsedMillis param_timer_ms_;
  static constexpr int PARAM_PERIOD_MS_ = 1000;
  /* Mission Microservice */
  enum class MissionType {
    MISSION = 0,
    FENCE = 1,
    RALLY = 2,
    ALL = 255
  };
  struct MissionItem {
    bool autocontinue;
    float param1;
    float param2;
    float param3;
    int32_t lat_dege7;
    int32_t long_dege7;
    float alt_m;
  };
  static constexpr int MAX_MISSION_COUNT = 1000;
  static constexpr int MAX_FENCE_COUNT = 500;
  static constexpr int MAX_RALLY_COUNT = 500;
  std::array<MissionItem, MAX_MISSION_COUNT> mission_active_;
  std::array<MissionItem, MAX_MISSION_COUNT> mission_upload_;
  std::array<MissionItem, MAX_FENCE_COUNT> fence_;
  std::array<MissionItem, MAX_RALLY_COUNT> rally_;
  int mission_current_[3] = {0, 0, 0};
  int mission_count_[3] = {0, 0, 0};






  /* Mission upload */
  int mission_upload_index_ = -1;
  int mission_count_upload_ = 0;
  elapsedMillis mission_timer_ms_;
  static constexpr int MISSION_PERIOD_MS_ = 200;
  void SetMissionCount(const mavlink_mission_count_t &ref);
  void SendMissionRequestInt(const std::size_t i);






  void SetMissionItemInt();

  void MissionRequestList();
  void MissionRequest(const mavlink_mission_request_t &ref);
  void MissionRequestInt(const mavlink_mission_request_int_t &ref);
  void SendMissionItemInt(int index);
  void SendMissionCount();
  void SendMissionCurrent();
  void MissionSetCurrent(const mavlink_mission_set_current_t &ref);
  void MissionClearAll();
  /* Util */
  void SendStatusText();
  template<typename T, std::size_t N>
  bool InRange(const int index, const std::array<T, N> &ref) {
    if ((index >= 0) && (index < N)) {
      return true;
    } else {
      return false;
    }
  }
  bool InRange(const int index, const int max) {
    if ((index >= 0) && (index < max)) {
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_MAVLINK_H_
