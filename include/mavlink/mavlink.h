/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_MAVLINK_MAVLINK_H_
#define INCLUDE_MAVLINK_MAVLINK_H_

#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"

namespace telemetry {

class MavLink {
 public:
  enum VehicleType {
    FIXED_WING = 1,
    QUADROTOR = 2,
    HEXAROTOR = 13,
    OCTOROTOR = 14
  };
  enum DeviceType {
    FULL_SUPPORT = 0,
    WAYPOINTS = 5,
    WAYPOINTS_AND_SIMPLE_NAVIGATION = 6,
    MISSION = 7
  };
  enum Status {

  };
  MavLink(HardwareSerial *bus, uint8_t sys_id, VehicleType veh_type) : bus_(bus), sys_id_(sys_id), veh_type_(static_cast<uint8_t>(veh_type)) {}
  void Begin(uint32_t baud);
  bool Update();
  void SendAttitude(uint32_t time_ms, float roll_rad, float pitch_rad, float yaw_rad, float gx_radps, float gy_radps, float gz_radps);
  void SendGnss(uint32_t time_ms, uint8_t fix, uint8_t num_sv, double lat_rad, double lon_rad, float alt_msl_m, float alt_wgs84_m, float ground_speed_mps, float track_rad, float hacc, float vacc, float sacc, float tacc);
  void SendHud(uint32_t time_ms, float airspeed_mps, float ground_speed_mps, float alt_msl_m, float climb_mps, float heading_rad, float throttle_nd);
  void SendBattery(float voltage);

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_, veh_type_;
  uint8_t dev_type_ = FULL_SUPPORT;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  uint8_t status_ = MAV_STATE_STANDBY;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Heartbeat timing */
  elapsedMillis heartbeat_timer_ = 2000;
  static constexpr int HEARTBEAT_PERIOD_MS_ = 1000;
};

}  // namespace telemetry

#endif  // INCLUDE_MAVLINK_MAVLINK_H_
