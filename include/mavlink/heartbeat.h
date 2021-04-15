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

#ifndef INCLUDE_MAVLINK_HEARTBEAT_H_
#define INCLUDE_MAVLINK_HEARTBEAT_H_

#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

enum class VehicleType : uint8_t {
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
enum class VehicleMode : uint8_t {
  MANUAL,
  STABALIZED,
  ATTITUDE,
  AUTO,
  TEST
};
enum class VehicleState : uint8_t {
  UNINIT = 0,
  BOOT = 1,
  CALIBRATING = 2,
  STANDBY = 3,
  ACTIVE = 4,
  CAUTION = 5,
  EMERGENCY = 6,
  POWEROFF = 7,
  FTS = 8
};

class MavLinkHeartbeat {
 public:
  MavLinkHeartbeat(HardwareSerial *bus, const VehicleType type) : bus_(bus),
    vehicle_type_(type) {}
  MavLinkHeartbeat(HardwareSerial *bus, const VehicleType type,
                   const uint8_t sys_id) : bus_(bus),
                   vehicle_type_(type), sys_id_(sys_id) {}
  /* Vehicle type, system and component ID getters */
  inline constexpr VehicleType vehicle_type() const {return vehicle_type_;}
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* 
  * Setters and getters for the throttle enabled flag, vehicle mode,
  * and vehicle state
  */
  inline void throttle_enabled(const bool val) {throttle_enabled_ = val;}
  inline void vehicle_mode(const VehicleMode val) {vehicle_mode_ = val;}
  inline void vehicle_state(const VehicleState val) {vehicle_state_ = val;}
  /* Update method */
  void Update();

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  const uint8_t sys_id_ = 1;
  const VehicleType vehicle_type_;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  static const uint8_t autopilot_ = MAV_AUTOPILOT_GENERIC_MISSION_FULL;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Data */
  bool throttle_enabled_ = false;
  VehicleMode vehicle_mode_ = VehicleMode::MANUAL;
  VehicleState vehicle_state_ = VehicleState::STANDBY;
  /* Timing */
  static constexpr int HEARTBEAT_PERIOD_MS_ = 1000;
  elapsedMillis heartbeat_timer_ms_;
  /* Heartbeat */
  void SendHeartbeat();
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_HEARTBEAT_H_
