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

#ifndef MAVLINK_SRC_HEARTBEAT_H_  // NOLINT
#define MAVLINK_SRC_HEARTBEAT_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

namespace bfs {

enum AircraftType : int8_t {
  FIXED_WING = 0,
  HELICOPTER = 1,
  MULTIROTOR = 2,
  VTOL = 3
};

enum AircraftState : int8_t {
  INIT = 0,
  STANDBY = 1,
  ACTIVE = 2,
  CAUTION = 3,
  EMERGENCY = 4,
  FTS = 5
};

enum AircraftMode : int8_t {
  MANUAL = 0,
  STABALIZED = 1,
  ATTITUDE = 2,
  AUTO = 3,
  TEST = 4
};

class MavLinkHeartbeat {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void aircraft_type(const int8_t type) {aircraft_type_ = type;}
  inline void sys_id(const uint8_t sys_id) {sys_id_ = sys_id;}
  inline void comp_id(const uint8_t comp_id) {comp_id_ = comp_id;}
  /* Aircraft type, system and component ID getters */
  inline int8_t aircraft_type() const {return aircraft_type_;}
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* 
  * Setters for the throttle enabled flag, aircraft mode, and aircraft state
  */
  inline void throttle_enabled(const bool val) {throttle_enabled_ = val;}
  inline void aircraft_mode(const int8_t val) {aircraft_mode_ = val;}
  inline void aircraft_state(const int8_t val) {aircraft_state_ = val;}
  /* Update method */
  void Update();

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  int8_t aircraft_type_;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  static constexpr uint8_t autopilot_ = MAV_AUTOPILOT_GENERIC;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Data */
  bool throttle_enabled_ = false;
  int8_t aircraft_mode_ = MANUAL;
  int8_t aircraft_state_ = INIT;
  /* Timing */
  static constexpr uint16_t HEARTBEAT_PERIOD_MS_ = 1000;
  elapsedMillis heartbeat_timer_ms_;
  /* Heartbeat */
  void SendHeartbeat();
  /* Heartbeat variables */
  static constexpr uint32_t custom_mode_ = 0;
  uint8_t type_;
  uint8_t mode_;
  uint8_t state_;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_HEARTBEAT_H_ NOLINT
