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

#ifndef INCLUDE_MAVLINK_MISSION_H_
#define INCLUDE_MAVLINK_MISSION_H_

#include <array>
#include <string>
#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/util.h"

namespace bfs {

/* Coordinate system */
enum class CoordSys : uint8_t {
  WGS84_MSL_ALT = 0,
  NED = 1,
  MISSION = 2,
  WGS84_REL_ALT = 3,
  ENU = 4,
  WGS84_AGL_ALT = 10,
  BODY = 12,
  LOCAL_FRD = 20,
  LOCAL_FLU = 21
};
/* Command */
enum class MissionCmd : uint16_t {};
/* Mission item */
struct MissionItem {
  bool autocontinue;
  int frame;
  MissionCmd cmd;
  float param1;
  float param2;
  float param3;
  float param4;
  int32_t x;
  int32_t y;
  int32_t z;
};

class MavLinkMission {
 public:
  MavLinkMission(HardwareSerial *bus, MissionItem * const mission,
                 MissionItem * const temp, const std::size_t size) :
                 bus_(bus), mission_(mission), temp_(temp),
                 mission_size_(size), util_(bus) {}
  MavLinkMission(HardwareSerial *bus, const uint8_t sys_id,
                 MissionItem * const mission, MissionItem * const temp,
                 const std::size_t size) : bus_(bus), mission_(mission),
                 temp_(temp), mission_size_(size), sys_id_(sys_id),
                 util_(bus, sys_id) {}
  /* System and component ID getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* Update and message handler methods */
  void Update();
  void MsgHandler(const mavlink_message_t &ref);
  /* Waypoints */
  inline bool waypoints_updated() {
    bool status = mission_updated_;
    mission_updated_ = false;
    return status;
  }
  inline int32_t active_waypoint() const {return mission_current_index_;}
  inline std::size_t num_waypoints() const {return mission_current_count_;}
  void AdvanceWaypoint();

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Storage */
  MissionItem * const mission_, * const temp_;
  std::size_t mission_size_;
  /* Config */
  const uint8_t sys_id_ = 1;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Util class, for status text */
  MavLinkUtil util_;
  /* Mission types */
  enum class MissionType : uint8_t {
    MISSION = 0,
    FENCE = 1,
    RALLY = 2,
    ALL = 255
  };
  /* Mission result */
  enum class MissionResult : uint8_t {
    ACCEPTED = 0,
    ERROR = 1,
    UNSUPPORTED_FRAME = 2,
    UNSUPPORTED_CMD = 3,
    NO_SPACE = 4,
    INVALID = 5,
    INVALID_PARAM1 = 6,
    INVALID_PARAM2 = 7,
    INVALID_PARAM3 = 8,
    INVALID_PARAM4 = 9,
    INVALID_PARAM5 = 10,
    INVALID_PARAM6 = 11,
    INVALID_PARAM7 = 12,
    INVALID_SEQUENCE = 13,
    DENIED = 14,
    OPERATION_CANCELLED = 15
  };
  /* State tracking */
  std::size_t mission_current_count_ = 0;
  std::size_t mission_upload_count_ = 0;
  int32_t mission_current_index_ = -1;
  int32_t mission_upload_index_ = -1;
  bool mission_updated_ = false;
  /* Timing */
  elapsedMillis upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
  static constexpr int32_t UPLOAD_TIMEOUT_MS_ = 250;
  std::size_t retries_ = 0;
  static constexpr std::size_t MAX_RETRIES_ = 5;
  /* Message handlers */
  uint8_t rx_sys_id_, rx_comp_id_;
  int cmd_ = -1;
  void MessageRequestListHandler(const mavlink_mission_request_list_t &ref);
  void MissionCountHandler(const mavlink_mission_count_t &ref);
  void MissionRequestHandler(const mavlink_mission_request_t &ref);
  void MissionRequestIntHandler(const mavlink_mission_request_int_t &ref);
  void MissionItemHandler(const mavlink_mission_item_t &ref);
  void MissionItemIntHandler(const mavlink_mission_item_int_t &ref);
  void MissionSetCurrentHandler(const mavlink_mission_set_current_t &ref);
  void MissionClearAllHandler(const mavlink_mission_clear_all_t &ref);
  /* Message emitters */
  void SendMissionRequestInt(const std::size_t index, const MissionType type);
  void SendMissionCount(const std::size_t count, const MissionType type);
  void SendMissionItemInt(const std::size_t index, const MissionType type);
  void SendMissionCurrent(const std::size_t index);
  void SendMissionItemReached(const std::size_t index);
  void SendMissionAck(const MissionResult result, const MissionType type);
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_MISSION_H_
