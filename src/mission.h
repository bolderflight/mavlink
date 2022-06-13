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

#ifndef MAVLINK_SRC_MISSION_H_  // NOLINT
#define MAVLINK_SRC_MISSION_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "util.h"  // NOLINT

namespace bfs {

struct MissionItem {
  bool autocontinue;
  uint8_t frame;
  uint16_t cmd;
  float param1;
  float param2;
  float param3;
  float param4;
  int32_t x;
  int32_t y;
  float z;
};

class MavLinkMission {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {
    bus_ = bus;
    util_.hardware_serial(bus);
  }
  inline void sys_id(const uint8_t sys_id) {
    sys_id_ = sys_id;
    util_.sys_id(sys_id);
  }
  inline void comp_id(const uint8_t comp_id) {
    comp_id_ = comp_id;
    util_.comp_id(comp_id);
  }
  inline void mission(MissionItem * const mission,
                      const std::size_t mission_size,
                      MissionItem * const temp) {
    mission_ = mission;
    mission_size_ = mission_size;
    temp_ = temp;
  }
  inline void fence(MissionItem * const fence, const std::size_t fence_size) {
    fence_ = fence;
    fence_size_ = fence_size;
  }
  inline void rally(MissionItem * const rally, const std::size_t rally_size) {
    rally_ = rally;
    rally_size_ = rally_size;
  }
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* Update and message handler methods */
  void Update();
  void MsgHandler(const mavlink_message_t &ref);
  /* Mission */
  inline bool mission_updated() {
    bool status = mission_updated_;
    mission_updated_ = false;
    return status;
  }
  inline int32_t active_mission_item() const {return mission_current_index_;}
  inline std::size_t num_mission_items() const {return mission_current_count_;}
  inline void num_mission_items(const std::size_t val) {
    if (val > 0) {
      mission_current_index_ = 0;
    } else {
      mission_current_index_ = -1;
    }
    mission_current_count_ = val;
    SendMissionChanged(MAV_MISSION_TYPE_MISSION);
  }
  void AdvanceMissionItem();
  /* Fence */
  inline bool fence_updated() {
    bool status = fence_updated_;
    fence_updated_ = false;
    return status;
  }
  inline std::size_t num_fence_items() const {return fence_current_count_;}
  inline void num_fence_items(const std::size_t val) {
    fence_current_count_ = val;
    SendMissionChanged(MAV_MISSION_TYPE_FENCE);
  }
  /* Rally */
  inline bool rally_points_updated() {
    bool status = rally_updated_;
    rally_updated_ = false;
    return status;
  }
  inline std::size_t num_rally_points() const {return rally_current_count_;}
  inline void num_rally_points(const std::size_t val) {
    rally_current_count_ = val;
    SendMissionChanged(MAV_MISSION_TYPE_RALLY);
  }
  /* Flag to say whether we're working with Mission Planner */
  inline void use_mission_planner(const bool mp) {use_mission_planner_ = mp;}

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Storage */
  MissionItem * mission_, * temp_;
  MissionItem * fence_ = nullptr;
  MissionItem * rally_ = nullptr;
  std::size_t mission_size_;
  std::size_t fence_size_ = 0, rally_size_ = 0;
  /* Config */
  uint8_t sys_id_ = 1;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  bool use_mission_planner_ = false;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Util class, for status text */
  MavLinkUtil util_;
  /* State tracking */
  std::size_t mission_current_count_ = 0;
  std::size_t mission_upload_count_ = 0;
  int32_t mission_current_index_ = -1;
  int32_t mission_upload_index_ = -1;
  bool mission_updated_ = false;
  std::size_t fence_current_count_ = 0;
  std::size_t fence_upload_count_ = 0;
  int32_t fence_upload_index_ = -1;
  bool fence_updated_ = false;
  std::size_t rally_current_count_ = 0;
  std::size_t rally_upload_count_ = 0;
  int32_t rally_upload_index_ = -1;
  bool rally_updated_ = false;
  /* Timing */
  elapsedMillis upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
  static constexpr int32_t UPLOAD_TIMEOUT_MS_ = 250;
  std::size_t retries_ = 0;
  static constexpr std::size_t MAX_RETRIES_ = 5;
  /* Message handlers */
  uint8_t rx_sys_id_, rx_comp_id_;
  void MessageRequestListHandler(const mavlink_mission_request_list_t &ref);
  void MissionCountHandler(const mavlink_mission_count_t &ref);
  void MissionRequestHandler(const mavlink_mission_request_t &ref);
  void MissionRequestIntHandler(const mavlink_mission_request_int_t &ref);
  void MissionItemHandler(const mavlink_mission_item_t &ref);
  void MissionItemIntHandler(const mavlink_mission_item_int_t &ref);
  void MissionSetCurrentHandler(const mavlink_mission_set_current_t &ref);
  void MissionClearAllHandler(const mavlink_mission_clear_all_t &ref);
  /* Message emitters */
  void SendMissionRequestInt(const std::size_t index, const uint8_t type);
  void SendMissionCount(const std::size_t count, const uint8_t type);
  void SendMissionItemInt(const std::size_t index, const uint8_t type);
  void SendMissionCurrent(const std::size_t index);
  void SendMissionItemReached(const std::size_t index);
  void SendMissionAck(const uint8_t result, const uint8_t type);
  void SendMissionChanged(const uint8_t type);
  /* Message data */
  mavlink_mission_count_t mission_count_;
  mavlink_mission_item_t mission_item_;
  mavlink_mission_item_int_t mission_item_int_;
  mavlink_mission_request_list_t mission_request_list_;
  mavlink_mission_request_t mission_request_;
  mavlink_mission_request_int_t mission_request_int_;
  mavlink_mission_set_current_t mission_set_current_;
  mavlink_mission_clear_all_t mission_clear_all_;
  mavlink_mission_changed_t mission_changed_;
  /* Mission item */
  MissionItem item_;
  bool current_;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_MISSION_H_ NOLINT
