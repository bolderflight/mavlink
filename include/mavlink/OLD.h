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
enum class MissionCmd : uint16_t {

};
/* Mission item */
struct MissionItem {
  bool autocontinue;
  CoordSys frame;
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
                 bus_(bus), mission_(mission),temp_(temp), util_(bus) {}
  MavLinkMission(HardwareSerial *bus, MissionItem * const mission,
                 MissionItem * const temp, const std::size_t size,
                 const uint8_t sys_id) : bus_(bus), mission_(mission),
                 temp_(temp), sys_id_(sys_id), util_(bus, sys_id) {}
  /* System and component ID getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}




  // /* Get waypoints */
  // inline std::array<MissionItem, MISSION_N> waypoints() const {return mission_[mission_bank_current_];}
  // inline bool waypoints_updated() const {
  //   bool status = mission_updated_;
  //   mission_updated_ = false;
  //   return status;
  // }
  // inline int active_waypoint() const {return mission_current_index_;}
//   /* Setters */
//   inline void WaypointReached() {
//     SendMissionItemReached(mission_current_index_);
//   }
//   void AdvanceWaypoint() {
//     SendMissionItemReached(mission_current_index_);
//     if (mission_current_index_ < mission_current_count_ - 1) {
//       mission_current_index_++;
//     }
//     SendMissionCurrent(mission_current_index_);
//   }
//   void Update() {
//     if ((mission_upload_index_ >= 0) &&
//         (mission_upload_index_ < mission_upload_count_)) {
//       if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
//         SendMissionRequestInt(mission_upload_index_, MissionType::MISSION);
//         upload_timer_ms_ = 0;
//       }
//     }
//     if ((fence_upload_index_ >= 0) &&
//         (fence_upload_index_ < fence_upload_count_)) {
//       if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
//         SendMissionRequestInt(fence_upload_index_, MissionType::FENCE);
//         upload_timer_ms_ = 0;
//       }
//     }
//     if ((rally_upload_index_ >= 0) &&
//         (rally_upload_index_ < rally_upload_count_)) {
//       if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
//         SendMissionRequestInt(rally_upload_index_, MissionType::RALLY);
//         upload_timer_ms_ = 0;
//       }
//     }
//   }
//   void MsgHandler(const mavlink_message_t &ref) {
//     rx_sys_id_ = ref.sysid; 
//     rx_comp_id_ = ref.compid;
//     switch (ref.msgid) {
//       case MAVLINK_MSG_ID_MISSION_COUNT: {
//         mavlink_mission_count_t mission_count;
//         mavlink_msg_mission_count_decode(&ref, &mission_count);
//         MissionCountHandler(mission_count);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_ITEM: {
//         mavlink_mission_item_t mission_item;
//         mavlink_msg_mission_item_decode(&ref, &mission_item);
//         MissionItemHandler(mission_item);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
//         mavlink_mission_item_int_t mission_item_int;
//         mavlink_msg_mission_item_int_decode(&ref, &mission_item_int);
//         MissionItemIntHandler(mission_item_int);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
//         mavlink_mission_request_list_t mission_request_list;
//         mavlink_msg_mission_request_list_decode(&ref, &mission_request_list);
//         MessageRequestListHandler(mission_request_list);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_REQUEST: {
//         mavlink_mission_request_t mission_request;
//         mavlink_msg_mission_request_decode(&ref, &mission_request);
//         MissionRequestHandler(mission_request);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
//         mavlink_mission_request_int_t mission_request_int;
//         mavlink_msg_mission_request_int_decode(&ref, &mission_request_int);
//         MissionRequestIntHandler(mission_request_int);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
//         mavlink_mission_set_current_t mission_set_current;
//         mavlink_msg_mission_set_current_decode(&ref, &mission_set_current);
//         MissionSetCurrentHandler(mission_set_current);
//         break;
//       }
//       case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
//         mavlink_mission_clear_all_t mission_clear_all;
//         mavlink_msg_mission_clear_all_decode(&ref, &mission_clear_all);
//         MissionClearAllHandler(mission_clear_all);
//         break;
//       }
//     }
//   }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Storage */
  MissionItem * const mission_, * const temp_;
  std::size_t size_;
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
//   /*
//   * Arrays of mission items, each stored in 2 banks. One bank remains the
//   * current set that the aircraft is flying, which the other is available
//   * for uploads. Once an upload is complete the bank is switched and the
//   * other bank is available for uploads. An int is used to track the index
//   * of the current bank.
//   */
//   std::array<std::array<MissionItem, 2>, MISSION_N> mission_;
//   std::array<std::array<MissionItem, 2>, FENCE_N> fence_;
//   std::array<std::array<MissionItem, 2>, RALLY_N> rally_;
//   /* 
//   * Using bools for this, since it's super easy to toggle and
//   * they cast to int
//   */
//   bool mission_bank_current_ = 0;
//   bool fence_bank_current_ = 0;
//   bool rally_bank_current_ = 0;
//   /* State tracking */
//   std::size_t mission_current_count_ = 0;
//   std::size_t mission_upload_count_ = 0;
//   std::size_t fence_current_count_ = 0;
//   std::size_t fence_upload_count_ = 0;
//   std::size_t rally_current_count_ = 0;
//   std::size_t rally_upload_count_ = 0;
//   int mission_current_index_ = -1;
//   int mission_upload_index_ = -1;
//   int fence_current_index_ = -1;
//   int fence_upload_index_ = -1;
//   int rally_current_index_ = -1;
//   int rally_upload_index_ = -1;
//   bool mission_updated_ = false;
//   bool fence_updated_ = false;
//   bool rally_updated_ = false;
  /* Timing */
  elapsedMillis upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
  static constexpr int32_t UPLOAD_TIMEOUT_MS_ = 250;
  int retries_ = 0;
  static constexpr int MAX_RETRIES_ = 5;
//  /* Message handlers */
//   uint8_t rx_sys_id_, rx_comp_id_;
//   void MessageRequestListHandler(const mavlink_mission_request_list_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       switch (mission_type) {
//         case MissionType::MISSION: {
//           SendMissionCount(mission_current_count_, mission_type);
//           break;
//         }
//         case MissionType::FENCE: {
//           SendMissionCount(fence_current_count_, mission_type);
//           break;
//         }
//         case MissionType::RALLY: {
//           SendMissionCount(rally_current_count_, mission_type);
//           break;
//         }
//       }
//     }
//   }
//   void MissionCountHandler(const mavlink_mission_count_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       switch (mission_type) {
//         case MissionType::MISSION: {
//           mission_upload_count_ = ref.count;
//           mission_upload_index_ = 0;
//           break;
//         }
//         case MissionType::FENCE: {
//           fence_upload_count_ = ref.count;
//           fence_upload_index_ = 0;
//           break;
//         }
//         case MissionType::RALLY: {
//           rally_upload_count_ = ref.count;
//           rally_upload_index_ = 0;
//           break;
//         }
//       }
//     }
//   }
//   void MissionRequestHandler(const mavlink_mission_request_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       SendMissionItemInt(ref.seq, mission_type);
//     }
//   }
//   void MissionRequestIntHandler(const mavlink_mission_request_int_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       SendMissionItemInt(ref.seq, mission_type);
//     }
//   }
//   void MissionItemHandler(const mavlink_mission_item_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       /* Parse the item */
//       MissionItem item;
//       item.autocontinue = ref.autocontinue;
//       if ((ref.frame == MAV_FRAME_GLOBAL) ||
//           (ref.frame == MAV_FRAME_LOCAL_NED) ||
//           (ref.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
//         item.frame = static_cast<CoordSys>(ref.frame);
//       } else {
//         SendMissionAck(MissionResult::UNSUPPORTED_FRAME, mission_type);
//       }
//       item.cmd = static_cast<MissionCmd>(ref.command);
//       item.param1 = ref.param1;
//       item.param2 = ref.param2;
//       item.param3 = ref.param3;
//       item.param4 = ref.param4;
//       item.x = static_cast<int32_t>(ref.x * 10000000.0f);
//       item.y = static_cast<int32_t>(ref.y * 10000000.0f);
//       item.z = ref.z;
//       /* 
//       * Check that the sequence is valid, assign to the correct array,
//       * step the index, and send an ack. If the last item, switch the
//       * active back and reset the upload index, else reset the timeout
//       * to immediately request the next item.
//       */
//       switch (mission_type) {
//         case MissionType::MISSION: {
//           if (ref.seq == mission_upload_index_) {
//             bool upload_bank = !mission_bank_current_;
//             mission_[upload_bank][mission_upload_index_++] = item;
//             /* Just received the last item */
//             if (mission_upload_index_ == mission_upload_count_) {
//               mission_updated_  = true;
//               mission_current_index_ = 0;
//               mission_current_count_ = mission_upload_count_;
//               mission_bank_current_ = upload_bank;
//               mission_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//         case MissionType::FENCE: {
//           if (ref.seq == fence_upload_index_) {
//             bool upload_bank = !fence_bank_current_;
//             fence_[upload_bank][fence_upload_index_++] = item;
//             /* Just received the last item */
//             if (fence_upload_index_ == fence_upload_count_) {
//               fence_updated_  = true;
//               fence_current_index_ = 0;
//               fence_current_count_ = fence_upload_count_;
//               fence_bank_current_ = upload_bank;
//               fence_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//         case MissionType::RALLY: {
//           if (ref.seq == rally_upload_index_) {
//             bool upload_bank = !rally_bank_current_;
//             rally_[upload_bank][rally_upload_index_++] = item;
//             /* Just received the last item */
//             if (rally_upload_index_ == rally_upload_count_) {
//               rally_updated_  = true;
//               rally_current_index_ = 0;
//               rally_current_count_ = rally_upload_count_;
//               rally_bank_current_ = upload_bank;
//               rally_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//       }
//     }
//   }
//   void MissionItemIntHandler(const mavlink_mission_item_int_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       /* Parse the item */
//       MissionItem item;
//       item.autocontinue = ref.autocontinue;
//       if ((ref.frame == MAV_FRAME_GLOBAL) ||
//           (ref.frame == MAV_FRAME_LOCAL_NED) ||
//           (ref.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
//         item.frame = static_cast<CoordSys>(ref.frame);
//       } else {
//         SendMissionAck(MissionResult::UNSUPPORTED_FRAME, mission_type);
//       }
//       item.cmd = static_cast<MissionCmd>(ref.command);
//       item.param1 = ref.param1;
//       item.param2 = ref.param2;
//       item.param3 = ref.param3;
//       item.param4 = ref.param4;
//       item.x = ref.x;
//       item.y = ref.y;
//       item.z = ref.z;
//       /* 
//       * Check that the sequence is valid, assign to the correct array,
//       * step the index, and send an ack. If the last item, switch the
//       * active back and reset the upload index, else reset the timeout
//       * to immediately request the next item.
//       */
//       switch (mission_type) {
//         case MissionType::MISSION: {
//           if (ref.seq == mission_upload_index_) {
//             bool upload_bank = !mission_bank_current_;
//             mission_[upload_bank][mission_upload_index_++] = item;
//             /* Just received the last item */
//             if (mission_upload_index_ == mission_upload_count_) {
//               mission_updated_  = true;
//               mission_current_index_ = 0;
//               mission_current_count_ = mission_upload_count_;
//               mission_bank_current_ = upload_bank;
//               mission_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//         case MissionType::FENCE: {
//           if (ref.seq == fence_upload_index_) {
//             bool upload_bank = !fence_bank_current_;
//             fence_[upload_bank][fence_upload_index_++] = item;
//             /* Just received the last item */
//             if (fence_upload_index_ == fence_upload_count_) {
//               fence_updated_  = true;
//               fence_current_index_ = 0;
//               fence_current_count_ = fence_upload_count_;
//               fence_bank_current_ = upload_bank;
//               fence_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//         case MissionType::RALLY: {
//           if (ref.seq == rally_upload_index_) {
//             bool upload_bank = !rally_bank_current_;
//             rally_[upload_bank][rally_upload_index_++] = item;
//             /* Just received the last item */
//             if (rally_upload_index_ == rally_upload_count_) {
//               rally_updated_  = true;
//               rally_current_index_ = 0;
//               rally_current_count_ = rally_upload_count_;
//               rally_bank_current_ = upload_bank;
//               rally_upload_index_ = -1;
//               upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
//               SendMissionAck(MissionResult::ACCEPTED, mission_type);
//             }            
//           } else {
//             SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
//           }
//           break;
//         }
//       }
//     }
//   }
//   void MissionSetCurrentHandler(const mavlink_mission_set_current_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       if (ref.seq < mission_current_count_) {
//         mission_current_index_ = ref.seq;
//         SendMissionCurrent(mission_current_index_);
//       } else {
//         util_.SendStatusText(bfs::Severity::NOTICE, "Invalid mission sequence number.");
//       }
//     }
//   }
//   void MissionClearAllHandler(const mavlink_mission_clear_all_t &ref) {
//     if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
//       MissionType mission_type = static_cast<MissionType>(ref.mission_type);
//       switch (mission_type) {
//         case MissionType::MISSION: {
//           mission_current_index_ = -1;
//           mission_current_count_ = 0;
//           break;
//         }
//         case MissionType::FENCE: {
//           fence_current_index_ = -1;
//           fence_current_count_ = 0;
//           break;
//         }
//         case MissionType::RALLY: {
//           rally_current_index_ = -1;
//           rally_current_count_ = 0;
//           break;
//         }
//         case MissionType::ALL: {
//           mission_current_index_ = -1;
//           mission_current_count_ = 0;
//           fence_current_index_ = -1;
//           fence_current_count_ = 0;
//           rally_current_index_ = -1;
//           rally_current_count_ = 0;
//         }
//       }
//       SendMissionAck(MissionResult::ACCEPTED, mission_type);
//     }
//   }
//   /* Message emitters */
//   void SendMissionRequestInt(const std::size_t index, const MissionType type) {
//     uint8_t mission_type = static_cast<uint8_t>(type);
//     msg_len_ = mavlink_msg_mission_request_int_pack(sys_id_, comp_id_, &msg_,
//                                                     rx_sys_id_, rx_comp_id_,
//                                                     index, mission_type);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
//   void SendMissionCount(const std::size_t count, const MissionType type) {
//     uint8_t mission_type = static_cast<uint8_t>(type);
//     msg_len_ = mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg_,
//                                               rx_sys_id_, rx_comp_id_,
//                                               count, mission_type);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
//   void SendMissionItemInt(const std::size_t index, const MissionType type) {
//     MissionItem mission_item;
//     bool current;
//     switch (type) {
//       case MissionType::MISSION: {
//         if (index < mission_current_count_) {
//           mission_item = mission_[mission_bank_current_][index];
//           current = (index == mission_current_index_) ? true : false;
//         } else {
//           SendMissionAck(MissionResult::ERROR, type);
//           return;
//         }
//         break;
//       }
//       case MissionType::FENCE: {
//         if (index < fence_current_count_) {
//           mission_item = fence_[fence_bank_current_][index];
//           current = (index == fence_current_index_) ? true : false;
//         } else {
//           SendMissionAck(MissionResult::ERROR, type);
//           return;
//         }
//         break;
//       }
//       case MissionType::RALLY: {
//         if (index < rally_current_count_) {
//           mission_item = rally_[rally_bank_current_][index];
//           current = (index == rally_current_index_) ? true : false;
//         } else {
//           SendMissionAck(MissionResult::ERROR, type);
//           return;
//         }
//         break;
//       }
//     }
//     uint8_t mission_type = static_cast<uint8_t>(type);
//     uint8_t frame = static_cast<uint8_t>(mission_item.frame);
//     uint16_t cmd = static_cast<uint16_t>(mission_item.cmd);
//     msg_len_ = mavlink_msg_mission_item_int_pack(sys_id_, comp_id_, &msg_,
//                                                  rx_sys_id_, rx_comp_id_,
//                                                  index, frame, cmd,
//                                                  current,
//                                                  mission_item.autocontinue,
//                                                  mission_item.param1,
//                                                  mission_item.param2,
//                                                  mission_item.param3,
//                                                  mission_item.param4,
//                                                  mission_item.x,
//                                                  mission_item.y,
//                                                  mission_item.z,
//                                                  mission_type);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
//   void SendMissionCurrent(const std::size_t index) {
//     msg_len_ = mavlink_msg_mission_current_pack(sys_id_, comp_id_, &msg_,
//                                                 index);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
//   void SendMissionItemReached(const std::size_t index) {
//     msg_len_ = mavlink_msg_mission_item_reached_pack(sys_id_, comp_id_, &msg_,
//                                                      index);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
//   void SendMissionAck(const MissionResult result, const MissionType type) {
//     uint8_t mission_result = static_cast<uint8_t>(result);
//     uint8_t mission_type = static_cast<uint8_t>(type);
//     msg_len_ = mavlink_msg_mission_ack_pack(sys_id_, comp_id_, &msg_,
//                                             rx_sys_id_, rx_comp_id_,
//                                             mission_result, mission_type);
//     mavlink_msg_to_send_buffer(msg_buf_, &msg_);
//     bus_->write(msg_buf_, msg_len_);
//   }
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_MISSION_H_
