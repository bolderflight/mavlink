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

#include "mavlink/mission.h"
#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

/*
* TO DO: test, add other types (fence, rally) , see what frames / cmds are
* sent from QGC and MP. Transform to common frame.
*/



void MavLinkMission::Update() {
  if ((mission_upload_index_ >= 0) &&
      (mission_upload_index_ < mission_upload_count_)) {
    if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
      SendMissionRequestInt(mission_upload_index_, MissionType::MISSION);
      upload_timer_ms_ = 0;
      retries_++;
      if (retries_ > MAX_RETRIES_) {
        mission_upload_count_ = 0;
        mission_upload_index_ = -1;
        retries_ = 0;
        upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
      }
    }
  }
}
void MavLinkMission::MsgHandler(const mavlink_message_t &ref) {
  rx_sys_id_ = ref.sysid;
  rx_comp_id_ = ref.compid;
  switch (ref.msgid) {
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t mission_count;
      mavlink_msg_mission_count_decode(&ref, &mission_count);
      MissionCountHandler(mission_count);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      mavlink_mission_item_t mission_item;
      mavlink_msg_mission_item_decode(&ref, &mission_item);
      MissionItemHandler(mission_item);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t mission_item_int;
      mavlink_msg_mission_item_int_decode(&ref, &mission_item_int);
      MissionItemIntHandler(mission_item_int);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t mission_request_list;
      mavlink_msg_mission_request_list_decode(&ref, &mission_request_list);
      MessageRequestListHandler(mission_request_list);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      mavlink_mission_request_t mission_request;
      mavlink_msg_mission_request_decode(&ref, &mission_request);
      MissionRequestHandler(mission_request);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      mavlink_mission_request_int_t mission_request_int;
      mavlink_msg_mission_request_int_decode(&ref, &mission_request_int);
      MissionRequestIntHandler(mission_request_int);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
      mavlink_mission_set_current_t mission_set_current;
      mavlink_msg_mission_set_current_decode(&ref, &mission_set_current);
      MissionSetCurrentHandler(mission_set_current);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
      mavlink_mission_clear_all_t mission_clear_all;
      mavlink_msg_mission_clear_all_decode(&ref, &mission_clear_all);
      MissionClearAllHandler(mission_clear_all);
      break;
    }
  }
}
void MavLinkMission::AdvanceWaypoint() {
  SendMissionItemReached(mission_current_index_);
  if (mission_current_index_ < mission_current_count_ - 1) {
    mission_current_index_++;
  }
  SendMissionCurrent(mission_current_index_);
}
void MavLinkMission::MessageRequestListHandler(
                     const mavlink_mission_request_list_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    switch (mission_type) {
      case MissionType::MISSION: {
        SendMissionCount(mission_current_count_, mission_type);
        break;
      }
    }
  }
}
void MavLinkMission::MissionCountHandler(const mavlink_mission_count_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    switch (mission_type) {
      case MissionType::MISSION: {
        if (ref.count <= mission_size_) {
          mission_upload_count_ = ref.count;
          mission_upload_index_ = 0;
        } else {
          SendMissionAck(MissionResult::NO_SPACE, mission_type);
        }
        break;
      }
    }
  }
}
void MavLinkMission::MissionRequestHandler(
                     const mavlink_mission_request_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    SendMissionItemInt(ref.seq, mission_type);
  }
}
void MavLinkMission::MissionRequestIntHandler(
                     const mavlink_mission_request_int_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    SendMissionItemInt(ref.seq, mission_type);
  }
}
void MavLinkMission::MissionItemHandler(const mavlink_mission_item_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    retries_ = 0;
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    /* Parse the item */
    MissionItem item;
    item.autocontinue = ref.autocontinue;
    // if ((ref.frame == MAV_FRAME_GLOBAL) ||
    //     (ref.frame == MAV_FRAME_LOCAL_NED) ||
    //     (ref.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
      item.frame = ref.frame;
    // } else {
    //   SendMissionAck(MissionResult::UNSUPPORTED_FRAME, mission_type);
    // }
    item.cmd = static_cast<MissionCmd>(ref.command);
    item.param1 = ref.param1;
    item.param2 = ref.param2;
    item.param3 = ref.param3;
    item.param4 = ref.param4;
    item.x = static_cast<int32_t>(ref.x * 10000000.0f);
    item.y = static_cast<int32_t>(ref.y * 10000000.0f);
    item.z = ref.z;
    /* 
    * Check that the sequence is valid, assign to the correct array,
    * step the index, and send an ack. If the last item, switch the
    * active back and reset the upload index, else reset the timeout
    * to immediately request the next item.
    */
    switch (mission_type) {
      case MissionType::MISSION: {
        if (ref.seq == mission_upload_index_) {
          temp_[mission_upload_index_++] = item;
          /* Just received the last item */
          if (mission_upload_index_ == mission_upload_count_) {
            mission_updated_  = true;
            mission_current_index_ = 0;
            mission_current_count_ = mission_upload_count_;
            memcpy(mission_, temp_, mission_size_);
            mission_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MissionResult::ACCEPTED, mission_type);
          }
        } else {
          SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
        }
        break;
      }
    }
  }
}
void MavLinkMission::MissionItemIntHandler(
                     const mavlink_mission_item_int_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    // Serial.println("MISSION ITEM");
    retries_ = 0;
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    /* Parse the item */
    MissionItem item;
    item.autocontinue = ref.autocontinue;
    // if ((ref.frame == MAV_FRAME_GLOBAL) ||
    //     (ref.frame == MAV_FRAME_LOCAL_NED) ||
    //     (ref.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
      item.frame = ref.frame;
    // } else {
    //   SendMissionAck(MissionResult::UNSUPPORTED_FRAME, mission_type);
    // }
    item.cmd = static_cast<MissionCmd>(ref.command);
    item.param1 = ref.param1;
    item.param2 = ref.param2;
    item.param3 = ref.param3;
    item.param4 = ref.param4;
    item.x = ref.x;
    item.y = ref.y;
    item.z = ref.z;
    /* 
    * Check that the sequence is valid, assign to the correct array,
    * step the index, and send an ack. If the last item, switch the
    * active back and reset the upload index, else reset the timeout
    * to immediately request the next item.
    */
    switch (mission_type) {
      case MissionType::MISSION: {
        if (ref.seq == mission_upload_index_) {
          temp_[mission_upload_index_++] = item;
          /* Just received the last item */
          if (mission_upload_index_ == mission_upload_count_) {
            mission_updated_  = true;
            mission_current_index_ = 0;
            mission_current_count_ = mission_upload_count_;
            memcpy(mission_, temp_, mission_size_ * sizeof(MissionItem));
            mission_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MissionResult::ACCEPTED, mission_type);
          }
        } else {
          SendMissionAck(MissionResult::INVALID_SEQUENCE, mission_type);
        }
        break;
      }
    }
  }
}
void MavLinkMission::MissionSetCurrentHandler(
                     const mavlink_mission_set_current_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    if (ref.seq < mission_current_count_) {
      mission_current_index_ = ref.seq;
      SendMissionCurrent(mission_current_index_);
    } else {
      util_.SendStatusText(bfs::Severity::NOTICE,
                           "Invalid mission sequence number.");
    }
  }
}
void MavLinkMission::MissionClearAllHandler(
                     const mavlink_mission_clear_all_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    MissionType mission_type = static_cast<MissionType>(ref.mission_type);
    switch (mission_type) {
      case MissionType::MISSION: {
        mission_current_index_ = -1;
        mission_current_count_ = 0;
        break;
      }
      case MissionType::ALL: {
        mission_current_index_ = -1;
        mission_current_count_ = 0;
      }
    }
    SendMissionAck(MissionResult::ACCEPTED, mission_type);
  }
}
/* Message emitters */
void MavLinkMission::SendMissionRequestInt(const std::size_t index,
                                           const MissionType type) {
  uint8_t mission_type = static_cast<uint8_t>(type);
  msg_len_ = mavlink_msg_mission_request_int_pack(sys_id_, comp_id_, &msg_,
                                                  rx_sys_id_, rx_comp_id_,
                                                  index, mission_type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionCount(const std::size_t count,
                                      const MissionType type) {
  uint8_t mission_type = static_cast<uint8_t>(type);
  msg_len_ = mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg_,
                                            rx_sys_id_, rx_comp_id_,
                                            count, mission_type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionItemInt(const std::size_t index,
                                        const MissionType type) {
  MissionItem mission_item;
  bool current;
  switch (type) {
    case MissionType::MISSION: {
      if (index < mission_current_count_) {
        mission_item = mission_[index];
        current = (index == mission_current_index_) ? true : false;
      } else {
        SendMissionAck(MissionResult::ERROR, type);
        return;
      }
      break;
    }
  }
  uint8_t mission_type = static_cast<uint8_t>(type);
  uint8_t frame = static_cast<uint8_t>(mission_item.frame);
  uint16_t cmd = static_cast<uint16_t>(mission_item.cmd);
  msg_len_ = mavlink_msg_mission_item_int_pack(sys_id_, comp_id_, &msg_,
                                                rx_sys_id_, rx_comp_id_,
                                                index, frame, cmd,
                                                current,
                                                mission_item.autocontinue,
                                                mission_item.param1,
                                                mission_item.param2,
                                                mission_item.param3,
                                                mission_item.param4,
                                                mission_item.x,
                                                mission_item.y,
                                                mission_item.z,
                                                mission_type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionCurrent(const std::size_t index) {
  msg_len_ = mavlink_msg_mission_current_pack(sys_id_, comp_id_, &msg_, index);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionItemReached(const std::size_t index) {
  msg_len_ = mavlink_msg_mission_item_reached_pack(sys_id_, comp_id_, &msg_,
                                                   index);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionAck(const MissionResult result,
                                    const MissionType type) {
  uint8_t mission_result = static_cast<uint8_t>(result);
  uint8_t mission_type = static_cast<uint8_t>(type);
  msg_len_ = mavlink_msg_mission_ack_pack(sys_id_, comp_id_, &msg_,
                                          rx_sys_id_, rx_comp_id_,
                                          mission_result, mission_type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // namespace bfs
