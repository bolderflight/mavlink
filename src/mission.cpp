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

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include "mission.h"  // NOLINT
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "util.h"  // NOLINT

namespace bfs {

void MavLinkMission::Update() {
  if ((mission_upload_index_ >= 0) &&
      (mission_upload_index_ < mission_upload_count_)) {
    if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
      SendMissionRequestInt(mission_upload_index_, MAV_MISSION_TYPE_MISSION);
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
  if ((fence_upload_index_ >= 0) &&
      (fence_upload_index_ < fence_upload_count_)) {
    if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
      SendMissionRequestInt(fence_upload_index_, MAV_MISSION_TYPE_FENCE);
      upload_timer_ms_ = 0;
      retries_++;
      if (retries_ > MAX_RETRIES_) {
        fence_upload_count_ = 0;
        fence_upload_index_ = -1;
        retries_ = 0;
        upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
      }
    }
  }
  if ((rally_upload_index_ >= 0) &&
      (rally_upload_index_ < rally_upload_count_)) {
    if (upload_timer_ms_ > UPLOAD_TIMEOUT_MS_) {
      SendMissionRequestInt(rally_upload_index_, MAV_MISSION_TYPE_RALLY);
      upload_timer_ms_ = 0;
      retries_++;
      if (retries_ > MAX_RETRIES_) {
        rally_upload_count_ = 0;
        rally_upload_index_ = -1;
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
      mavlink_msg_mission_count_decode(&ref, &mission_count_);
      MissionCountHandler(mission_count_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      mavlink_msg_mission_item_decode(&ref, &mission_item_);
      MissionItemHandler(mission_item_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_msg_mission_item_int_decode(&ref, &mission_item_int_);
      MissionItemIntHandler(mission_item_int_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_msg_mission_request_list_decode(&ref, &mission_request_list_);
      MessageRequestListHandler(mission_request_list_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      mavlink_msg_mission_request_decode(&ref, &mission_request_);
      MissionRequestHandler(mission_request_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      mavlink_msg_mission_request_int_decode(&ref, &mission_request_int_);
      MissionRequestIntHandler(mission_request_int_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
      mavlink_msg_mission_set_current_decode(&ref, &mission_set_current_);
      MissionSetCurrentHandler(mission_set_current_);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
      mavlink_msg_mission_clear_all_decode(&ref, &mission_clear_all_);
      MissionClearAllHandler(mission_clear_all_);
      break;
    }
  }
}
void MavLinkMission::AdvanceMissionItem() {
  SendMissionItemReached(mission_current_index_);
  if (mission_current_index_ < mission_current_count_ - 1) {
    mission_current_index_++;
  }
  SendMissionCurrent(mission_current_index_);
}
void MavLinkMission::MessageRequestListHandler(
                     const mavlink_mission_request_list_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        SendMissionCount(mission_current_count_, ref.mission_type);
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        SendMissionCount(fence_current_count_, ref.mission_type);
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        SendMissionCount(rally_current_count_, ref.mission_type);
        break;
      }
    }
  }
}
void MavLinkMission::MissionCountHandler(const mavlink_mission_count_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    /* Ack an empty count */
    if (ref.count == 0) {
      SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
      return;
    }
    /* Otherwise set the upload count and start getting mission items */
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        if (ref.count <= mission_size_) {
          mission_upload_count_ = ref.count;
          mission_upload_index_ = 0;
        } else {
          SendMissionAck(MAV_MISSION_NO_SPACE, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        if (ref.count <= fence_size_) {
          fence_upload_count_ = ref.count;
          fence_upload_index_ = 0;
        } else {
          SendMissionAck(MAV_MISSION_NO_SPACE, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        if (ref.count <= rally_size_) {
          rally_upload_count_ = ref.count;
          rally_upload_index_ = 0;
        } else {
          SendMissionAck(MAV_MISSION_NO_SPACE, ref.mission_type);
        }
        break;
      }
    }
  }
}
void MavLinkMission::MissionRequestHandler(
                     const mavlink_mission_request_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    SendMissionItemInt(ref.seq, ref.mission_type);
  }
}
void MavLinkMission::MissionRequestIntHandler(
                     const mavlink_mission_request_int_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    SendMissionItemInt(ref.seq, ref.mission_type);
  }
}
void MavLinkMission::MissionItemHandler(const mavlink_mission_item_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    retries_ = 0;
    /* Parse the item */
    item_.autocontinue = ref.autocontinue;
    item_.frame = ref.frame;
    item_.cmd = ref.command;
    item_.param1 = ref.param1;
    item_.param2 = ref.param2;
    item_.param3 = ref.param3;
    item_.param4 = ref.param4;
    item_.x = static_cast<int32_t>(ref.x * 10000000.0f);
    item_.y = static_cast<int32_t>(ref.y * 10000000.0f);
    item_.z = ref.z;
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        if (ref.seq == mission_upload_index_) {
          temp_[mission_upload_index_++] = item_;
          /* Just received the last item */
          if (mission_upload_index_ == mission_upload_count_) {
            mission_updated_  = true;
            mission_current_index_ = 0;
            mission_current_count_ = mission_upload_count_;
            memcpy(mission_, temp_,
                   mission_upload_count_ * sizeof(MissionItem));
            mission_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(mission_upload_index_, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        if (ref.seq == fence_upload_index_) {
          temp_[fence_upload_index_++] = item_;
          /* Just received the last item */
          if (fence_upload_index_ == fence_upload_count_) {
            fence_updated_  = true;
            fence_current_count_ = fence_upload_count_;
            memcpy(fence_, temp_, fence_upload_count_ * sizeof(MissionItem));
            fence_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(fence_upload_index_, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        if (ref.seq == rally_upload_index_) {
          temp_[rally_upload_index_++] = item_;
          /* Just received the last item */
          if (rally_upload_index_ == rally_upload_count_) {
            rally_updated_  = true;
            rally_current_count_ = rally_upload_count_;
            memcpy(rally_, temp_, rally_upload_count_ * sizeof(MissionItem));
            rally_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(rally_upload_index_, ref.mission_type);
        }
        break;
      }
    }
  }
}
void MavLinkMission::MissionItemIntHandler(
                     const mavlink_mission_item_int_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    retries_ = 0;
    /* Parse the item */
    item_.autocontinue = ref.autocontinue;
    item_.frame = ref.frame;
    item_.cmd = ref.command;
    item_.param1 = ref.param1;
    item_.param2 = ref.param2;
    item_.param3 = ref.param3;
    item_.param4 = ref.param4;
    item_.x = ref.x;
    item_.y = ref.y;
    item_.z = ref.z;
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        if (ref.seq == mission_upload_index_) {
          temp_[mission_upload_index_++] = item_;
          /* Just received the last item */
          if (mission_upload_index_ == mission_upload_count_) {
            mission_updated_  = true;
            mission_current_index_ = 0;
            mission_current_count_ = mission_upload_count_;
            memcpy(mission_, temp_,
                   mission_upload_count_ * sizeof(MissionItem));
            mission_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(mission_upload_index_, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        if (ref.seq == fence_upload_index_) {
          temp_[fence_upload_index_++] = item_;
          /* Just received the last item */
          if (fence_upload_index_ == fence_upload_count_) {
            fence_updated_  = true;
            fence_current_count_ = fence_upload_count_;
            memcpy(fence_, temp_, fence_upload_count_ * sizeof(MissionItem));
            fence_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(fence_upload_index_, ref.mission_type);
        }
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        if (ref.seq == rally_upload_index_) {
          temp_[rally_upload_index_++] = item_;
          /* Just received the last item */
          if (rally_upload_index_ == rally_upload_count_) {
            rally_updated_  = true;
            rally_current_count_ = rally_upload_count_;
            memcpy(rally_, temp_, rally_upload_count_ * sizeof(MissionItem));
            rally_upload_index_ = -1;
            upload_timer_ms_ = UPLOAD_TIMEOUT_MS_;
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
          if (use_mission_planner_) {
            SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
          }
        } else {
          SendMissionRequestInt(rally_upload_index_, ref.mission_type);
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
    switch (ref.mission_type) {
      case MAV_MISSION_TYPE_MISSION: {
        mission_current_index_ = -1;
        mission_current_count_ = 0;
        break;
      }
      case MAV_MISSION_TYPE_FENCE: {
        fence_current_count_ = 0;
        break;
      }
      case MAV_MISSION_TYPE_RALLY: {
        rally_current_count_ = 0;
        break;
      }
      case MAV_MISSION_TYPE_ALL: {
        mission_current_index_ = -1;
        mission_current_count_ = 0;
        fence_current_count_ = 0;
        rally_current_count_ = 0;
      }
    }
    SendMissionAck(MAV_MISSION_ACCEPTED, ref.mission_type);
  }
}
/* Message emitters */
void MavLinkMission::SendMissionRequestInt(const std::size_t index,
                                           const uint8_t type) {
  msg_len_ = mavlink_msg_mission_request_int_pack(sys_id_, comp_id_, &msg_,
                                                  rx_sys_id_, rx_comp_id_,
                                                  index, type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionCount(const std::size_t count,
                                      const uint8_t type) {
  msg_len_ = mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg_,
                                            rx_sys_id_, rx_comp_id_,
                                            count, type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionItemInt(const std::size_t index,
                                        const uint8_t type) {
  switch (type) {
    case MAV_MISSION_TYPE_MISSION: {
      if (index < mission_current_count_) {
        item_ = mission_[index];
        current_ = (index == mission_current_index_) ? true : false;
      } else {
        SendMissionAck(MAV_MISSION_ERROR, type);
        return;
      }
      break;
    }
    case MAV_MISSION_TYPE_FENCE: {
      if (index < fence_current_count_) {
        item_ = fence_[index];
        current_ = false;
      } else {
        SendMissionAck(MAV_MISSION_ERROR, type);
        return;
      }
      break;
    }
    case MAV_MISSION_TYPE_RALLY: {
      if (index < rally_current_count_) {
        item_ = rally_[index];
        current_ = false;
      } else {
        SendMissionAck(MAV_MISSION_ERROR, type);
        return;
      }
      break;
    }
  }
  msg_len_ = mavlink_msg_mission_item_int_pack(sys_id_, comp_id_, &msg_,
                                                rx_sys_id_, rx_comp_id_,
                                                index,
                                                item_.frame,
                                                item_.cmd,
                                                current_,
                                                item_.autocontinue,
                                                item_.param1,
                                                item_.param2,
                                                item_.param3,
                                                item_.param4,
                                                item_.x,
                                                item_.y,
                                                item_.z,
                                                type);
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
void MavLinkMission::SendMissionAck(const uint8_t result,
                                    const uint8_t type) {
  msg_len_ = mavlink_msg_mission_ack_pack(sys_id_, comp_id_, &msg_,
                                          rx_sys_id_, rx_comp_id_,
                                          result, type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}
void MavLinkMission::SendMissionChanged(const uint8_t type) {
  msg_len_ = mavlink_msg_mission_changed_pack(sys_id_, comp_id_, &msg_,
                                              -1, -1,
                                              sys_id_, comp_id_, type);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // namespace bfs
