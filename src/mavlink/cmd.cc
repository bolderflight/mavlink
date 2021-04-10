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

#include <array>
#include "mavlink/cmd.h"
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

void MavLinkCmd::MsgHandler(const mavlink_message_t &ref) {
  rx_sys_id_ = ref.sysid; 
  rx_comp_id_ = ref.compid;
  switch (ref.msgid) {
    case MAVLINK_MSG_ID_COMMAND_INT: {
      mavlink_command_int_t cmd_int;
      mavlink_msg_command_int_decode(&ref, &cmd_int);
      CmdIntHandler(cmd_int);
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd_long;
      mavlink_msg_command_long_decode(&ref, &cmd_long);
      CmdLongHandler(cmd_long);
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_CANCEL: {
      mavlink_command_cancel_t cmd_cancel;
      mavlink_msg_command_cancel_decode(&ref, &cmd_cancel);
      CmdCancelHandler(cmd_cancel);
      break;
    }
  }
}
void MavLinkCmd::CmdIntHandler(const mavlink_command_int_t & ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    if (cmd_ == -1) {
      new_cmd_ = true;
      cmd_ = ref.command;
      cancel_cmd_ = false;
      cmd_item_.param[0] = static_cast<double>(ref.param1);
      cmd_item_.param[1] = static_cast<double>(ref.param2);
      cmd_item_.param[2] = static_cast<double>(ref.param3);
      cmd_item_.param[3] = static_cast<double>(ref.param4);
      cmd_item_.param[4] = static_cast<double>(ref.x);
      cmd_item_.param[5] = static_cast<double>(ref.y);
      cmd_item_.param[6] = static_cast<double>(ref.z);
    } else {
      SendCmdAck(MAV_RESULT_TEMPORARILY_REJECTED, 255);
    }
  }
}
void MavLinkCmd::CmdLongHandler(const mavlink_command_long_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    if (cmd_ == -1) {
      new_cmd_ = true;
      cmd_ = ref.command;
      cancel_cmd_ = false;
      cmd_item_.param[0] = static_cast<double>(ref.param1);
      cmd_item_.param[1] = static_cast<double>(ref.param2);
      cmd_item_.param[2] = static_cast<double>(ref.param3);
      cmd_item_.param[3] = static_cast<double>(ref.param4);
      cmd_item_.param[4] = static_cast<double>(ref.param5);
      cmd_item_.param[5] = static_cast<double>(ref.param6);
      cmd_item_.param[6] = static_cast<double>(ref.param7);
    } else {
      SendCmdAck(MAV_RESULT_TEMPORARILY_REJECTED, 255);
    }
  }
}
void MavLinkCmd::CmdCancelHandler(const mavlink_command_cancel_t &ref) {
  if ((ref.target_system == sys_id_) && (ref.target_component == comp_id_)) {
    if (ref.command == cmd_) {
      cancel_cmd_ = true;
    }
  }
}
void MavLinkCmd::CmdComplete() {
  SendCmdAck(MAV_RESULT_ACCEPTED, 255);
  cmd_ = -1;
}
void MavLinkCmd::CmdInProgress(int8_t progress) {
  if ((progress >= 0) && (progress <= 100)) {
    SendCmdAck(MAV_RESULT_IN_PROGRESS, static_cast<uint8_t>(progress));
  } else {
    SendCmdAck(MAV_RESULT_IN_PROGRESS, 255);
  }
}
void MavLinkCmd::CmdInvalid() {
  SendCmdAck(MAV_RESULT_DENIED, 255);
  cmd_ = -1;
}
void MavLinkCmd::CmdUnsupported() {
  SendCmdAck(MAV_RESULT_UNSUPPORTED, 255);
  cmd_ = -1;
}
void MavLinkCmd::CmdCancelled() {
  SendCmdAck(MAV_RESULT_CANCELLED, 255);
  cmd_ = -1;
}
void MavLinkCmd::SendCmdAck(const uint8_t result, const uint8_t progress) {
  int32_t result_param2 = 0;
  msg_len_ = mavlink_msg_command_ack_pack(sys_id_, comp_id_, &msg_,
      cmd_, result, progress, result_param2, rx_sys_id_, rx_comp_id_);
}

}  // namespace bfs
