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

#ifndef INCLUDE_MAVLINK_CMD_H_
#define INCLUDE_MAVLINK_CMD_H_

#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/mission.h"

namespace bfs {

struct CmdItem {
  MissionCmd cmd;
  double param[7];
};

class MavLinkCmd {
 public:
  MavLinkCmd(HardwareSerial *bus) : bus_(bus) {}
  MavLinkCmd(HardwareSerial *bus, const uint8_t sys_id) : bus_(bus),
                                                          sys_id_(sys_id) {}
  /* Getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* Get cmd */
  inline bool new_cmd() {
    bool status = new_cmd_;
    new_cmd_ = false;
    return status;
  }
  inline bool cancel_cmd() const {return cancel_cmd_;}
  inline CmdItem cmd() {return cmd_item_;}
  /* Cmd status */
  void CmdComplete();
  void CmdInProgress(int8_t progress);
  void CmdInvalid();
  void CmdUnsupported();
  void CmdCancelled();

  void MsgHandler(const mavlink_message_t &ref);
 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  const uint8_t sys_id_ = 1;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* State */
  bool new_cmd_ = false;
  bool cancel_cmd_ = false;
  int cmd_ = -1;
  CmdItem cmd_item_;
  /* Message handlers */
  uint8_t rx_sys_id_, rx_comp_id_;
  void CmdIntHandler(const mavlink_command_int_t & ref);
  void CmdLongHandler(const mavlink_command_long_t &ref);
  void CmdCancelHandler(const mavlink_command_cancel_t &ref);
  /* Message senders */
  void SendCmdAck(const uint8_t result, const uint8_t progress);

};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_CMD_H_
