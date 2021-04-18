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

#ifndef INCLUDE_MAVLINK_UTIL_H_
#define INCLUDE_MAVLINK_UTIL_H_

#include <string>
#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

enum class Severity {
  EMERGENCY = 0,
  ALERT = 1,
  CRITICAL = 2,
  ERROR = 3,
  WARNING = 4,
  NOTICE = 5,
  INFO = 6,
  DEBUG = 7
};

class MavLinkUtil {
 public:
  explicit MavLinkUtil(HardwareSerial *bus) : bus_(bus) {}
  MavLinkUtil(HardwareSerial *bus, const uint8_t sys_id) :
              bus_(bus), sys_id_(sys_id) {}
  /* System and component ID getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* Send status text */
  void SendStatusText(Severity severity, char const *msg) {
    uint8_t sev = static_cast<uint8_t>(severity);
    uint16_t id = 0;
    uint8_t chunk_seq = 0;
    msg_len_ = mavlink_msg_statustext_pack(sys_id_, comp_id_, &msg_,
                                           sev, msg,
                                           id, chunk_seq);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }

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
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_UTIL_H_
