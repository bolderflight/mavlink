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

#ifndef MAVLINK_SRC_UTIL_H_  //NOLINT
#define MAVLINK_SRC_UTIL_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <string>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

namespace bfs {

enum class Severity : uint8_t {
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
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void sys_id(const uint8_t sys_id) {sys_id_ = sys_id;}
  inline void comp_id(const uint8_t comp_id) {comp_id_ = comp_id;}
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* Send status text */
  template<std::size_t N>
  void SendStatusText(Severity severity, const char(&msg)[N]) {
    static_assert(N < 51, "Maximum message length is 50");
    char text[50] = {0};
    memcpy(text, msg, N);
    msg_len_ = mavlink_msg_statustext_pack(sys_id_, comp_id_, &msg_,
                                           static_cast<uint8_t>(severity),
                                           text,
                                           id_, chunk_seq_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Status text variables */
  static constexpr uint16_t id_ = 0;
  static constexpr uint8_t chunk_seq_ = 0;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_UTIL_H_ NOLINT
