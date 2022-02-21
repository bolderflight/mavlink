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

#ifndef MAVLINK_SRC_RTCM_H_  // NOLINT
#define MAVLINK_SRC_RTCM_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

namespace bfs {

class MavLinkRtcm {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void gnss_serial(HardwareSerial *bus) {gnss_bus_ = bus;}
  /* Update and message handler methods */
  void MsgHandler(const mavlink_message_t &ref);

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Bus to gnss receiver */
  HardwareSerial *gnss_bus_ = NULL;
  /* Message buffer */
  mavlink_message_t msg_;
  /* Message data */
  mavlink_gps_rtcm_data_t rtcm_data_;
  /* Message handler */
  void RtcmDataHandler(const mavlink_gps_rtcm_data_t &ref);
  /* Max MAV Link message length */
  static constexpr std::size_t MAVLINK_MSG_LEN_ = 180;
  /* Max number of fragments */
  static constexpr std::size_t NUM_FRAGMENTS_ = 4;
  /* RTCM message buffer */
  uint8_t len_[NUM_FRAGMENTS_];
  uint8_t data_[NUM_FRAGMENTS_ * MAVLINK_MSG_LEN_];
  /* Message parsing */
  bool fragmented_;
  bool reconstructed_;
  int8_t fragment_id_;
  int8_t sequence_id_;
  int8_t prev_sequence_id_ = -1;
  std::size_t msg_len_;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_RTCM_H_ NOLINT
