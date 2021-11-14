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

#ifndef INCLUDE_MAVLINK_RTK_H_
#define INCLUDE_MAVLINK_RTK_H_

#include "core/core.h"
#include "./mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

template<std::size_t N>
class MavLinkRtk {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void gnss_serial(HardwareSerial *bus) {gnss_bus_ = bus;}
  /* Update and message handler methods */
  void MsgHandler(const mavlink_message_t &ref) {
    switch (ref.msgid) {
      case MAVLINK_MSG_ID_GPS_RTCM_DATA: {
        mavlink_msg_gps_rtcm_data_decode(&ref, &rtcm_data_);
        RtcmDataHandler(rtcm_data_);
        break;
      }
    }
  }

 private:
   /* Serial bus */
  HardwareSerial *bus_;
  /* Bus to gnss receiver */
  HardwareSerial *gnss_bus_;
  /* Message buffer */
  mavlink_message_t msg_;
  /* Message data */
  mavlink_gps_rtcm_data_t rtcm_data_;
  /* Message handler */
  void RtcmDataHandler(const mavlink_gps_rtcm_data_t &ref) {
    /* Parse the message flags */
    fragmented_ = 0x01 & ref.flags;
    fragment_id_ = static_cast<int8_t>(ref.flags >> 1 & 0x03);
    sequence_id_ = static_cast<int8_t>(ref.flags >> 3 & 0x1F);
    Serial.print(fragmented_);
    Serial.print("\t");
    Serial.print(fragment_id_);
    Serial.print("\t");
    Serial.print(ref.len);
    Serial.print("\t");
    Serial.println(sequence_id_);
    /* Trivial case where we have an unfragmented message */
    if (!fragmented_) {
      Serial.print("PARSED MESSAGE, LEN: ");
      Serial.println(ref.len);
      // gnss_bus_->write(ref.data, ref.len);
    /* Need to reconstruct fragmented messages */
    } else {
      parsing_ = false;
      clear_to_parse_ = false;
      for (buf_idx_ = 0; buf_idx_ < N; buf_idx_++) {
        /* Check if we're already parsing the message */
        if (sequence_id_ == rtcm_msg_buf_[buf_idx_].id) {
          parsing_ = true;
          clear_to_parse_ = true;
          break;
        }
      }
      if (!parsing_) {
        /* Find an empty buffer slot */
        for (buf_idx_ = 0; buf_idx_ < N; buf_idx_++) {
          if (rtcm_msg_buf_[buf_idx_].id == EMPTY_) {
            clear_to_parse_ = true;
            break;
          }
        }
      }
      if (!clear_to_parse_) {
        /* Clear out the most stale buffer */
        max_dist_ = 0;
        for (buf_idx_ = 0; buf_idx_ < N; buf_idx_++) {
          dist_ = (sequence_id_ - rtcm_msg_buf_[buf_idx_].id) & ((1 << 5) - 1);
          if (dist_ > max_dist_) {
            max_dist_ = dist_;
            stale_idx_ = buf_idx_;
          }
        }
        buf_idx_ = stale_idx_;
        /* Reset the message states */
        rtcm_msg_buf_[buf_idx_].id = EMPTY_;
        for (std::size_t i = 0; i < NUM_FRAGMENTS_; i++) {
          rtcm_msg_buf_[buf_idx_].len[i] = 0;
        }
      }
      /* Set the fragment length and data */
      rtcm_msg_buf_[buf_idx_].id = sequence_id_;
      rtcm_msg_buf_[buf_idx_].len[fragment_id_] = ref.len;
      memcpy(rtcm_msg_buf_[buf_idx_].data + fragment_id_ * MAVLINK_MSG_LEN_,
              ref.data, ref.len);
      /* Check if the message is fully-reconstructed */
      if (ref.len < MAVLINK_MSG_LEN_) {
        reconstructed_ = true;
        msg_len_ = rtcm_msg_buf_[buf_idx_].len[fragment_id_];
        for (int32_t i = fragment_id_ - 1; i >= 0; i--) {
          msg_len_ += rtcm_msg_buf_[buf_idx_].len[i];
          if (rtcm_msg_buf_[buf_idx_].len[i] == 0) {
            reconstructed_ = false;
            break;
          }
        }
        if (reconstructed_) {
          /* Send the message */
          // gnss_bus_->write(rtcm_msg_buf_[buf_idx_].data, msg_len_);
          Serial.print("PARSED MESSAGE, LEN: ");
          Serial.println(msg_len_);
          /* Reset the message states */
          rtcm_msg_buf_[buf_idx_].id = EMPTY_;
          for (std::size_t i = 0; i < NUM_FRAGMENTS_; i++) {
            rtcm_msg_buf_[buf_idx_].len[i] = 0;
          }
        }
      }
    }
  }
  /* Max MAV Link message length */
  static constexpr std::size_t MAVLINK_MSG_LEN_ = 180;
  /* Max number of fragments */
  static constexpr std::size_t NUM_FRAGMENTS_ = 4;
  /* Sequence ID used to indicate an empty buffer slot */
  static constexpr int8_t EMPTY_ = -1;
  /* Up to 4 messages may be used to reconstruct an RTCM message */
  struct RtcmMsg {
    int8_t id = EMPTY_;                               // sequence id
    uint8_t len[NUM_FRAGMENTS_] = {0};                // length of each fragment
    uint8_t data[NUM_FRAGMENTS_ * MAVLINK_MSG_LEN_];  // buffer to hold data
  };
  /* Array of message buffers */
  RtcmMsg rtcm_msg_buf_[N];
  /* Message parsing */
  bool fragmented_;
  bool parsing_;
  bool reconstructed_;
  bool clear_to_parse_;
  int8_t fragment_id_;
  int8_t sequence_id_;
  std::size_t msg_len_;
  std::size_t buf_idx_;
  /* Stale buffer handling */
  int8_t dist_, max_dist_;
  std::size_t stale_idx_;
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_RTK_H_
