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
#include "rtcm.h"  // NOLINT
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

namespace bfs {

void MavLinkRtcm::MsgHandler(const mavlink_message_t &ref) {
  switch (ref.msgid) {
    case MAVLINK_MSG_ID_GPS_RTCM_DATA: {
      mavlink_msg_gps_rtcm_data_decode(&ref, &rtcm_data_);
      RtcmDataHandler(rtcm_data_);
      break;
    }
  }
}

void MavLinkRtcm::RtcmDataHandler(const mavlink_gps_rtcm_data_t &ref) {
  /* Parse the message flags */
  fragmented_ = 0x01 & ref.flags;
  fragment_id_ = static_cast<int8_t>(ref.flags >> 1 & 0x03);
  sequence_id_ = static_cast<int8_t>(ref.flags >> 3 & 0x1F);
  /* Trivial case where we have an unfragmented message */
  if (!fragmented_) {
    if (gnss_bus_) {
      gnss_bus_->write(ref.data, ref.len);
    }
    prev_sequence_id_ = sequence_id_;
  /* Need to reconstruct fragmented messages */
  } else {
    if (sequence_id_ != prev_sequence_id_) {
      /* New message, reset states */
      for (std::size_t i = 0; i < NUM_FRAGMENTS_; i++) {
        len_[i] = 0;
      }
    }
    /* Get the fragment length and data */
    len_[fragment_id_] = ref.len;
    memcpy(data_ + fragment_id_ * MAVLINK_MSG_LEN_, ref.data, ref.len);
    /* See if message complete */
    if (ref.len < MAVLINK_MSG_LEN_) {
      reconstructed_ = true;
      msg_len_ = len_[fragment_id_];
      for (int32_t i = fragment_id_ - 1; i >= 0; i--) {
        msg_len_ += len_[i];
        if (len_[i] == 0) {
          reconstructed_ = false;
          break;
        }
      }
      if (reconstructed_) {
        if (gnss_bus_) {
          gnss_bus_->write(data_, msg_len_);
        }
      }
    }
    prev_sequence_id_ = sequence_id_;
  }
}

}  // namespace bfs
