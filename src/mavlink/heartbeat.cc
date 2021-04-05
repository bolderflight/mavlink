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

#include "mavlink/heartbeat.h"
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"

namespace bfs {

void MavLinkHeartbeat::Update() {
  if (heartbeat_timer_ms_ > HEARTBEAT_PERIOD_MS_) {
    SendHeartbeat();
    heartbeat_timer_ms_ = 0;
  }
}

void MavLinkHeartbeat::SendHeartbeat() {
  uint8_t type = static_cast<uint8_t>(vehicle_type_);
  uint8_t mode = 0;
  if (throttle_enabled_) {
    mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  switch (vehicle_mode_) {
    case VehicleMode::MANUAL: {
      mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    }
    case VehicleMode::STABALIZED: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case VehicleMode::ATTITUDE: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case VehicleMode::AUTO: {
      mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
      break;
    }
    case VehicleMode::TEST: {
      mode |= MAV_MODE_FLAG_TEST_ENABLED;
      break;
    }
  }
  uint32_t custom_mode = 0;
  uint8_t state = static_cast<uint8_t>(vehicle_state_);
  msg_len_ = mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg_,
                                        type, autopilot_, mode,
                                        custom_mode, state);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // namespace bfs
