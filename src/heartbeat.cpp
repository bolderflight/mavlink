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

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include "heartbeat.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

namespace bfs {

void MavLinkHeartbeat::Update() {
  if (heartbeat_timer_ms_ > HEARTBEAT_PERIOD_MS_) {
    SendHeartbeat();
    heartbeat_timer_ms_ = 0;
  }
}

void MavLinkHeartbeat::SendHeartbeat() {
  switch (aircraft_type_) {
    case AircraftType::FIXED_WING: {
      type_ = MAV_TYPE_FIXED_WING;
      break;
    }
    case AircraftType::HELICOPTER: {
      type_ = MAV_TYPE_HELICOPTER;
      break;
    }
    case AircraftType::MULTIROTOR: {
      type_ = MAV_TYPE_QUADROTOR;
      break;
    }
    case AircraftType::VTOL: {
      type_ = MAV_TYPE_VTOL_TILTROTOR;
      break;
    }
  }
  mode_ = 0;
  if (throttle_enabled_) {
    mode_ |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  switch (aircraft_mode_) {
    case AircraftMode::MANUAL: {
      mode_ |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    }
    case AircraftMode::STABALIZED: {
      mode_ |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case AircraftMode::ATTITUDE: {
      mode_ |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      break;
    }
    case AircraftMode::AUTO: {
      mode_ |= MAV_MODE_FLAG_STABILIZE_ENABLED;
      mode_ |= MAV_MODE_FLAG_GUIDED_ENABLED;
      break;
    }
    case AircraftMode::TEST: {
      mode_ |= MAV_MODE_FLAG_TEST_ENABLED;
      break;
    }
  }
  switch (aircraft_state_) {
    case AircraftState::INIT: {
      state_ = MAV_STATE_BOOT;
      break;
    }
    case AircraftState::STANDBY: {
      state_ = MAV_STATE_STANDBY;
      break;
    }
    case AircraftState::ACTIVE: {
      state_ = MAV_STATE_ACTIVE;
      break;
    }
    case AircraftState::CAUTION: {
      state_ = MAV_STATE_CRITICAL;
      break;
    }
    case AircraftState::EMERGENCY: {
      state_ = MAV_STATE_EMERGENCY;
      break;
    }
    case AircraftState::FTS: {
      state_ = MAV_STATE_FLIGHT_TERMINATION;
      break;
    }
  }
  msg_len_ = mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg_,
                                        type_, autopilot_, mode_,
                                        custom_mode_, state_);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // namespace bfs
