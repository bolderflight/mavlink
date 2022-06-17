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

#ifndef MAVLINK_SRC_UTM_H_  //NOLINT
#define MAVLINK_SRC_UTM_H_

#if defined(ARDUINO)
#include "Arduino.h"
#include "optional.hpp"  // NOLINT
#else
#include <optional>
#include "core/core.h"
#endif
#include <array>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "units.h"  // NOLINT

namespace bfs {

#if defined(ARDUINO)
using nonstd::optional;
#else
using std::optional;
#endif

enum FlightState : uint8_t {
  FLIGHT_STATE_UNKNOWN = UTM_FLIGHT_STATE_UNKNOWN,
  FLIGHT_STATE_GROUND = UTM_FLIGHT_STATE_GROUND,
  FLIGHT_STATE_AIRBORNE = UTM_FLIGHT_STATE_AIRBORNE,
  FLIGHT_STATE_EMERGENCY = UTM_FLIGHT_STATE_EMERGENCY,
  FLIGHT_STATE_NOCTRL = UTM_FLIGHT_STATE_NOCTRL
};

template<std::size_t N>
class MavLinkUtm {
 public:
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {bus_ = bus;}
  inline void sys_id(const uint8_t sys_id) {sys_id_ = sys_id;}
  inline void comp_id(const uint8_t comp_id) {comp_id_ = comp_id;}
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* Send and receive UTM data */
  void Update() {
    if (send_timeout_ms_ > 0) {
      /* Send the data */
      if (send_timer_ms_ > send_timeout_ms_) {
        SendUtm();
        send_timer_ms_ = 0;
      }
    }
  }
  void MsgHandler(const mavlink_message_t &ref) {
    if (ref.msgid == MAVLINK_MSG_ID_UTM_GLOBAL_POSITION) {
      mavlink_msg_utm_global_position_decode(&ref, &utm_global_pos_);
      UtmMsgHandler(utm_global_pos_);
    }
  }
  /* Config */
  inline void update_period_s(const float val) {
    tx_data_.update_period_s = val;
    send_timeout_ms_ = static_cast<uint32_t>(val * 1000.0f);
  }
  /* Prepare to send UTM data */
  static constexpr std::size_t UAS_ID_LEN = 18;
  inline void unix_time_us(const uint64_t val) {tx_data_.unix_time_us = val;}
  inline void uas_id(const std::array<uint8_t, UAS_ID_LEN> &id) {
    tx_data_.uas_id = id;
  }
  inline void lat_rad(const double val) {tx_data_.lat_rad = val;}
  inline void lon_rad(const double val) {tx_data_.lon_rad = val;}
  inline void alt_wgs84_m(const float val) {tx_data_.alt_wgs84_m = val;}
  inline void rel_alt_m(const float val) {tx_data_.rel_alt_m = val;}
  inline void north_vel_mps(const float val) {tx_data_.north_vel_mps = val;}
  inline void east_vel_mps(const float val) {tx_data_.east_vel_mps = val;}
  inline void down_vel_mps(const float val) {tx_data_.down_vel_mps = val;}
  inline void horz_acc_m(const float val) {tx_data_.horz_acc_m = val;}
  inline void vert_acc_m(const float val) {tx_data_.vert_acc_m = val;}
  inline void vel_acc_mps(const float val) {tx_data_.vel_acc_mps = val;}
  inline void next_lat_rad(const double val) {tx_data_.next_lat_rad = val;}
  inline void next_lon_rad(const double val) {tx_data_.next_lon_rad = val;}
  inline void next_alt_wgs84_m(const float val) {
    tx_data_.next_alt_wgs84_m = val;
  }
  inline void flight_state(const FlightState val) {
    tx_data_.flight_state = val;
  }
  /* Receive UTM data */
  inline std::size_t num_rx() const {return num_rx_;}
  inline void ClearRx() {
    ResetRxData();
    num_rx_ = 0;
  }
  inline optional<uint64_t> get_unix_time_us(const std::size_t idx) const {
    return rx_data_[idx].unix_time_us;
  }
  inline optional<std::array<uint8_t, UAS_ID_LEN>> get_uas_id(
                                                  const std::size_t idx) const {
    return rx_data_[idx].uas_id;
  }
  inline optional<double> get_lat_rad(const std::size_t idx) const {
    return rx_data_[idx].lat_rad;
  }
  inline optional<double> get_lon_rad(const std::size_t idx) const {
    return rx_data_[idx].lon_rad;
  }
  inline optional<float> get_alt_wgs84_m(const std::size_t idx) const {
    return rx_data_[idx].alt_wgs84_m;
  }
  inline optional<float> get_rel_alt_m(const std::size_t idx) const {
    return rx_data_[idx].rel_alt_m;
  }
  inline optional<float> get_north_vel_mps(const std::size_t idx) const {
    return rx_data_[idx].north_vel_mps;
  }
  inline optional<float> get_east_vel_mps(const std::size_t idx) const {
    return rx_data_[idx].east_vel_mps;
  }
  inline optional<float> get_down_vel_mps(const std::size_t idx) const {
    return rx_data_[idx].down_vel_mps;
  }
  inline optional<float> get_horz_acc_m(const std::size_t idx) const {
    return rx_data_[idx].horz_acc_m;
  }
  inline optional<float> get_vert_acc_m(const std::size_t idx) const {
    return rx_data_[idx].vert_acc_m;
  }
  inline optional<float> get_vel_acc_mps(const std::size_t idx) const {
    return rx_data_[idx].vel_acc_mps;
  }
  inline optional<double> get_next_lat_rad(const std::size_t idx) const {
    return rx_data_[idx].next_lat_rad;
  }
  inline optional<double> get_next_lon_rad(const std::size_t idx) const {
    return rx_data_[idx].next_lon_rad;
  }
  inline optional<float> get_next_alt_wgs84_m(const std::size_t idx) const {
    return rx_data_[idx].next_alt_wgs84_m;
  }
  inline float get_update_period_s(const std::size_t idx) const {
    return rx_data_[idx].update_period_s;
  }
  inline FlightState get_flight_state(const std::size_t idx) const {
    return rx_data_[idx].flight_state;
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* UTM data */
  struct UtmData {
    uint8_t data_avail_flags;
    FlightState flight_state;
    float update_period_s;
    optional<float> alt_wgs84_m;
    optional<float> rel_alt_m;
    optional<float> next_alt_wgs84_m;
    optional<float> horz_acc_m;
    optional<float> vert_acc_m;
    optional<float> vel_acc_mps;
    optional<float> north_vel_mps, east_vel_mps, down_vel_mps;
    optional<std::array<uint8_t, UAS_ID_LEN>> uas_id;
    optional<uint64_t> unix_time_us;
    optional<double> lat_rad;
    optional<double> lon_rad;
    optional<double> next_lat_rad;
    optional<double> next_lon_rad;
  };
  /* TX data */
  UtmData tx_data_;
  /* RX data */
  std::size_t num_rx_ = 0;
  std::array<UtmData, N> rx_data_;
  std::array<uint8_t, UAS_ID_LEN> uas_temp_id_;
  /* TX buffer */
  uint64_t time_;
  std::array<uint8_t, UAS_ID_LEN> uas_id_;
  int32_t lat_;
  int32_t lon_;
  int32_t alt_;
  int32_t relative_alt_;
  int16_t vx_;
  int16_t vy_;
  int16_t vz_;
  uint16_t h_acc_;
  uint16_t v_acc_;
  uint16_t vel_acc_;
  int32_t next_lat_;
  int32_t next_lon_;
  int32_t next_alt_;
  uint16_t update_rate_;
  uint8_t flight_state_;
  uint8_t flags_;
  /* Message buffer */
  mavlink_utm_global_position_t utm_global_pos_;
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Timer to send data */
  int32_t send_timeout_ms_ = -1;
  elapsedMillis send_timer_ms_;
  /* Helper functions */
  void SendUtm() {
    /* Data available */
    flags_ = 0;
    if (tx_data_.unix_time_us) {
      time_ = tx_data_.unix_time_us.value();
      flags_ |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
    } else {
      time_ = 0;
    }
    if (tx_data_.uas_id) {
      uas_id_ = tx_data_.uas_id.value();
      flags_ |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
    } else {
      for (std::size_t i = 0; i < uas_id_.size(); i++) {
        uas_id_[i] = 0;
      }
    }
    if ((tx_data_.lat_rad) && (tx_data_.lon_rad) && (tx_data_.horz_acc_m)) {
      lat_ = static_cast<int32_t>(rad2deg(tx_data_.lat_rad.value()) * 1e7);
      lon_ = static_cast<int32_t>(rad2deg(tx_data_.lon_rad.value()) * 1e7);
      h_acc_ = static_cast<uint16_t>(tx_data_.horz_acc_m.value() * 1000.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
    } else {
      lat_ = 0;
      lon_ = 0;
      h_acc_ = 0;
    }
    if ((tx_data_.alt_wgs84_m) && (tx_data_.vert_acc_m)) {
      alt_ = static_cast<int32_t>(tx_data_.alt_wgs84_m.value() * 1000.0f);
      v_acc_ = static_cast<uint16_t>(tx_data_.vert_acc_m.value() * 1000.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;
    } else {
      alt_ = 0;
      v_acc_ = 0;
    }
    if (tx_data_.rel_alt_m) {
      relative_alt_ =
        static_cast<int32_t>(tx_data_.rel_alt_m.value() * 1000.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
    } else {
      relative_alt_ = 0;
    }
    if ((tx_data_.north_vel_mps) && (tx_data_.east_vel_mps)) {
      vx_ = static_cast<int16_t>(tx_data_.north_vel_mps.value() * 100.0f);
      vy_ = static_cast<int16_t>(tx_data_.east_vel_mps.value() * 100.0f);
      vel_acc_ = static_cast<int16_t>(tx_data_.vel_acc_mps.value() * 100.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
    } else {
      vx_ = 0;
      vy_ = 0;
    }
    if (tx_data_.down_vel_mps) {
      vz_ = static_cast<int16_t>(tx_data_.down_vel_mps.value() * 100.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
    } else {
      vz_ = 0;
    }
    if ((tx_data_.next_lat_rad) && (tx_data_.next_lon_rad) &&
        (tx_data_.next_alt_wgs84_m)) {
      next_lat_ = static_cast<int32_t>(rad2deg(tx_data_.next_lat_rad.value())
                                       * 1e7);
      next_lon_ = static_cast<int32_t>(rad2deg(tx_data_.next_lon_rad.value())
                                       * 1e7);
      next_alt_ = static_cast<int32_t>(tx_data_.next_alt_wgs84_m.value()
                                       * 1000.0f);
      flags_ |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
    } else {
      next_lat_ = 0;
      next_lon_ = 0;
      next_alt_ = 0;
    }
    update_rate_ = static_cast<uint16_t>(tx_data_.update_period_s * 100.0f);
    flight_state_ = static_cast<uint8_t>(tx_data_.flight_state);
    msg_len_ = mavlink_msg_utm_global_position_pack(sys_id_, comp_id_, &msg_,
                                                    time_, uas_id_.data(),
                                                    lat_, lon_, alt_,
                                                    relative_alt_,
                                                    vx_, vy_, vz_,
                                                    h_acc_, v_acc_, vel_acc_,
                                                    next_lat_, next_lon_,
                                                    next_alt_, update_rate_,
                                                    flight_state_, flags_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
    ResetTxData();
  }
  void ResetTxData() {
    tx_data_.alt_wgs84_m.reset();
    tx_data_.rel_alt_m.reset();
    tx_data_.next_alt_wgs84_m.reset();
    tx_data_.horz_acc_m.reset();
    tx_data_.vert_acc_m.reset();
    tx_data_.vel_acc_mps.reset();
    tx_data_.north_vel_mps.reset();
    tx_data_.east_vel_mps.reset();
    tx_data_.down_vel_mps.reset();
    tx_data_.unix_time_us.reset();
    tx_data_.lat_rad.reset();
    tx_data_.lon_rad.reset();
    tx_data_.next_lat_rad.reset();
    tx_data_.next_lon_rad.reset();
  }
  void ResetRxData() {
    for (std::size_t i = 0; i < rx_data_.size(); i++) {
      rx_data_[i].data_avail_flags = 0;
      rx_data_[i].flight_state = FLIGHT_STATE_UNKNOWN;
      rx_data_[i].update_period_s = 0;
      rx_data_[i].alt_wgs84_m.reset();
      rx_data_[i].rel_alt_m.reset();
      rx_data_[i].next_alt_wgs84_m.reset();
      rx_data_[i].horz_acc_m.reset();
      rx_data_[i].vert_acc_m.reset();
      rx_data_[i].vel_acc_mps.reset();
      rx_data_[i].north_vel_mps.reset();
      rx_data_[i].east_vel_mps.reset();
      rx_data_[i].down_vel_mps.reset();
      rx_data_[i].uas_id.reset();
      rx_data_[i].unix_time_us.reset();
      rx_data_[i].lat_rad.reset();
      rx_data_[i].lon_rad.reset();
      rx_data_[i].next_lat_rad.reset();
      rx_data_[i].next_lon_rad.reset();
    }
  }
  void UtmMsgHandler(const mavlink_utm_global_position_t &ref) {
    if (num_rx_ < N) {
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_TIME_VALID) {
        rx_data_[num_rx_].unix_time_us = ref.time;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE) {
        memcpy(uas_temp_id_.data(), ref.uas_id, UAS_ID_LEN);
        rx_data_[num_rx_].uas_id = uas_temp_id_;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE) {
        rx_data_[num_rx_].lat_rad = deg2rad(static_cast<double>(ref.lat) / 1e7);
        rx_data_[num_rx_].lon_rad = deg2rad(static_cast<double>(ref.lon) / 1e7);
        rx_data_[num_rx_].horz_acc_m = static_cast<float>(ref.h_acc) / 1000.0f;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE) {
        rx_data_[num_rx_].alt_wgs84_m = static_cast<float>(ref.alt) / 1000.0f;
        rx_data_[num_rx_].vert_acc_m = static_cast<float>(ref.v_acc) / 1000.0f;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE) {
        rx_data_[num_rx_].rel_alt_m = static_cast<float>(ref.relative_alt) /
                                      1000.0f;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE) {
        rx_data_[num_rx_].north_vel_mps = static_cast<float>(ref.vx) / 100.0f;
        rx_data_[num_rx_].east_vel_mps = static_cast<float>(ref.vy) / 100.0f;
        rx_data_[num_rx_].vel_acc_mps = static_cast<float>(ref.vel_acc) /
                                        100.0f;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE) {
        rx_data_[num_rx_].down_vel_mps = static_cast<float>(ref.vz) / 100.0f;
      }
      if (ref.flags & UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE) {
        rx_data_[num_rx_].next_lat_rad =
          deg2rad(static_cast<double>(ref.next_lat) / 1e7);
        rx_data_[num_rx_].next_lon_rad =
          deg2rad(static_cast<double>(ref.next_lon) / 1e7);
        rx_data_[num_rx_].next_alt_wgs84_m = static_cast<float>(ref.next_alt) /
                                             1000.0f;
      }
      rx_data_[num_rx_].update_period_s = static_cast<float>(ref.update_rate) /
                                          100.0f;
      rx_data_[num_rx_].flight_state =
        static_cast<FlightState>(ref.flight_state);
      rx_data_[num_rx_].data_avail_flags = ref.flags;
      num_rx_++;
    }
  }
};

}  // namespace bfs

#endif  // MAVLINK_SRC_UTM_H_ NOLINT
