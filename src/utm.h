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
#else
#include "core/core.h"
#endif
#include <array>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "optional.hpp"

namespace bfs {

using nonstd::optional;

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
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* Send and receive UTM data */
  void Update() {
    
  }
  /* Prepare to send UTM data */
  static constexpr std::size_t UAS_ID_LEN = 18;
  inline void unix_time_us(const uint64_t val) {tx_data_.unix_time_us = val;}
  inline void uas_id(const std::array<uint8_t, UAS_ID_LEN> &id) {
    tx_data_.uas_id = id;
  }
  inline void lat_rad(const double val) {tx_data_.lat_rad = val;}
  inline void lon_rad(const double val) {tx_data_.lon_rad = val;}
  inline void alt_wgs84_m(const float val) {tx_data_.alt_m = val;}
  inline void rel_alt_m(const float val) {tx_data_.rel_alt_m = val;}
  inline void north_vel_mps(const float val) {tx_data_.ned_vel_mps[0] = val;}
  inline void east_vel_mps(const float val) {tx_data_.ned_vel_mps[1] = val;}
  inline void down_vel_mps(const float val) {tx_data_.ned_vel_mps[2] = val;}
  inline void horz_acc_m(const float val) {tx_data_.h_acc_m = val;}
  inline void vert_acc_m(const float val) {tx_data_.v_acc_m = val;}
  inline void vel_acc_mps(const float val) {tx_data_.vel_acc_mps = val;}
  inline void next_lat_rad(const double val) {tx_data_.next_lat_rad = val;}
  inline void next_lon_rad(const double val) {tx_data_.next_lon_rad = val;}
  inline void next_alt_wgs84_m(const float val) {tx_data_.next_alt_m = val;}
  inline void update_period_s(const float val) {
    tx_data_.update_period_s = val;
  }
  inline void flight_state(const FlightState val) {
    tx_data_.flight_state = val;
  }
  /* Receive UTM data */
  inline std::size_t num_rx() {return num_rx_;}
  inline optional<uint64_t> unix_time_us(const std::size_t idx) {
    return rx_data_[idx].unix_time_us;
  }
  inline optional<std::array<uint8_t, UAS_ID_LEN>> uas_id(
                                                        const std::size_t idx) {
    return rx_data_[idx].uas_id;
  }
  inline optional<double> lat_rad(const std::size_t idx) {
    return rx_data_[idx].lat_rad;
  }
  inline optional<double> lon_rad(const std::size_t idx) {
    return rx_data_[idx].lon_rad;
  }
  inline optional<float> alt_wgs84_m(const std::size_t idx) {
    return rx_data_[idx].alt_m;
  }
  inline optional<float> rel_alt_m(const std::size_t idx) {
    return rx_data_[idx].rel_alt_m;
  }
  inline optional<float> north_vel_mps(const std::size_t idx) {
    return rx_data_[idx].ned_vel_mps[0];
  }
  inline optional<float> east_vel_mps(const std::size_t idx) {
    return rx_data_[idx].ned_vel_mps[1];
  }
  inline optional<float> down_vel_mps(const std::size_t idx) {
    return rx_data_[idx].ned_vel_mps[2];
  }
  inline optional<float> horz_acc_m(const std::size_t idx) {
    return rx_data_[idx].h_acc_m;
  }
  inline optional<float> vert_acc_m(const std::size_t idx) {
    return rx_data_[idx].v_acc_m;
  }
  inline optional<float> vel_acc_mps(const std::size_t idx) {
    return rx_data_[idx].vel_acc_mps;
  }
  inline optional<double> next_lat_rad(const std::size_t idx) {
    return rx_data_[idx].next_lat_rad;
  }
  inline optional<double> next_lon_rad(const std::size_t idx) {
    return rx_data_[idx].next_lon_rad;
  }
  inline optional<float> next_alt_wgs84_m(const std::size_t idx) {
    return rx_data_[idx].next_alt_m;
  }
  inline optional<float> update_period_s(const std::size_t idx) {
    return rx_data_[idx].update_period_s;
  }
  inline FlightState flight_state(const std::size_t idx) {
    return rx_data_[idx].flight_state;
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  uint8_t sys_id_ = 1;
  static constexpr uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  /* UTM data */
  struct UtmData {
    uint8_t data_avail_flags;
    FlightState flight_state;
    optional<float> alt_m;
    optional<float> rel_alt_m;
    optional<float> next_alt_m;
    optional<float> update_period_s;
    optional<float> h_acc_m;
    optional<float> v_acc_m;
    optional<float> vel_acc_mps;
    optional<float> ned_vel_mps[3];
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
  std::size_t num_rx_;
  std::array<UtmData, N> rx_data_;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_UTM_H_ NOLINT
