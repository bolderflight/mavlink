/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "mavlink/mavlink.h"
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "global_defs/global_defs.h"

namespace telemetry {

void MavLink::Begin(uint32_t baud) {
  bus_->begin(baud);
}

bool MavLink::Update() {
  /* Send heartbeat at 1 Hz */
  if (heartbeat_timer_ > HEARTBEAT_PERIOD_MS_) {
    msg_len_ = mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg_, veh_type_, dev_type_, 0, 0, status_);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
    heartbeat_timer_ = 0;
  }
  /* Setup to check for packets received in the future */
  return false;  
}

void MavLink::SendAttitude(uint32_t time_ms, float roll_rad, float pitch_rad, float yaw_rad, float gx_radps, float gy_radps, float gz_radps) {
  msg_len_ = mavlink_msg_attitude_pack(sys_id_, comp_id_, &msg_, time_ms, roll_rad, pitch_rad, yaw_rad, gx_radps, gy_radps, gz_radps);
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

void MavLink::SendGnss(uint32_t time_ms, uint8_t fix, uint8_t num_sv, double lat_rad, double lon_rad, float alt_msl_m, float alt_wgs84_m, float ground_speed_mps, float track_rad, float hacc, float vacc, float sacc, float tacc) {
  msg_len_ = mavlink_msg_gps_raw_int_pack(sys_id_, comp_id_, &msg_, 
    static_cast<uint64_t>(time_ms * 1000),
    fix,
    static_cast<int32_t>(global::conversions::Rad_to_Deg(lat_rad) * 1e7),
    static_cast<int32_t>(global::conversions::Rad_to_Deg(lon_rad) * 1e7),
    static_cast<int32_t>(alt_msl_m * 1000.0f),
    UINT16_MAX,
    UINT16_MAX,
    static_cast<uint16_t>(ground_speed_mps * 100.0f),
    static_cast<uint16_t>(global::conversions::Rad_to_Deg(track_rad) * 100.0f),
    num_sv,
    static_cast<int32_t>(alt_wgs84_m * 1000.0f),
    static_cast<uint32_t>(hacc * 1000.0f),
    static_cast<uint32_t>(vacc * 1000.0f),
    static_cast<uint32_t>(sacc * 1000.0f),
    static_cast<uint32_t>(global::conversions::Rad_to_Deg(tacc) * 100000.0f),
    0
  );
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

void MavLink::SendHud(uint32_t time_ms, float airspeed_mps, float ground_speed_mps, float alt_msl_m, float climb_mps, float heading_rad, float throttle_nd) {
  msg_len_ = mavlink_msg_vfr_hud_pack(sys_id_, comp_id_, &msg_,
    airspeed_mps,
    ground_speed_mps,
    static_cast<int16_t>(global::conversions::Rad_to_Deg(heading_rad)),
    static_cast<uint16_t>(throttle_nd * 100.0f),
    alt_msl_m,
    climb_mps
  );
  mavlink_msg_to_send_buffer(msg_buf_, &msg_);
  bus_->write(msg_buf_, msg_len_);
}

}  // telemetry
