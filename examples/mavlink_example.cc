/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/mavlink.h"

// mavlink_status_t rx_status, status;
// mavlink_message_t msg, rx_msg, r_msg;
// uint16_t len;
// uint8_t buf[MAVLINK_MAX_PACKET_LEN];

// int main() {
//   Serial.begin(115200);
//   while(!Serial) {}
//   Serial3.begin(57600);
//   elapsedMillis rx_time = 2000;
//   elapsedMillis hud_time = 2000;
//   while(1) {
//     if (rx_time > 1000) {
//       len = mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, 0, 0, MAV_STATE_STANDBY);
//       mavlink_msg_to_send_buffer(buf, &msg);
//       Serial3.write(buf, len);
//       rx_time = 0;
//     }
//     if (hud_time > 20) {
//       len = mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, millis(), 25 * 3.14 / 180, 10 * 3.14 / 180, 90 * 3.14 / 180 , 0, 0, 0);
//       len = mavlink_msg_to_send_buffer(buf, &msg);
//       Serial3.write(buf, len);
//       hud_time = 0;
//     }

//     if (Serial3.available()) {
//       if(mavlink_frame_char_buffer(&rx_msg, &rx_status, Serial3.read(), &r_msg, &status)) {
//         Serial.println(r_msg.msgid);
//       }      
//     }
//   }
// }

telemetry::MavLink mavlink(&Serial3, 1, telemetry::MavLink::VehicleType::FIXED_WING);

elapsedMillis att = 0;
elapsedMillis gnss = 0;

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  mavlink.Begin(57600);
  while(1) {
    mavlink.Update();
    if (att > 20) {
      mavlink.SendAttitude(millis(), 25 * 3.14/180, 10 * 3.14/180, 0, 0, 0, 0);
      mavlink.SendHud(millis(), 20, 20, 100, 0, 0, 0.5);
      att = 0;
    }
    if (gnss > 100) {
      mavlink.SendGnss(millis(), 5, 18, 35.679848 * 3.14/180, -105.962268 * 3.14/180, 2000, 2000, 12, 0, 0.01, 0.01, 0.01, 0.01);
      gnss = 0;
    }
  }
}
