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
* IN THE SOFTWARE.dd
*/

#include "mavlink/mavlink.h"

std::array<bfs::MissionItem, 10> mission;
std::array<bfs::MissionItem, 10> temp;

bfs::MavLink<5> mavlink(&Serial4, bfs::VehicleType::FIXED_WING, mission.data(), temp.data(), mission.size());

int main() {
  Serial.begin(115200);
  while (!Serial) {}
  mavlink.param_id(0, "CONTROL_00");
  mavlink.param_id(1, "MISSION_00");
  mavlink.param_id(2, "PARAM_00");
  mavlink.param_id(3, "FENCE_00");
  mavlink.param_id(4, "RALLY_00");
  mavlink.Begin(57600);
  while (1) {
    mavlink.Update();
    if (mavlink.waypoints_updated()) {
      Serial.println("WAYPOINTS UPDATED");
      Serial.println(mavlink.num_waypoints());
      Serial.println(mavlink.active_waypoint());
      for (int i = 0; i < mavlink.num_waypoints(); i++) {
        Serial.print(mission[i].autocontinue);
        Serial.print("\t");
        Serial.print(static_cast<int>(mission[i].cmd));
        Serial.print("\t");
        Serial.print(mission[i].frame);
        Serial.print("\t");
        Serial.print(mission[i].param1);
        Serial.print("\t");
        Serial.print(mission[i].param2);
        Serial.print("\t");
        Serial.print(mission[i].param3);
        Serial.print("\t");
        Serial.print(mission[i].param4);
        Serial.print("\t");
        Serial.print(mission[i].x);
        Serial.print("\t");
        Serial.print(mission[i].y);
        Serial.print("\t");
        Serial.print(mission[i].z);
        Serial.print("\n");
      }
    }
  }
}
