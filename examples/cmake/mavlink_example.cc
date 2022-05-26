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

#include "mavlink.h"

/*
* Storage for mission items (i.e. flight plans), fence vertices, rally points,
* and temporary storage, which is used to hold items during upload from the
* GCS and before the upload is verified good.
*/
std::array<bfs::MissionItem, 250> mission;
std::array<bfs::MissionItem, 250> fence;
std::array<bfs::MissionItem, 5> rally;
std::array<bfs::MissionItem, 250> temp;

/*
* A MavLink object with 5 parameters that can be tuned in real-time from the GCS
* and up to 10 simultaneous UTM messages received
*/
bfs::MavLink<5, 10> mavlink;

bool send_update = false;
elapsedMillis t = 0;

int main() {
  /* Starting serial to print results */
  Serial.begin(115200);
  while (!Serial) {}
  /* Configuring MavLink serial port and aircraft type */
  mavlink.hardware_serial(&Serial4);
  mavlink.aircraft_type(bfs::FIXED_WING);
  /* Passing info about where to store mission data */
  mavlink.mission(mission.data(), mission.size(), temp.data());
  mavlink.fence(fence.data(), fence.size());
  mavlink.rally(rally.data(), rally.size());
  /* Starting communication on the serial port */
  mavlink.Begin(57600);
  while (1) {
    /* Needed to send and receive MavLink data */
    mavlink.Update();
    /* Check to see if the mission has been updated and print mission items */
    if (mavlink.mission_updated()) {
      Serial.println("MISSION UPDATED");
      Serial.println(mavlink.num_mission_items());
      for (std::size_t i = 0; i < mavlink.num_mission_items(); i++) {
        Serial.print(mission[i].x);
        Serial.print("\t");
        Serial.print(mission[i].y);
        Serial.print("\t");
        Serial.print(mission[i].z);
        Serial.print("\t");
        mission[i].z += 100;
        Serial.print(mission[i].z);
        Serial.print("\n");
      }
      send_update = true;
      t = 0;
    }
    if ((send_update) && (t > 10000)) {
      mavlink.num_mission_items(mavlink.num_mission_items());
      send_update = false;
      Serial.println("UPDATE SENT");
    }
  }
}
