# Changelog

# v4.7.6
- Having issues with the newest navigation library, rolling back until those are resolved.

# v4.7.5
- Added methods to set the number of mission, fence, and rally point items. This allows updating the mission on the process and sending the updated mission to the GCS.
- Added MISSION_CHANGED message to notify ground station that the mission has changed.

# v4.7.4
- Added *get_* to the UTM getters to fix an issue where it isn't clear whether the setter or getter is being used. This is a temporary fix until v5 of the library.

# v4.7.3
- Added MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST handler to send capabilities to Mission Planner
- Added a flag to identify whether we are communicating with Mission Planner to send an Ack after receiving each mission item, which Mission Planner looks for to send the next item, but Q Ground Control sees as an error

# v4.7.2
- Fixed bug in sending home position data

# v4.7.1
- Fixed bug in setting home position altitude

# v4.7.0
- Added home position reception

# v4.6.0
- Added home position transmit message

# v4.5.0
- Added system time transmit message

# v4.4.0
- Added wind covariance transmit message

## v4.3.0
- Added capability to receive SCALED_IMU, SCALED_PRESSURE, GPS_RAW_INT, ATTITUDE, VFR_HUD, LOCAL_POSITION_NED, and GLOBAL_POSITION_NED messages and output the data

## v4.2.0
- Added UTM microservice

## v4.1.0
- Added a method to indicate whether communication has been established with a GCS
- Added a method to indicate whether the link has been lost with a GCS
- Added methods to set and get the link lost timeout

## v4.0.2
- Updated to pull in new MAV Link messages

## v4.0.1
- Updated to pull in Navigation v4.0.1, which fixed some inline-ness issues

## v4.0.0
- Updating to support both Arduino and CMake build systems

## v3.5.1
- Pulling raw PWM and SBUS to mavlink object.

## v3.5.0
- Added ability to use raw PWM and SBUS values for inceptor and effector telemetry.

## v3.4.0
- Added RTCM corrections

## v3.3.0
- Added ability to set the parameter value from code; this is useful if parameters are stored in EEPROM or SD for the microcontroller to read them and set to the stored values.

## v3.2.1
- Fix to add the methods from v3.2.0 to the MavLink object.

## v3.2.0
- Added battery current, battery consumed, battery remaining (percent), and battery remaining (time, s) telemetry messages

## v3.1.2
- Fixed indices used to re-request items in v3.1.1

## v3.1.1
- If a mission item is received out of order, re-request the item in the correct order instead of just sending an error.

## v3.1.0
- Pulling mission item from global defs

## v3.0.4
- Had to update GnssFix in the mavlink class too.

## v3.0.3
- Updated GnssFix to an int8_t

## v3.0.2
- Updated for gnss v2.4.0

## v3.0.1
- Constness bug with updated_param

## v3.0.0
- Moved config items out of constructor.

## v2.0.2
- Moved *int* to specific size for better support across different processor architectures

## v2.0.1
- Fixed a typo in the README.

## v2.0.0
- Rewritten and updated to improve functionality, should be considered a new baseline
- Supports the heartbeat, parameter, and mission microservices along with telemetry

## v1.0.3
- Updated for core v1.0.3 and global_defs v1.1.2

## v1.0.2
- Updated CONTRIBUTING.md
- Updated *fetch_content* links to use https instead of ssh to facilitate public access
- Updated *flash_mcu.cmake* to use local loader for Linux

## v1.0.1
- Updated license to MIT
- Specified versions for dependencies

## v1.0.0
- Initial baseline
