[![Pipeline](https://gitlab.com/bolderflight/software/mavlink/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/mavlink/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# mavlink
This library supports the [MAVLink](https://mavlink.io/) protocol, which is a common protocol for interfacing with ground control stations (i.e. [Mission Planner](https://ardupilot.org/planner/) and [QGroundControl](http://qgroundcontrol.com/)). This library has been specifically built for drone / UAS applications and tested against [QGroundControl](http://qgroundcontrol.com/).
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
MAVLink is a messaging protocol for communicating between drone components, located both on and off the vehicle. The protocol consists of many message and microservice definitions and [C headers](https://github.com/bolderflight/mavlink_c_library_v2) are available for packing, unpacking, sending, and parsing messages. This library implements each microservice as a class and then wraps all of them into a parent class for easier integration with vehicle software. The supported microservices are:
   * [Heartbeat](https://mavlink.io/en/services/heartbeat.html): establishes communication with other MAVLink components and relays information on vehicle type and status.
   * Utility: sends status texts to the ground station.
   * Telemetry: sends sensor and estimation data for display on a ground station or use in a MAVLink component. Can also receive SCALED_IMU, SCALED_PRESSURE, GPS_RAW_INT, ATTITUDE, VFR_HUD, LOCAL_POSITION_NED, and GLOBAL_POSITION_NED messages and output the received data.
   * [Parameter](https://mavlink.io/en/services/parameter.html): defines parameters whose values can be set by other MAVLink components, such as ground stations. Useful for defining on-board excitations, tunable control law gains, etc. Only floating point types are currently supported.
   * [Mission](https://mavlink.io/en/services/mission.html): enables Mission Items to be sent to the drone from a ground station. These can include flight plans, fences, and rally points.
   * [UTM](http://mavlink.io/en/services/traffic_management.html): implements the UAS Traffic Management (UTM) microservice, broadcasting the UTM data from the drone and receiving UTM data from other nearby drones.

The MAVLink protocol is massive and documentation is sometimes sparse with respect to *how* components should communicate with each other (the order of messages, what actions the drone or ground control station should take on receiving each message). Further, ground control stations often only use a subset of MAVLink. This library attempts to simplify MAVLink usage by providing an intuitive interface and abstracting away the microservice details.

This library has been tested extensively against [QGroundControl](http://qgroundcontrol.com/). If functionality appears to be missing or implemented incorrectly, please submit an issue detailing the observed behavior, the expected behavior, and the ground control station used.

# Installation
## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. In addition to this library, the [Bolder Flight Systems Units library](https://github.com/bolderflight/units) and the the [Bolder Flight Systems Navigation library](https://github.com/bolderflight/navigation) must be installed. The library is added as:

```C++
#include "mavlink.h"
```

An example is located in *examples/arduino/mavlink_example/mavlink_example.ino*. This library is tested with Teensy 3.x, 4.x, and LC devices and is expected to work with other Arduino ARM devices. It is **not** expected to work with AVR devices.

## CMake
CMake is used to build this library, which is exported as a library target called *mavlink*. The header is added as:

```C++
#include "mavlink.h"
```

The library can also be compiled stand-alone using the CMake idiom of creating a build directory and then, from within that directory, issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executable called *mavlink_example*. The example executable source files are located at *examples/cmake/mavlink_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *mavlink_example* target also has a *_hex* for creating the hex file and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*

# Methods

## Mission Item

**MissionItem** This struct defines a Mission Item, which can be a waypoint in a flight plan, a fence vertex, or a rally point. The struct is defined as:

| Name | Description |
| --- | --- |
| bool autocontinue | Whether to automatically continue to the next MissionItem |
| uint8_t frame | The [coordinate frame](https://mavlink.io/en/messages/common.html#MAV_FRAME) of the MissionItem |
| uint16_t cmd | The [command](https://mavlink.io/en/messages/common.html#mav_commands) associated with the MissionItem |
| float param1 | Command dependent parameter |
| float param2 | Command dependent parameter |
| float param3 | Command dependent parameter |
| float param4 | Command dependent parameter |
| int32_t x | Typically latitude represented as 1e7 degrees |
| int32_t y |  Typically longitude represented as 1e7 degrees  |
| float z | Typically altitude, but can be dependent on the command and frame |

## MavLink

**MavLink<std::size_t N, std::size_t M>** The MavLink constructor is templated with the number of parameters to support and the number of received UTM messages.

```C++
/* MavLink object support 5 parameters and up to 10 simultaneous UTM broadcasts received */
bfs::MavLink<5, 10> mavlink;
```

## Config

**inline void hardware_serial(HardwareSerial &ast;bus)** Sets the hardware serial bus to use.

```C++
mavlink.hardware_serial(&Serial4);
```

**inline void aircraft_type(const AircraftType type)** Sets the aircraft type, which the ground control station uses for configuring displays and options (i.e. how the vehicle takes off, the type of icon used to display the vehicle).

```C++
mavlink.aircraft_type(bfs::FIXED_WING);
```

Available aircraft types are:
| Enum               | Value (int8_t) | Description |
| ------------------ | ----- | -------------------  |
| FIXED_WING         | 0     | Fixed-wing aircraft  |
| HELICOPTER         | 1     | Helicopter           |
| MULTIROTOR         | 2     | Multirotor           |
| VTOL               | 3     | VTOL aircraft        |

**inline void sys_id(const uint8_t sys_id)** Used to set a non-default system id. By default the system id is set to 1. This could be used if other vehicles are connected to the ground control station to avoid conflicting system ids.

**inline void gnss_serial(HardwareSerial &ast;bus)** Sets the hardware serial bus connected to the GNSS receiver to provide RTCM corrections from a base station GNSS as transmitted via MAV Link from the ground control station software. It is assumed that the serial port is configured external to this class (i.e. serial port initialized and baudrate set correctly).

**inline void mission(MissionItem &ast; const mission, const std::size_t mission_size, MissionItem &ast; const temp)** Sets storage location for flight plans given a pointer for the data, the number of Mission Items that can be stored, and a pointer for temporary data. The temporary storage location is used while the ground station uploads flight plans, fences, and rally points. Once the upload is complete and verified, it's moved to the correct mission, fence, and rally point storage location. It's assumed that the temporary storage is at least as large as the largest storage location for flight plans, fences, and rally points. For example, if there is enough storage for 250 flight plan mission items, 100 fence points, and 5 rally points, the temporary storage would need at least enough capacity for 250 mission items.

**inline void fence(MissionItem &ast; const fence, const std::size_t fence_size)** Sets the storage location and size for fence points.

**inline void rally(MissionItem &ast; const rally, const std::size_t rally_size)** Sets the storage location and size for rally points.

```C++
std::array<bfs::MissionItem, 250> mission;
std::array<bfs::MissionItem, 100> fence;
std::array<bfs::MissionItem, 5> rally;
std::array<bfs::MissionItem, 250> temp;
mavlink.mission(mission.data(), mission.size(), temp.data());
mavlink.fence(fence.data(), fence.size());
mavlink.rally(rally.data(), rally.size());
```

## Setup / Update

**void Begin(uint32_t baud)** Initializes communication given a baudrate.

```C++
mavlink.Begin(57600);
```

**void Update()** Should be run often, sends, receives, and parses MAVLink messages.

```C++
while (1) {
  mavlink.Update();
}
```

## GCS Link

**bool gcs_link_established()** Returns true if communication has been established with a GCS.

**bool gcs_link_lost()** Returns true if communication has been established with a GCS and a heartbeat from the GCS has not been received in *gcs_timeout_ms* time.

**void gcs_lost_link_timeout_ms(const int32_t val)** Sets the GCS link lost timeout threshold, ms. The default is 5000 ms.

**int32_t gcs_lost_link_timeout_ms()** Returns the GCS link lost timeout threshold, ms.


## Heartbeat data

**inline constexpr AircraftType aircraft_type()** Returns the aircraft type.

**inline constexpr uint8_t sys_id()** Returns the system id.

**inline constexpr uint8_t comp_id()** Returns the component id.

**inline void throttle_enabled(const bool val)** Sets whether the motors are enabled.

**inline void aircraft_mode(const AircraftMode val)** Sets the aircraft mode. Available modes are:

| Enum       | Value (int8_t) | Description            |
| ---------- | ----- | ------------------------------- |
| MANUAL     | 0     | Manual flight mode              |
| STABALIZED | 1     | Stability augmented flight mode |
| ATTITUDE   | 2     | Attitude feedback flight mode   |
| AUTO       | 3     | Autonomous flight mode          | 
| TEST       | 4     | Test point                      |

**inline void aircraft_state(const AircraftState val)** Sets the aircraft state. Available states are:

| Enum      | Value (int8_t) | Description                         |
| --------- | ----- | -------------------------------------------- |
| INIT      | 0     | System is initializing                       |
| STANDBY   | 1     | Ready to fly, motors are not enabled         |
| ACTIVE    | 2     | Flying, motors are enabled                   |
| CAUTION   | 3     | Off-nominal condition, still able to operate |
| EMERGENCY | 4     | Emergency condition, not able to operate     |
| FTS       | 5     | Flight Termination System has been activated |

### Data Streams
Telemetry data is grouped into [Data Streams](https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM). For our library, the following stream and packet associations are defined:

| Stream | Telemetry Message(s) |
| --- | --- |
| Raw Sensors | [Scaled IMU](https://mavlink.io/en/messages/common.html#SCALED_IMU), [Scaled Pressure](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE), [GPS Raw](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) |
| Extended Status | [System Status](https://mavlink.io/en/messages/common.html#SYS_STATUS), [Battery Status](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) |
| RC Channels | [Servo Output Raw](https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW), [RC Channels](https://mavlink.io/en/messages/common.html#RC_CHANNELS) |
| Raw Controller | - |
| Position | [Local Position NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED), [Global Position](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT) |
| Extra 1 | [Attitude](https://mavlink.io/en/messages/common.html#ATTITUDE) |
| Extra 2 | [VFR HUD](https://mavlink.io/en/messages/common.html#VFR_HUD) |
| Extra 3 | [Wind Covariance](http://mavlink.io/en/messages/common.html#WIND_COV), [System Time](http://mavlink.io/en/messages/common.html#SYSTEM_TIME) |

Some ground stations, such as [Mission Planner](https://ardupilot.org/planner/) will request data stream rates from the autopilot, which this library will correctly handle. Other ground stations, such as [QGroundControl](http://qgroundcontrol.com/) do not request data stream rates and they must be set by the autopilot. The following methods enable getting and setting the data stream rates.

**inline void raw_sens_stream_period_ms(const int val)** Sets the raw sensors stream period, ms.

**inline int raw_sens_stream_period_ms()** Gets the raw sensors stream period, ms.

**inline void ext_status_stream_period_ms(const int val)** Sets the extended status stream period, ms.

**inline int ext_status_stream_period_ms()** Gets the extended status stream period, ms.

**inline void rc_chan_stream_period_ms(const int val)** Sets the RC channels stream period, ms.

**inline int rc_chan_stream_period_ms()** Gets the RC channels stream period, ms.

**inline void pos_stream_period_ms(const int val)** Sets the position stream period, ms.

**inline int pos_stream_period_ms()** Gets the position stream period, ms.

**inline void extra1_stream_period_ms(const int val)** Sets the extra 1 stream period, ms.

**inline int extra1_stream_period_ms()** Gets the extra 1 stream period, ms.

**inline void extra2_stream_period_ms(const int val)** Sets the extra 2 stream period, ms.

**inline int extra2_stream_period_ms()** Gets the extra 2 stream period, ms.

**inline void extra3_stream_period_ms(const int val)** Sets the extra 3 stream period, ms.

**inline int extra3_stream_period_ms()** Gets the extra 3 stream period, ms.

## Telemetry data

### System

**inline void sys_time_us(const uint64_t val)** Sets the system time (time since boot), us.

**inline void cpu_load(uint32_t frame_time_us, uint32_t frame_period_us)** Sets the CPU load given the frame time, us, and frame period, us.

#### Installed Sensors

**inline void gyro_installed(const bool val)** Sets whether a gyro is installed in the aircraft.

**inline void accel_installed(const bool val)** Sets whether an accelerometer is installed in the aircraft.

**inline void mag_installed(const bool val)** Sets whether a magnetometer is installed in the aircraft.

**inline void static_pres_installed(const bool val)** Sets whether a static pressure sensor is installed in the aircraft.

**inline void diff_pres_installed(const bool val)** Sets whether a differential pressure sensors is installed in the aircraft.

**inline void gnss_installed(const bool val)** Sets whether a GNSS receiver is installed in the aircraft.

**inline void inceptor_installed(const bool val)** Sets whether inceptors (i.e. pilot input) are installed in the aircraft.

#### Healthy Sensors

**inline void gyro_healthy(const bool val)** Sets whether the gyro is healthy.

**inline void accel_healthy(const bool val)** Sets whether the accelerometer is healthy.

**inline void mag_healthy(const bool val)** Sets whether the magnetometer is healthy.

**inline void static_pres_healthy(const bool val)** Sets whether the static pressure sensor is healthy.

**inline void diff_pres_healthy(const bool val)** Sets whether the differential pressure sensor is healthy.

**inline void gnss_healthy(const bool val)** Sets whether the GNSS receiver is healthy.

**inline void inceptor_healthy(const bool val)** Sets whether the inceptors (i.e. pilot inputs) are healthy.

#### Battery Data

**inline void battery_volt(const float val)** Sets the battery voltage.

**inline void battery_current_ma(const float val)** Sets the measured battery current, mA.

**inline void battery_consumed_mah(const float val)** Sets the battery current consumed, mAh.

**inline void battery_remaining_prcnt(const float val)** Sets the battery capacity remaining, % (i.e. 75 for 75%).

**inline void battery_remaining_time_s(const float val)** Sets the estimated battery time remaining, s.

### IMU Data

**inline void imu_accel_x_mps2(const float val)** Sets the IMU x acceleration, m/s/s.

**inline void imu_accel_y_mps2(const float val)** Sets the IMU y acceleration, m/s/s.

**inline void imu_accel_z_mps2(const float val)** Sets the IMU z acceleration, m/s/s.

**inline void imu_gyro_x_radps(const float val)** Sets the IMU x gyro, rad/s.

**inline void imu_gyro_y_radps(const float val)** Sets the IMU y gyro, rad/s.

**inline void imu_gyro_z_radps(const float val)** Sets the IMU z gyro, rad/s.

**inline void imu_mag_x_ut(const float val)** Sets the IMU x magnetometer, uT.

**inline void imu_mag_y_ut(const float val)** Sets the IMU y magnetometer, uT.

**inline void imu_mag_z_ut(const float val)** Sets the IMU z magnetometer, uT.

**inline void imu_die_temp_c(const float val)** Sets the IMU die temperature, C.

### Airdata

**inline void static_pres_pa(const float val)** Sets the static pressure data, Pa.

**inline void diff_pres_pa(const float val)** Sets the differential pressure data, Pa.

**inline void static_pres_die_temp_c(const float val)** Sets the static pressure die temperature, C.

**inline void diff_pres_die_temp_c(const float val)** Sets the differential pressure die temperature, C.

### GNSS Data

**inline void gnss_fix(const GnssFix val)** Sets the GNSS Fix. Available fix types are:

| Description | Enum |
| --- | --- |
| No fix | GNSS_FIX_NONE |
| 2D fix | GNSS_FIX_2D |
| 3D fix | GNSS_FIX_3D |
| 3D fix with differential GNSS corrections applied | GNSS_FIX_DGNSS |
| 3D fix with RTK float integer ambiguity | GNSS_FIX_RTK_FLOAT |
| 3D fix with RTK fixed integer ambiguity | GNSS_FIX_RTK_FIXED |

**inline void gnss_num_sats(const uint8_t val)** Sets the number of satellites used in the GNSS solution.

**inline void gnss_lat_rad(const double val)** Sets the GNSS latitude, rad.

**inline void gnss_lon_rad(const double val)** Sets the GNSS longitude, rad.

**inline void gnss_alt_msl_m(const float val)** Sets the GNSS altitude above Mean Sea Level, m.

**inline void gnss_alt_wgs84_m(const float val)** Sets the GNSS altitude above the WGS84 ellipsoid, m.

**inline void gnss_hdop(const float val)** Sets the horizontal dilution of precision.

**inline void gnss_vdop(const float val)** Sets the vertical dilution of precision.

**inline void gnss_track_rad(const float val)** Sets the GNSS estimated ground track, rad.

**inline void gnss_spd_mps(const float val)** Sets the GNSS estimated ground speed, m/s.

**inline void gnss_horz_acc_m(const float val)** Sets the estimated horizontal position accuracy, m.

**inline void gnss_vert_acc_m(const float val)** Sets the estimated vertical position accuracy, m.

**inline void gnss_vel_acc_mps(const float val)** Sets the estimated velocity accuracy, m/s.

**inline void gnss_track_acc_rad(const float val)** Sets the estimated ground track accuracy, rad.

### Navigation Filter Data

**inline void nav_lat_rad(const double val)** Latitude, rad, from the navigation filter.

**inline void nav_lon_rad(const double val)** Longitude, rad, from the navigation filter.

**inline void nav_alt_msl_m(const float val)** Altitude above Mean Sea Level, m, from the navigation filter.

**inline void nav_alt_agl_m(const float val)** Altitude relative to the home position (typically the location where the GNSS first achieves lock), m.

**inline void nav_north_pos_m(const float val)** Position North of the home position, m.

**inline void nav_east_pos_m(const float val)** Position East of the home position, m.

**inline void nav_down_pos_m(const float val)** Position Down of the home position, m.

**inline void nav_north_vel_mps(const float val)** Ground velocity in the north direction, m/s.

**inline void nav_east_vel_mps(const float val)** Ground velocity in the east direction, m/s.

**inline void nav_down_vel_mps(const float val)** Ground velocity in the down direction, m/s.

**inline void nav_gnd_spd_mps(const float val)** Ground speed, m/s.

**inline void nav_ias_mps(const float val)** Indicated airspeed, m/s.

**inline void nav_pitch_rad(const float val)** Pitch, rad.

**inline void nav_roll_rad(const float val)** Roll, rad.

**inline void nav_hdg_rad(const float val)** Heading, rad.

**inline void nav_gyro_x_radps(const float val)** Corrected gyro x axis measurement, rad/s.

**inline void nav_gyro_y_radps(const float val)** Corrected gyro y axis measurement, rad/s.

**inline void nav_gyro_z_radps(const float val)** Corrected gyro z axis measurement, rad/s.

### Effector

**inline void effector(const std::array<float, 16> &ref)** Array of effector commands, normalized 0 - 1.

**inline void effector(const std::array<int16_t, 16> &ref)** Array of effector commands, raw values.

### Inceptor

**inline void inceptor(const std::array<float, 16> &ref)** Array of inceptor commands, normalized 0 - 1.

**inline void inceptor(const std::array<int16_t, 16> &ref)** Array of inceptor commands, raw values.

**inline void throttle_ch(const uint8_t val)** The inceptor channel number for the throttle.

**inline void throttle_prcnt(const float val)** Throttle percent.

### Wind

**inline void wind_north_vel_mps(const float val)** Wind in X (NED) direction, m/s

**inline void wind_east_vel_mps(const float val)** Wind in Y (NED) direction, m/s

**inline void wind_down_vel_mps(const float val)** Wind in Z (NED) direction, m/s

**inline void wind_var_horz_mps(const float val)** Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate, m/s.

**inline void wind_var_vert_mps(const float val)** Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate, m/s.

**inline void wind_meas_alt_m(const float val)** Altitude (MSL) that this measurement was taken at, m.

**inline void wind_horz_acc_mps(const float val)** Horizontal speed 1-STD accuracy, m/s.

**inline void wind_vert_acc_mps(const float val)** Vertical speed 1-STD accuracy, m/s.

### Unix Time

**inline void unix_time_us(const uint64_t val)** Timestamp (UNIX epoch time), us.

### Home Position
Sends the home position latitude (rad), longitude (rad), and altitude (m).

**inline void home_lat_rad(const double val)** Sets the home position latitude, rad.

**inline void home_lon_rad(const double val)** Sets the home position longitude, rad.

**inline void home_alt_m(const float val)** Sets the home position altitude, m.

**inline void SendHomePos()** Sends the home position data.

## Receiving Telemetry Data

### IMU

**optional<float> imu_accel_x_mps2()** Returns the IMU x accelerometer data, m/s/s, if it has been updated since the last time this method was called.

**optional<float> imu_accel_y_mps2()** Returns the IMU y accelerometer data, m/s/s, if it has been updated since the last time this method was called.

**optional<float> imu_accel_z_mps2()** Returns the IMU z accelerometer data, m/s/s, if it has been updated since the last time this method was called.

**optional<float> imu_gyro_x_radps()** Returns the IMU x gyro data, rad/s, if it has been updated since the last time this method was called.

**optional<float> imu_gyro_y_radps()** Returns the IMU y gyro data, rad/s, if it has been updated since the last time this method was called.

**optional<float> imu_gyro_z_radps()** Returns the IMU z gyro data, rad/s, if it has been updated since the last time this method was called.

**optional<float> imu_mag_x_ut()** Returns the IMU x magnetometer data, uT, if it has been updated since the last time this method was called.

**optional<float> imu_mag_y_ut()** Returns the IMU y magnetometer data, uT, if it has been updated since the last time this method was called.

**optional<float> imu_mag_z_ut()** Returns the IMU z magnetometer data, uT, if it has been updated since the last time this method was called.

**optional<float> imu_die_temp_c()** Returns the IMU die temperature, C, if it has been updated since the last time this method was called.

### Air Data

**optional<float> static_pres_pa()** Returns the static pressure, Pa, if it has been updated since the last time this method was called.

**optional<float> diff_pres_pa()** Returns the differential pressure, Pa, if it has been updated since the last time this method was called.

**optional<float> static_pres_die_temp_c()** Returns the static pressure die temperature, C, if it has been updated since the last time this method was called.

**optional<float> diff_pres_die_temp_c()** Returns the differential pressure die temperature, C, if it has been updated since the last time this method was called.

### GNSS

**optional<int8_t> gnss_fix()** Returns the GNSS fix, if it has been updated since the last time this method was called.

**optional<int8_t> gnss_num_sats()** Returns the number of satellites used in the GNSS solution, if it has been updated since the last time this method was called.

**optional<double> gnss_lat_rad()** Returns the latitude, rad, if it has been updated since the last time this method was called.

**optional<double> gnss_lon_rad()** Returns the longitude, rad, if it has been updated since the last time this method was called.

**optional<float> gnss_alt_msl_m()** Returns the altitude above mean sea level, m, if it has been updated since the last time this method was called.

**optional<float> gnss_alt_wgs84_m()** Returns the altitude above the WGS84 ellipsoid, m, if it has been updated since the last time this method was called.

**optional<float> gnss_hdop()** Returns the horizontal dilution of precision if it has been updated since the last time this method was called.

**optional<float> gnss_vdop()** Returns the vertical dilution of precision if it has been updated since the last time this method was called.

**optional<float> gnss_track_rad()** Returns the GNSS ground track, rad, if it has been updated since the last time this method was called.

**optional<float> gnss_spd_mps()** Returns the GNSS estimated ground speed, m/s, if it has been updated since the last time this method was called.

**optional<float> gnss_horz_acc_m()** Returns the GNSS estimated horizontal position accuracy, m, if it has been updated since the last time this method was called.

**optional<float> gnss_vert_acc_m()** Returns the GNSS estimated vertical position accuracy, m, if it has been updated since the last time this method was called.

**optional<float> gnss_vel_acc_mps()** Returns the GNSS estimated velocity accuracy, m/s, if it has been updated since the last time this method was called.

**optional<float> gnss_track_acc_rad()** Returns the GNSS estimated track accuracy, rad, if it has been updated since the last time this method was called.

**optional<float> gnss_yaw_rad()** Returns the GNSS yaw, rad, if it has been updated since the last time this method was called.

### Navigation Filter

**optional<double> nav_lat_rad()** Returns the latitude, rad, from the navigation filter, if it has been updated since the last time this method was called.

**optional<double> nav_lon_rad()** Returns the longitude, rad, from the navigation filter, if it has been updated since the last time this method was called.

**optional<float> nav_alt_msl_m()** Returns the altitude above mean sea level, m, if it has been updated since the last time this method was called.

**optional<float> nav_alt_agl_m()** Returns the altitude above ground level, m, if it has been updated since the last time this method was called.

**optional<float> nav_north_pos_m()** Returns the position north of the home position, m, if it has been updated since the last time this method was called.

**optional<float> nav_east_pos_m()** Returns the position east of the home position, m, if it has been updated since the last time this method was called.

**optional<float> nav_down_pos_m()** Returns the position down from the home position, m, if it has been updated since the last time this method was called.

**optional<float> nav_north_vel_mps()** Returns the vehicle velocity in the north direction, m/s, if it has been updated since the last time this method was called.

**optional<float> nav_east_vel_mps()** Returns the vehicle velocity in the east direction, m/s, if it has been updated since the last time this method was called.

**optional<float> nav_down_vel_mps()** Returns the vehicle velocity in the down direction, m/s, if it has been updated since the last time this method was called.

**optional<float> nav_gnd_spd_mps()** Returns the vehicle ground speed, m/s, if it has been updated since the last time this method was called.

**optional<float> nav_ias_mps()** Returns the indicated airspeed, m/s, if it has been updated since the last time this method was called.

**optional<float> nav_pitch_rad()** Returns the pitch angle, rad, if it has been updated since the last time this method was called.

**optional<float> nav_roll_rad()** Returns the roll angle, rad, if it has been updated since the last time this method was called.

**optional<float> nav_hdg_rad()** Returns the vehicle heading, rad, if it has been updated since the last time this method was called.

**optional<float> nav_gyro_x_radps()** Returns the corrected gyro x measurement, rad/s, if it has been updated since the last time this method was called.

**optional<float> nav_gyro_y_radps()** Returns the corrected gyro y measurement, rad/s, if it has been updated since the last time this method was called.

**optional<float> nav_gyro_z_radps()** Returns the corrected gyro z measurement, rad/s, if it has been updated since the last time this method was called.


### Home Position

**optional<double> home_lat_rad()** Returns the home position latitude, rad, if it has been updated since the last time this method was called.

**optional<double> home_lon_rad()** Returns the home position longitude, rad, if it has been updated since the last time this method was called.

**optional<float> home_alt_m()** Returns the home position altitude, m, if it has been updated since the last time this method was called.

## Parameters

**static constexpr std::size_t params_size()** The number of parameters supported.

**inline void params(const std::array<float, N> &val)** Sets parameter values given an array. Should be called before the *Begin* method so parameter values are correctly sent to the ground station.

**inline std::array<float, NPARAM> params()** An array of parameter values.

**inline void param(const int32_t idx, const float val)** Sets a single parameter value, given the index and the value. Should be called before the *Begin* method so parameter values are correctly sent to the ground station.

**inline float param(const int32_t idx)** A single parameter value given the index.

**inline int32_t updated_param()** Returns the parameter number that was last updated, otherwise returns -1 if none updated.

**inline void param_id(const int32_t idx, char const (&name)[NCHAR])** Sets the parameter name given the index. By default, parameters are named "PARAM". Should be called before the *Begin* method, since parameter names are not intended to change after they are loaded.

**inline std::string param_id(const int32_t idx)** Returns the parameter name given the index.

## Mission / Flight Plan

**inline bool mission_updated()** Returns whether the flight plan has been updated.

**inline int32_t active_mission_item()** Returns the index number of the active mission item.

**inline std::size_t num_mission_items()** Returns the number of mission items in the flight plan.

**void AdvanceMissionItem()** Advances the flight plan to the next mission item. This should be called, for instance, when the aircraft reaches a waypoint.

## Fence

**inline bool fence_updated()** Returns whether the fence data has been updated.

**inline std::size_t num_fence_items()** Returns the number of fence items.

## Rally Points

**inline bool rally_points_updated()** Returns whether the rally points have been updated.

**inline std::size_t num_rally_points()** Returns the number of rally points.

## Utility

**void SendStatusText(Severity severity, char const &ast;msg)** Sends a status text given the severity and a message.

## UAS Traffic Management (UTM)

### Config

**void utm_update_period_s(const float val)** Sets the update rate for broadcasting UTM messages.

**constexpr std::size_t UTM_UAS_ID_LEN** The length of the UAS ID.

**void utm_uas_id(const std::array<uint8_t, UTM_UAS_ID_LEN> &id)** Sets the UTM UAS ID.

### Broadcast Data

**void utm_unix_time_us(const uint64_t val)** Time since the unix epoch, us.

**void utm_lat_rad(const double val)** Latitude, rad.

**void utm_lon_rad(const double val)** Longitude, rad.

**void utm_alt_wgs84_m(const float val)** Altitude above the WGS84 ellipsoid, m.

**void utm_rel_alt_m(const float val)** Altitude above ground, m.

**void utm_north_vel_mps(const float val)** North velocity, m/s.

**void utm_east_vel_mps(const float val)** East velocity, m/s.

**void utm_down_vel_mps(const float val)** Down velocity, m/s.

**void utm_horz_acc_m(const float val)** Horizontal position accuracy, m.

**void utm_vert_acc_m(const float val)** Vertical position accuracy, m.

**void utm_vel_acc_mps(const float val)** Velocity accuracy, m/s.

**void utm_next_lat_rad(const double val)** Latitude of the next waypoint, rad.

**void utm_next_lon_rad(const double val)** Longitude of the next waypoint, rad.

**void utm_next_alt_wgs84_m(const float val)** Altitude above the WGS84 ellipsoid of the next waypoint, m.

**void utm_flight_state(const FlightState val)** UTM flight state, which is an enum:

| Enum | Description |
| --- | --- |
| FLIGHT_STATE_UNKNOWN | The flight state can't be determined |
| FLIGHT_STATE_GROUND | UAS on ground |
| FLIGHT_STATE_AIRBORNE | UAS airborne |
| FLIGHT_STATE_EMERGENCY | UAS is in an emergency flight state |
| FLIGHT_STATE_NOCTRL | UAS has no active controls |

### Receive Data

**std::size_t utm_num_rx()** Number of UTM broadcasts received.

**void UtmClearRx()** Clear the received data.

**optional<uint64_t> get_utm_unix_time_us(const std::size_t idx)** Returns the received unix time (if available) from the given UTM broadcast index.

**optional<std::array<uint8_t, UTM_UAS_ID_LEN>> get_utm_uas_id(const std::size_t idx)** Returns the received UAS ID (if available) from the given UTM broadcast index.

**optional<double> get_utm_lat_rad(const std::size_t idx)** Returns the received latitude, rad (if available) from the given UTM broadcast index.

**optional<double> get_utm_lon_rad(const std::size_t idx)** Returns the received longitude, rad (if available) from the given UTM broadcast index.

**optional<float> get_utm_alt_wgs84_m(const std::size_t idx)** Returns the received altitude above the WGS84 ellipsoid, m (if available) from the given UTM broadcast index.

**optional<float> get_utm_rel_alt_m(const std::size_t idx)** Returns the received altitude above ground, m (if available) from the given UTM broadcast index.

**optional<float> get_utm_north_vel_mps(const std::size_t idx)** Returns the received north velocity, m/s (if available) from the given UTM broadcast index.

**optional<float> get_utm_east_vel_mps(const std::size_t idx)** Returns the received east velocity, m/s (if available) from the given UTM broadcast index.

**optional<float> get_utm_down_vel_mps(const std::size_t idx)** Returns the received down velocity, m/s (if available) from the given UTM broadcast index.

**optional<float> get_utm_horz_acc_m(const std::size_t idx)** Returns the received horizontal position accuracy, m (if available) from the given UTM broadcast index.

**optional<float> get_utm_vert_acc_m(const std::size_t idx)** Returns the received vertical position accuracy, m (if available) from the given UTM broadcast index.

**optional<float> get_utm_vel_acc_mps(const std::size_t idx)** Returns the received velocity accuracy, m/s (if available) from the given UTM broadcast index.

**optional<double> get_utm_next_lat_rad(const std::size_t idx)** Returns the received latitude of the next waypoint, rad (if available) from the given UTM broadcast index.

**optional<double> get_utm_next_lon_rad(const std::size_t idx)** Returns the received longitude of the next waypoint, rad (if available) from the given UTM broadcast index.

**optional<float> get_utm_next_alt_wgs84_m(const std::size_t idx)** Returns the received altitude above the WGS84 ellipsoid of the next waypoint, m (if available) from the given UTM broadcast index.

**float get_utm_update_period_s(const std::size_t idx)** Returns the received update period, s from the given UTM broadcast index.

**FlightState get_utm_flight_state(const std::size_t idx)** Returns the received UTM flight state from the given UTM broadcast index.


