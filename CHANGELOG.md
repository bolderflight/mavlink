# Changelog

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
