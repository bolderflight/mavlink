cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(Mavlink
    VERSION 4.4.0
    DESCRIPTION "Mavlink communication protocol interface"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Fetch units
  FetchContent_Declare(
    units
    GIT_REPOSITORY https://github.com/bolderflight/units.git
    GIT_TAG v4.1.0
  )
  FetchContent_MakeAvailable(units)
  # Fetch navigation
  FetchContent_Declare(
    navigation
    GIT_REPOSITORY https://github.com/bolderflight/navigation.git
    GIT_TAG v4.0.1
  )
  FetchContent_MakeAvailable(navigation)
  # Add the library target
  add_library(mavlink
    src/heartbeat.cpp
    src/telemetry.cpp
    src/mission.cpp
    src/rtcm.cpp
    src/mavlink.h
    src/heartbeat.h
    src/mavlink.h
    src/telemetry.h
    src/parameter.h
    src/mission.h
    src/rtcm.h
    src/utm.h
  )
  # Link libraries
  target_link_libraries(mavlink
    PUBLIC
      core
      units
      navigation
  )
  # Setup include directories 
  target_include_directories(mavlink PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )

  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the example target
    add_executable(mavlink_example examples/cmake/mavlink_example.cc)
    # Add the includes
    target_include_directories(mavlink_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mavlink_example
      PRIVATE 
        mavlink
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mavlink_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
