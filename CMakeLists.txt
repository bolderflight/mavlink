cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(Mavlink
	VERSION 3.5.1
	DESCRIPTION "Mavlink communication protocol interface"
	LANGUAGES CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	include(FetchContent)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY https://github.com/bolderflight/core.git
		GIT_TAG v2.0.5
	)
	FetchContent_MakeAvailable(core)
	# Fetch global defs
	FetchContent_Declare(
		global_defs
		GIT_REPOSITORY https://github.com/bolderflight/global_defs.git
		GIT_TAG v2.1.0
	)
	FetchContent_MakeAvailable(global_defs)
	# Fetch mavlink-v2 headers
	FetchContent_Declare(
		mavlink_headers_v2
		GIT_REPOSITORY https://github.com/bolderflight/mavlink_c_library_v2.git
		GIT_TAG v1.1.0
	)
	FetchContent_MakeAvailable(mavlink_headers_v2)
	# Fetch units
	FetchContent_Declare(
		units
		GIT_REPOSITORY https://github.com/bolderflight/units.git
		GIT_TAG v3.2.0
	)
	FetchContent_MakeAvailable(units)
	# Fetch navigation
	FetchContent_Declare(
		navigation
		GIT_REPOSITORY https://github.com/bolderflight/navigation.git
		GIT_TAG v2.0.1
	)
	FetchContent_MakeAvailable(navigation)
	# Fetch gnss
	FetchContent_Declare(
		gnss
		GIT_REPOSITORY https://github.com/bolderflight/gnss.git
		GIT_TAG v2.4.0
	)
	FetchContent_MakeAvailable(gnss)
	# Add the library target
	add_library(mavlink
		src/mavlink/heartbeat.cc
		src/mavlink/telemetry.cc
		src/mavlink/mission.cc
		src/mavlink/rtcm.cc
		include/mavlink/mavlink.h
		include/mavlink/heartbeat.h
		include/mavlink/mavlink.h
		include/mavlink/telemetry.h
		include/mavlink/parameter.h
		include/mavlink/mission.h
		include/mavlink/rtcm.h
	)
	# Link libraries
	target_link_libraries(mavlink
		PUBLIC
			core
			global_defs
			mavlink_headers_v2
			units
			navigation
			gnss
	)
	# Setup include directories 
	target_include_directories(mavlink PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
endif()

# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)		
		# Add the spi example target
		add_executable(mavlink_example examples/mavlink_example.cc)
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
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(mavlink_example ${MCU})
	endif()
endif()
