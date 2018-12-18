cmake_minimum_required(VERSION 2.6)

project(RealsenseCaptureVerification)

if(MSVC)
	add_definitions(-D_SCL_SECURE_NO_WARNINGS )
	add_definitions(-D_CRT_SECURE_NO_WARNINGS )
	add_definitions(-D_USE_MATH_DEFINES)
	add_definitions(-DWIN32_LEAN_AND_MEAN)
	add_definitions(-D_USE_MATH_DEFINES)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP ") # add multi processing compilation to the msvc to compile faster
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /MT" )
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  /MTd" )
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
else(MSVC)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3" )
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-unused-result -Wreturn-type")
endif(MSVC)

Set(Optional_LIBS)

#Find LibRealSense------------------------------------------------------------
find_package(realsense2 REQUIRED)

set(LibRealSense_THIRDPARTY_DIR "${LibRealSense_INCLUDE_DIR}/../third-party/")

include_directories(
	${LibRealSense_INCLUDE_DIR}
	${LibRealSense_THIRDPARTY_DIR}
	)

SET(SRC src/RealsenseCaptureVerification.cpp)
SET(HEADERS src/device_container.hpp)
	
add_executable(realsense_capture_verification ${SRC} ${HEADERS})
set_target_properties(realsense_capture_verification PROPERTIES CXX_STANDARD 14)

if (MSVC)
	target_link_libraries(
			realsense_capture_verification
			realsense2::realsense2
			${Optional_LIBS})
else (MSVC)
	target_link_libraries(
			realsense_capture_verification
			realsense2::realsense2
			${Optional_LIBS}
			-lpthread -lm)
endif (MSVC)
