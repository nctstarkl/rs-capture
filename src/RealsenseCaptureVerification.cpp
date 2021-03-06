// This code is a modified mash-up version of the Intel librealsense rs-save-to-disk and rs-multicam example projects.

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include "device_container.hpp" // Device container class from rs multicam example

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char * argv[]) try
{
	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;

	device_container connected_devices;

	rs2::context ctx;    // Create librealsense context for managing devices

	// Register callback for tracking which devices are currently connected
	ctx.set_devices_changed_callback([&](rs2::event_information& info)
	{
		connected_devices.remove_devices(info);
		for (auto&& dev : info.get_new_devices())
		{
			connected_devices.enable_device(dev);
		}
	});

	// Initial population of the device list
	std::cout << "\n======================================" << std::endl;
	std::cout << "Initialising devices..." << std::endl;
	for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
	{
		connected_devices.enable_device(dev);
		
		// Print camera information
		auto dev_name = dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
		auto dev_serial = dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
		auto dev_firmware = dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION);

		std::cout << "--------------------------------------" << std::endl;
		std::cout << "Device name: " << dev_name << std::endl;
		std::cout << "Serial: " << dev_serial << std::endl;
		std::cout << "Firmware: " << dev_firmware << std::endl;

	}

	std::cout << "\n======================================" << std::endl;
	const size_t num_frames_to_capture = 30;
	std::cout << "Capturing " << num_frames_to_capture << " frames...";
	for (auto&& ii = 0; ii < num_frames_to_capture; ++ii)
	{
		connected_devices.try_wait_frames();
	}
	std::cout << "complete\n" << std::endl;
	connected_devices.write_frames_to_disk();

	// Stop streaming
	connected_devices.stop();
	std::cout << "\nStreaming stopped" << std::endl;

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
