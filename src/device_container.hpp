#include <librealsense2/rs.hpp>
#include <map>
#include <mutex>

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third-party/stb_image_write.h"

class device_container
{
	// Helper struct per pipeline
	struct view_port
	{
		std::map<int, rs2::frame> frames_per_stream;
		rs2::colorizer colorize_frame;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
	};

public:

	void enable_device(rs2::device dev)
	{
		std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		std::lock_guard<std::mutex> lock(_mutex);

		if (_devices.find(serial_number) != _devices.end())
		{
			return; //already in
		}

		// Create a pipeline from the given device
		rs2::pipeline p;
		rs2::config c;
		c.enable_stream(rs2_stream::RS2_STREAM_DEPTH, -1, 1280, 720, rs2_format::RS2_FORMAT_Z16, 15);
		c.enable_stream(rs2_stream::RS2_STREAM_COLOR, -1, 1920, 1080, rs2_format::RS2_FORMAT_RGB8, 15);
		c.enable_stream(rs2_stream::RS2_STREAM_INFRARED, 1, rs2_format::RS2_FORMAT_Y8, 15);
		c.enable_stream(rs2_stream::RS2_STREAM_INFRARED, 2, rs2_format::RS2_FORMAT_Y8, 15);

		c.enable_device(serial_number);
		// Start the pipeline with the configuration
		rs2::pipeline_profile profile = p.start(c);
		// Hold it internally
		_devices.emplace(serial_number, view_port{ {},{}, p, profile });

	}

	void remove_devices(const rs2::event_information& info)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over the list of devices and check if it was disconnected
		auto itr = _devices.begin();
		while (itr != _devices.end())
		{
			if (info.was_removed(itr->second.profile.get_device()))
			{
				itr = _devices.erase(itr);
			}
			else
			{
				++itr;
			}
		}
	}

	size_t device_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _devices.size();
	}

	int stream_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int count = 0;
		for (auto&& sn_to_dev : _devices)
		{
			for (auto&& stream : sn_to_dev.second.frames_per_stream)
			{
				if (stream.second)
				{
					count++;
				}
			}
		}
		return count;
	}

	void try_wait_frames()
	{
		std::lock_guard<std::mutex> lock(_mutex);

		// Go over all device
		for (auto&& view : _devices)
		{
			view.second.colorize_frame;
			// Ask each pipeline if there are new frames available
			rs2::frameset frameset;
			if (view.second.pipe.try_wait_for_frames(&frameset))
			{
				for (int i = 0; i < frameset.size(); i++)
				{
					rs2::frame new_frame = frameset[i];
					int stream_id = new_frame.get_profile().unique_id();
					view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(new_frame); //update view port with the new stream
				}
			}
		}
	}

	void write_frames_to_disk()
	{
		for (auto&& dev : _devices)
		{
			for (auto&& serial_and_frame : dev.second.frames_per_stream)
			{
				auto&& frame = serial_and_frame.second;
				// We can only save video frames as pngs, so we skip the rest
				if (auto vf = frame.as<rs2::video_frame>())
				{
					// Write images to disk
					std::stringstream png_file;
					png_file << "rs-save-to-disk-output_" << dev.first << "_" << vf.get_profile().stream_name() << ".png";
					stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
						vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
					std::cout << "Saved " << png_file.str() << std::endl;

					// Record per-frame metadata for UVC streams
					std::stringstream csv_file;
					csv_file << "rs-save-to-disk-output_" << dev.first << "_" << vf.get_profile().stream_name()
						<< "-metadata.csv";
					metadata_to_csv(vf, csv_file.str());
				}
			}
		}
	}

	// Stop all devices
	void stop()
	{
		for (auto&& dev : _devices)
		{
			dev.second.pipe.stop();
		}
	}

private:
	std::mutex _mutex;
	std::map<std::string, view_port> _devices;

	// Helper function for writing metadata to disk as a csv file
	static void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
	{
		std::ofstream csv;

		csv.open(filename);

		//    std::cout << "Writing metadata to " << filename << endl;
		csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

		// Record all the available metadata attributes
		for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
		{
			if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
			{
				csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
					<< frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
			}
		}

		csv.close();
	}
};