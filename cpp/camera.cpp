// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <csignal>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "utils/stb_image_write.h"

static volatile bool running = true;

void signal_handler(int) { running = false; }

int main(int argc, char* argv[]) try
{
    std::signal(SIGINT, signal_handler);

    // Create output directory
    const std::string output_dir = "color_frames";
    std::filesystem::create_directories(output_dir);

    // Configure the pipeline for D435i color stream at max resolution
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 6);

    rs2::pipeline pipe;
    pipe.start(cfg);

    // Let auto-exposure settle
    std::cout << "Warming up camera (30 frames)..." << std::endl;
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();

    std::cout << "Recording color frames to '" << output_dir << "/' — press Ctrl+C to stop." << std::endl;

    int frame_count = 0;

    while (running)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();

        if (!color) continue;

        // Build filename: color_frames/frame_000001.png
        std::ostringstream filename;
        filename << output_dir << "/frame_"
                 << std::setfill('0') << std::setw(6) << frame_count
                 << ".png";

        stbi_write_png(filename.str().c_str(),
                       color.get_width(), color.get_height(),
                       color.get_bytes_per_pixel(),
                       color.get_data(),
                       color.get_stride_in_bytes());

        ++frame_count;

        if (frame_count % 30 == 0)
            std::cout << "Saved " << frame_count << " frames" << std::endl;
    }

    pipe.stop();
    std::cout << "\nDone. Saved " << frame_count << " frames to '" << output_dir << "/'" << std::endl;

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function()
              << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
