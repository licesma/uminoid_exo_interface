#include "camera/camera_recorder.hpp"

#include <csignal>
#include <iostream>
#include <librealsense2/rs.hpp>

static volatile sig_atomic_t running = 1;
static void sigint_handler(int) { running = 0; }

int main() try
{
    std::signal(SIGINT, sigint_handler);

    CameraRecorder recorder("color_frames", 30);
    recorder.collect_loop([] { return !running; });
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
