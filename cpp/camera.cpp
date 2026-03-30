#include "camera/camera_recorder.hpp"

#include <iostream>
#include <librealsense2/rs.hpp>

int main() try
{
    CameraRecorder recorder("color_frames", 30);
    recorder.record();
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
