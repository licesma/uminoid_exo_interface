#pragma once

#include <functional>
#include <librealsense2/rs.hpp>
#include <string>

#include "utils/csv_saver.hpp"

namespace camera_constants {
    //General
    constexpr int FRAMERATE = 30;
    constexpr unsigned int FRAME_TIMEOUT_MS = 5000;
    constexpr unsigned int WARMUP_COUNT = 30;
    // Frame — raw RGB8 frames in 480x640 definition (HxW).
    constexpr int FRAME_WIDTH = 640;
    constexpr int FRAME_HEIGHT = 480;
    constexpr int FRAME_BYTES_PER_PIXEL = 3; // RGB8
    constexpr int FRAME_STRIDE = FRAME_WIDTH * FRAME_BYTES_PER_PIXEL;
    constexpr int FRAME_SIZE = FRAME_STRIDE * FRAME_HEIGHT;
}

class CameraRecorder {
public:
    CameraRecorder(const std::string& recording_label,
                   const std::function<void(const std::string&)>& raise_error);

    void collect_loop(const std::function<int()>&  collection_id,
                      const std::function<bool()>& stop,
                      const std::function<bool()>& pause = [] { return false; });

private:
    bool write_frame(const std::string& filename, const uint8_t* data) const;

    std::string output_dir_;
    std::function<void(const std::string&)> raise_error_;
    CsvSaver csv_;
    rs2::context context_;
    rs2::pipeline pipe_;
    rs2::device device_;
};
