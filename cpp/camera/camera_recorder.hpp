#pragma once

#include <condition_variable>
#include <functional>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "utils/csv_saver.hpp"

namespace camera_constants {
    //General
    constexpr int FRAMERATE = 30;
    constexpr unsigned int FRAME_TIMEOUT_MS = 5000;
    constexpr unsigned int WARMUP_COUNT = 3000;
    // Frame
    constexpr int FRAME_WIDTH = 1920;
    constexpr int FRAME_HEIGHT = 1080;
    constexpr int FRAME_BYTES_PER_PIXEL = 3; // RGB8
    constexpr int FRAME_STRIDE = FRAME_WIDTH * FRAME_BYTES_PER_PIXEL;
    constexpr int FRAME_SIZE = FRAME_STRIDE * FRAME_HEIGHT;
    // Frame Writer
    constexpr int SAVE_BATCH_SIZE = 30;
    constexpr size_t MAX_WRITE_QUEUE_SIZE = 10;
}

struct FrameData {
    std::string filename;
    std::vector<uint8_t> pixels;
};

class CameraRecorder {
public:
    CameraRecorder(const std::string& recording_label,
                   const std::function<void(const std::string&)>& raise_error);
    ~CameraRecorder();

    void collect_loop(const std::function<int()>&  collection_id,
                      const std::function<bool()>& stop,
                      const std::function<bool()>& pause = [] { return false; });

private:
    void start_writer();
    void flush_batch();
    void stop_writer();

    std::string output_dir_;
    std::function<void(const std::string&)> raise_error_;
    CsvSaver csv_;
    rs2::pipeline pipe_;

    // Current batch being filled by main thread
    std::vector<FrameData> batch_;

    // Queue of batches ready to be written
    std::queue<std::vector<FrameData>> write_queue_;
    std::thread writer_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_writer_ = false;
};
