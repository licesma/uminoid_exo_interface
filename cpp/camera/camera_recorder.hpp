#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "utils/csv_saver.hpp"

constexpr int FRAME_WIDTH = 1920;
constexpr int FRAME_HEIGHT = 1080;
constexpr int FRAME_BYTES_PER_PIXEL = 3; // RGB8
constexpr int FRAME_STRIDE = FRAME_WIDTH * FRAME_BYTES_PER_PIXEL;
constexpr int FRAME_SIZE = FRAME_STRIDE * FRAME_HEIGHT;

struct FrameData {
    std::string filename;
    std::vector<uint8_t> pixels;
};

class CameraRecorder {
public:
    CameraRecorder(const std::string& recording_label, int framerate, int save_batch_size = 30);
    ~CameraRecorder();

    void collect_loop(const std::function<int()>& collection_id,
                      const std::function<bool()>& stop);

private:
    void start_writer();
    void flush_batch();
    void stop_writer();

    std::string output_dir_;
    CsvSaver csv_;
    int framerate;
    int save_batch_size_;

    // Current batch being filled by main thread
    std::vector<FrameData> batch_;

    // Queue of batches ready to be written
    std::queue<std::vector<FrameData>> write_queue_;
    std::thread writer_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_writer_ = false;
};
