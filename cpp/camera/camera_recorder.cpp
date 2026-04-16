#include "camera_recorder.hpp"

using namespace camera_constants;

#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

#include <librealsense2/rs.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "utils/stb_image_write.h"

CameraRecorder::CameraRecorder(const std::string& recording_label)
    : output_dir_(repo_constants::DATA_DIR + "/" + recording_label),
      csv_(output_dir_ + "/camera.csv", "collection_id,frame_number,camera_timestamp_ms,host_timestamp_ms") {
    batch_.reserve(SAVE_BATCH_SIZE);
}

CameraRecorder::~CameraRecorder() {
    stop_writer();
}

void CameraRecorder::start_writer(const std::function<void(const std::string&)>& raise_error) {
    writer_ = std::thread([this, &raise_error] {
        while (true) {
            std::vector<FrameData> batch;

            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this] { return !write_queue_.empty() || stop_writer_; });

                if (stop_writer_ && write_queue_.empty()) break;

                if (write_queue_.size() >= MAX_WRITE_QUEUE_SIZE) {
                    raise_error("[Camera] Write queue full — disk I/O can't keep up");
                    return;
                }

                batch = std::move(write_queue_.front());
                write_queue_.pop();
            }

            for (auto& f : batch) {
                int ok = stbi_write_png(f.filename.c_str(),
                                        FRAME_WIDTH, FRAME_HEIGHT,
                                        FRAME_BYTES_PER_PIXEL,
                                        f.pixels.data(),
                                        FRAME_STRIDE);
                if (!ok) {
                    raise_error("[Camera] Failed to write " + f.filename);
                    return;
                }
            }
        }
    });
}

void CameraRecorder::flush_batch() {
    if (batch_.empty()) return;

    std::lock_guard<std::mutex> lock(mtx_);
    write_queue_.push(std::move(batch_));
    batch_.clear();
    batch_.reserve(SAVE_BATCH_SIZE);
    cv_.notify_one();
}

void CameraRecorder::stop_writer() {
    if (!writer_.joinable()) return;

    flush_batch();

    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_writer_ = true;
    }
    cv_.notify_one();
    writer_.join();
}

void CameraRecorder::collect_loop(const std::function<int()>& collection_id,
                                  const std::function<bool()>& stop,
                                  const std::function<void(const std::string&)>& raise_error) {
    try {
        const std::string frames_dir = output_dir_ + "/frames";
        std::filesystem::create_directories(frames_dir);

        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, FRAMERATE);

        rs2::pipeline pipe;
        pipe.start(cfg);

        for (int i = 0; i < WARMUP_COUNT; ++i) pipe.wait_for_frames(FRAME_TIMEOUT_MS);

        start_writer(raise_error);

        int frame_count = 0;

        while (!stop()) {
            rs2::frameset frames = pipe.wait_for_frames(FRAME_TIMEOUT_MS);
            rs2::video_frame color = frames.get_color_frame();

            if (!color) continue;

            double camera_ts = color.get_timestamp();
            uint64_t host_ts = Time::ts();

            std::ostringstream row;
            row << collection_id() << "," << frame_count << "," << std::fixed << std::setprecision(3) << camera_ts << "," << host_ts;
            csv_.write_line(row.str());

            const uint8_t* data = static_cast<const uint8_t*>(color.get_data());

            std::ostringstream filename;
            filename << frames_dir << "/frame_"
                     << std::setfill('0') << std::setw(6) << frame_count
                     << ".png";

            batch_.push_back({
                filename.str(),
                std::vector<uint8_t>(data, data + FRAME_SIZE),
            });

            ++frame_count;

            if (static_cast<int>(batch_.size()) >= SAVE_BATCH_SIZE)
                flush_batch();
        }

        csv_.close();
        stop_writer();
        pipe.stop();
    } catch (const std::exception& e) {
        raise_error(std::string("[Camera] ") + e.what());
    }
}
