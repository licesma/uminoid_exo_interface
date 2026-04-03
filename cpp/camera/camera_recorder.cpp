#include "camera_recorder.hpp"

#include "utils/csv_saver.hpp"

#include <librealsense2/rs.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "utils/stb_image_write.h"

CameraRecorder::CameraRecorder(const std::string& output_dir, int framerate, int save_batch_size)
    : output_dir_(output_dir), framerate(framerate), save_batch_size_(save_batch_size) {
    batch_.reserve(save_batch_size_);
}

CameraRecorder::~CameraRecorder() {
    stop_writer();
}

void CameraRecorder::start_writer() {
    writer_ = std::thread([this] {
        while (true) {
            std::vector<FrameData> batch;

            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this] { return !write_queue_.empty() || stop_writer_; });

                if (stop_writer_ && write_queue_.empty()) break;

                batch = std::move(write_queue_.front());
                write_queue_.pop();
            }

            for (auto& f : batch) {
                stbi_write_png(f.filename.c_str(),
                               FRAME_WIDTH, FRAME_HEIGHT,
                               FRAME_BYTES_PER_PIXEL,
                               f.pixels.data(),
                               FRAME_STRIDE);
            }
        }
    });
}

void CameraRecorder::flush_batch() {
    if (batch_.empty()) return;

    std::lock_guard<std::mutex> lock(mtx_);
    write_queue_.push(std::move(batch_));
    batch_.clear();
    batch_.reserve(save_batch_size_);
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

void CameraRecorder::collect_loop(const std::function<bool()>& stop_requested) {
    std::filesystem::create_directories(output_dir_);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, framerate);

    rs2::pipeline pipe;
    pipe.start(cfg);

    std::cout << "Warming up camera (30 frames)..." << std::endl;
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames();

    std::cout << "Recording color frames to '" << output_dir_ << "/' — press Ctrl+C to stop." << std::endl;

    CsvSaver csv(output_dir_ + "/metadata.csv",
                 "frame_number,camera_timestamp_ms,host_timestamp_ms");

    start_writer();

    int frame_count = 0;

    while (!stop_requested()) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();

        if (!color) continue;

        double camera_ts = color.get_timestamp();
        long long host_ts = color.supports_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)
            ? color.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL)
            : 0;

        std::ostringstream row;
        row << frame_count << "," << std::fixed << std::setprecision(3) << camera_ts << "," << host_ts;
        csv.write_line(row.str());

        const uint8_t* data = static_cast<const uint8_t*>(color.get_data());

        std::ostringstream filename;
        filename << output_dir_ << "/frame_"
                 << std::setfill('0') << std::setw(5) << frame_count
                 << ".png";

        batch_.push_back({
            filename.str(),
            std::vector<uint8_t>(data, data + FRAME_SIZE),
        });

        ++frame_count;

        if (static_cast<int>(batch_.size()) >= save_batch_size_)
            flush_batch();

        if (frame_count % 30 == 0)
            std::cout << "Saved " << frame_count << " frames" << std::endl;
    }

    csv.close();
    stop_writer();
    pipe.stop();
    std::cout << "\nDone. Saved " << frame_count << " frames to '" << output_dir_ << "/'" << std::endl;
}
