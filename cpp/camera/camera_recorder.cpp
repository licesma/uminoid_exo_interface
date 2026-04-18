#include "camera_recorder.hpp"

using namespace camera_constants;

#include "utils/repo_constants.hpp"
#include "utils/time.hpp"

#include <librealsense2/rs.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <fstream>

CameraRecorder::CameraRecorder(const std::string& recording_label,
                               const std::function<void(const std::string&)>& raise_error)
    : output_dir_(repo_constants::DATA_DIR + "/" + recording_label),
      raise_error_(raise_error),
      csv_(output_dir_ + "/camera.csv", "collection_id,frame_number,camera_timestamp_ms,host_timestamp_ms"),
      pipe_(context_) {
    try {
        std::filesystem::create_directories(output_dir_ + "/frames");

        if (context_.query_devices().size() == 0) {
            raise_error_("[Camera] No camera connected");
            return;
        }

        context_.set_devices_changed_callback([this](rs2::event_information info) {
            if (info.was_removed(device_)) {
                raise_error_("[Camera] Camera disconnected");
            }
        });

        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, RS2_FORMAT_RGB8, FRAMERATE);
        device_ = pipe_.start(cfg).get_device();
    } catch (const std::exception& e) {
        raise_error_(std::string("[Camera] ") + e.what());
    }
}

bool CameraRecorder::write_frame(const std::string& filename, const uint8_t* data) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) return false;

    out.write(reinterpret_cast<const char*>(data),
              static_cast<std::streamsize>(FRAME_SIZE));
    return static_cast<bool>(out);
}

void CameraRecorder::collect_loop(const std::function<int()>&  collection_id,
                                  const std::function<bool()>& stop,
                                  const std::function<bool()>& pause) {
    try {
        const std::string frames_dir = output_dir_ + "/frames";
        auto camera_disconnected = [this] {
            return !device_ || !device_.is_connected();
        };

        for (int i = 0; i < WARMUP_COUNT && !stop(); ++i) {
            rs2::frameset frames;
            if (!pipe_.try_wait_for_frames(&frames, FRAME_TIMEOUT_MS) && camera_disconnected()) {
                raise_error_("[Camera] Camera disconnected");
                break;
            }
        }

        int frame_count = 0;

        while (!stop()) {
            rs2::frameset frames;
            if (!pipe_.try_wait_for_frames(&frames, FRAME_TIMEOUT_MS)) {
                if (camera_disconnected()) {
                    raise_error_("[Camera] Camera disconnected");
                    break;
                }
                continue;
            }
            rs2::video_frame color = frames.get_color_frame();

            if (!color || pause()) continue;

            double camera_ts = color.get_timestamp();
            uint64_t host_ts = Time::ts();

            std::ostringstream row;
            row << collection_id() << "," << frame_count << "," << std::fixed << std::setprecision(3) << camera_ts << "," << host_ts;
            csv_.write_line(row.str());

            const uint8_t* data = static_cast<const uint8_t*>(color.get_data());

            std::ostringstream filename;
            filename << frames_dir << "/frame_"
                     << std::setfill('0') << std::setw(6) << frame_count
                     << ".raw";

            if (!write_frame(filename.str(), data)) {
                raise_error_("[Camera] Failed to write " + filename.str());
                break;
            }

            ++frame_count;
        }

        csv_.close();
        pipe_.stop();
    } catch (const std::exception& e) {
        raise_error_(std::string("[Camera] ") + e.what());
    }
}
