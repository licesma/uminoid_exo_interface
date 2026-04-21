#pragma once

#include <functional>
#include <string>
#include <zmq.hpp>

#include "camera/camera_recorder.hpp"
#include "utils/csv_saver.hpp"

class PreviewServer;

namespace zmq_camera_constants {
    // G1 raw RGB bridge (see g1_files/realsense_raw_rgb.py).
    constexpr const char* ENDPOINT = "tcp://192.168.123.164:5556";
    constexpr int RECV_TIMEOUT_MS = 2000;
}

// G1 head-camera recorder: consumes the server-driven raw RGB stream produced.
class ZmqCameraRecorder : public CameraRecorder {
public:
    ZmqCameraRecorder(const std::string& recording_label,
                      const std::function<void(const std::string&)>& raise_error,
                      PreviewServer* preview = nullptr);

    void collect_loop(const std::function<int()>&  collection_id,
                      const std::function<bool()>& stop,
                      const std::function<bool()>& pause) override;

private:
    bool write_bytes(const std::string& filename,
                     const uint8_t* data, size_t size) const;

    std::string output_dir_;
    std::function<void(const std::string&)> raise_error_;
    CsvSaver csv_;
    zmq::context_t ctx_;
    zmq::socket_t  sock_;
    PreviewServer* preview_ = nullptr;
};
