#pragma once

#include <cstddef>
#include <functional>

// Raw RGB8 layout shared by USB and ZMQ recorders (480x640 HxW).
namespace camera_frame {
constexpr int FRAME_WIDTH = 640;
constexpr int FRAME_HEIGHT = 480;
constexpr int FRAME_BYTES_PER_PIXEL = 3;  // RGB8
constexpr int FRAME_STRIDE = FRAME_WIDTH * FRAME_BYTES_PER_PIXEL;
constexpr std::size_t FRAME_SIZE =
    static_cast<std::size_t>(FRAME_STRIDE) * static_cast<std::size_t>(FRAME_HEIGHT);
}

// Abstract camera recorder for  RealSense camera.
class CameraRecorder {
public:
    virtual ~CameraRecorder() = default;

    // Drive the recorder until `stop()` returns true. While `pause()` returns
    // true, frames are still consumed (so the preview stays fresh) but not
    // written to disk.
    virtual void collect_loop(const std::function<int()>&  collection_id,
                              const std::function<bool()>& stop,
                              const std::function<bool()>& pause) = 0;
};
