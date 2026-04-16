#include "camera/camera_recorder.hpp"
#include "collect_ui.hpp"
#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "upper_body_reader/upper_body_reader.hpp"
#include "utils/recording_label.hpp"
#include "utils/repo_constants.hpp"

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace config {
    static const std::string PATH = (std::filesystem::path(__FILE__).parent_path() / "collect_config.yaml").lexically_normal().string();
    static const YAML::Node yaml = YAML::LoadFile(PATH);

    inline const std::string left_arm   = yaml["upper_body"]["left_device"].as<std::string>();
    inline const std::string right_arm  = yaml["upper_body"]["right_device"].as<std::string>();
    inline const int         baudrate   = yaml["upper_body"]["baudrate"].as<int>();
    inline const std::string left_hand  = yaml["inspire"]["left_device"].as<std::string>();
    inline const std::string right_hand = yaml["inspire"]["right_device"].as<std::string>();
}

int main() {
    const std::string recording_label = generate_recording_label();

    std::atomic<bool> running{true};
    std::atomic<bool> _pause{false};
    std::atomic<int>  _collection_id{1};
    std::string       error_msg;

    auto stop          = [&running]        { return !running.load(); };
    auto paused        = [&_pause]        { return _pause.load(); };
    auto collection_id = [&_collection_id] { return _collection_id.load(); };
    auto raise_error   = [&](const std::string& msg) {
        error_msg = msg;
        ui::cancel_current(_collection_id.load());
        running.store(false);
    };

    std::filesystem::create_directories(repo_constants::DATA_DIR + "/" + recording_label);

    UpperBodyReader   upper_body(config::left_arm, config::right_arm, config::baudrate, recording_label, raise_error);
    InspireRetargeter inspire(config::left_hand, config::right_hand, recording_label, raise_error);
    CameraRecorder    camera(recording_label, raise_error);

    std::cout << "  " << recording_label
              << "   space/arrows → next collection   q/ctrl+c → stop\n";

    ui::add_collection(_collection_id.load());

    std::thread camera_thread    ([&] { camera.collect_loop(collection_id, stop, paused); });
    std::thread upper_body_thread([&] { upper_body.collect_loop(collection_id, stop, paused); });
    std::thread inspire_thread   ([&] { inspire.retarget_loop(stop, collection_id, paused); });

    std::thread display_thread([&] {
        while (running.load()) {
            ui::redraw();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        ui::redraw();
    });

    {
        RawMode raw;
        while (running.load()) {
            char key = 0;
            if (read(STDIN_FILENO, &key, 1) <= 0)
                continue;

            if (key == 'q' || key == 'Q' || key == 3 /* Ctrl+C */) {
                ui::complete_current(_collection_id.load());
                running.store(false);
            } else if (key == ' ') {
                // Complete current, queue next (pause saving until arrow)
                ui::complete_current(_collection_id.load());
                _collection_id.fetch_add(1);
                ui::add_next(_collection_id.load());
                _pause.store(true);
            } else if (key == '\033') {
                // Arrow keys: start the queued Next collection
                char seq[2] = {};
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && seq[0] == '[')
                    if (read(STDIN_FILENO, &seq[1], 1) > 0 &&
                        (seq[1] == 'A' || seq[1] == 'B' || seq[1] == 'C' || seq[1] == 'D'))
                        if (ui::start_next_collection())
                            _pause.store(false);
            }
        }
    }

    display_thread.join();
    camera_thread.join();
    upper_body_thread.join();
    inspire_thread.join();

    if (!error_msg.empty())
        std::cerr << "\n[FATAL] " << error_msg << "\n";

    return 0;
}
