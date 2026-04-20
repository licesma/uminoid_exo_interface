#include "camera/camera_recorder.hpp"
#include "collect_ui.hpp"
#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "upper_body_reader/upper_body_reader.hpp"
#include "utils/recording_label.hpp"
#include "utils/repo_constants.hpp"

#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace config {
    static const std::string PATH = (std::filesystem::path(__FILE__).parent_path() / "collect_config.yaml").lexically_normal().string();
    static const YAML::Node yaml = YAML::LoadFile(PATH);

    inline const std::string left_arm   = yaml["upper_body"]["left_device"].as<std::string>();
    inline const std::string right_arm  = yaml["upper_body"]["right_device"].as<std::string>();
    inline const int         baudrate   = yaml["upper_body"]["baudrate"].as<int>();

    inline const uint8_t inspire_left_id  = yaml["inspire"]["left_id"].as<int>();
    inline const uint8_t inspire_right_id = yaml["inspire"]["right_id"].as<int>();
}

int main() {
    const std::string recording_label = generate_recording_label();

    std::atomic<bool> running{true};
    std::atomic<bool> _paused{false};
    std::atomic<int>  _collection_id{1};
    std::atomic<bool> error_reported{false};
    std::string       error_msg;
    std::mutex        error_msg_mutex;

    auto stop          = [&running]        { return !running.load(); };
    auto paused        = [&_paused]        { return _paused.load(); };
    auto collection_id = [&_collection_id] { return _collection_id.load(); };
    auto raise_error   = [&](const std::string& msg) {
        if (error_reported.exchange(true)) return;
        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            error_msg = msg;
        }
        ui::cancel_current(_collection_id.load());
        running.store(false);
    };

    std::filesystem::create_directories(repo_constants::DATA_DIR + "/" + recording_label);

     UpperBodyReader   upper_body(config::left_arm, config::right_arm, config::baudrate, recording_label, raise_error);
    InspireRetargeter inspire(config::inspire_left_id, config::inspire_right_id, recording_label, raise_error);
    CameraRecorder    camera(recording_label, raise_error);

    std::cout << "  " << recording_label
              << "   space → next   x → cancel   q → stop\n";

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
                if (!_paused.load()) {
                    // Collecting → pause: complete current, queue next
                    ui::complete_current(_collection_id.load());
                    _collection_id.fetch_add(1);
                    ui::add_next(_collection_id.load());
                    _paused.store(true);
                } else {
                    // Paused → resume: start the queued Next collection
                    if (ui::start_next_collection())
                        _paused.store(false);
                }
            } else if (key == 'x' || key == 'X') {
                if (!_paused.load()) {
                    // Collecting → cancel current, queue next
                    ui::cancel_current(_collection_id.load());
                    _collection_id.fetch_add(1);
                    ui::add_next(_collection_id.load());
                    _paused.store(true);
                }
            }
        }
    }

    ui::save_status_csv(repo_constants::DATA_DIR + "/" + recording_label + "/collection_status.csv");

    display_thread.join();
    //camera_thread.join();
    upper_body_thread.join();
    //inspire_thread.join();

    if (error_reported.load()) {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        std::cerr << "\n[FATAL] " << error_msg << "\n";
    }

    return 0;
}
