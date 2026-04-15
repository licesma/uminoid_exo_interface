#include "camera/camera_recorder.hpp"
#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "upper_body_reader/upper_body_reader.hpp"
#include "utils/recording_label.hpp"
#include "utils/repo_constants.hpp"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace config {
    static const std::string PATH = (std::filesystem::path(__FILE__).parent_path() / "collect_config.yaml").lexically_normal().string();
    static const YAML::Node yaml = YAML::LoadFile(PATH);

    inline const std::string left_arm  = yaml["upper_body"]["left_device"].as<std::string>();
    inline const std::string right_arm = yaml["upper_body"]["right_device"].as<std::string>();
    inline const int         baudrate  = yaml["upper_body"]["baudrate"].as<int>();
    inline const std::string left_hand  = yaml["inspire"]["left_device"].as<std::string>();
    inline const std::string right_hand = yaml["inspire"]["right_device"].as<std::string>();
}

int main() {
    const std::string recording_label = generate_recording_label();

    std::atomic<bool> running{true};
    std::atomic<int>  _collection_id{1};

    auto stop         = [&running]        { return !running.load(); };
    auto collection_id = [&_collection_id] { return _collection_id.load(); };

    std::filesystem::create_directories(repo_constants::DATA_DIR + "/" + recording_label);

    UpperBodyReader upper_body(config::left_arm, config::right_arm, config::baudrate, recording_label);
    InspireRetargeter inspire(config::left_hand, config::right_hand, recording_label);
    //CameraRecorder camera(recording_label, 30);

    std::cout << "Collection running [" << recording_label << "] — 'space' to increment collection, 'q' to stop." << std::endl;
/*
    std::thread camera_thread([&] {
        camera.collect_loop(collection_id, stop);
    });
*/
    std::thread upper_body_thread([&] {
        upper_body.collect_loop(collection_id, stop);
    });

    std::thread inspire_thread([&] {
        inspire.retarget_loop(stop, collection_id);
    });

    char key = 0;
    while (running.load() && std::cin.get(key)) {
        if (key == 'q' || key == 'Q')
            running.store(false);
        else if (key == ' ') {
            _collection_id++;
            std::cout << "Collection ID: " << _collection_id.load() << std::endl;
        }
    }

    //camera_thread.join();
    upper_body_thread.join();
    inspire_thread.join();

    return 0;
}
