#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <zmq.hpp>

#include "manus_hand.hpp"

namespace manus_defaults {
    inline const std::string LEFT_ADDRESS  = "tcp://localhost:8002";
    inline const std::string RIGHT_ADDRESS = "tcp://localhost:8003";
    constexpr int POLL_MS = 100;
}

class ManusReader {
public:
    using ManusPose = std::pair<std::optional<ManusHand>, std::optional<ManusHand>>;

    ManusReader(
        const std::string& left_address  = manus_defaults::LEFT_ADDRESS,
        const std::string& right_address = manus_defaults::RIGHT_ADDRESS,
        const std::function<void(const std::string&)>& raise_error = nullptr
    );
    ~ManusReader();

    ManusPose step();
    ManusPose wait_for_next();
    std::optional<ManusPose> wait_for_next(const std::function<bool()>& stop_requested);

    void stop();

private:
    static ManusHand parse_zmq(const std::string& msg, HandSide side);
    void loop();

    zmq::context_t ctx_;
    zmq::socket_t  left_sock_;
    zmq::socket_t  right_sock_;

    std::mutex              lock_;
    std::condition_variable cv_;
    bool                    new_data_{false};
    std::optional<ManusHand> left_;
    std::optional<ManusHand> right_;
    std::atomic<bool>       running_{true};
    std::function<void(const std::string&)> raise_error_;

    std::thread thread_;
};
