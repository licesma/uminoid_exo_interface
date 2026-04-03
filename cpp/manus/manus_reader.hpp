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

class ManusReader {
public:
    using ManusPose = std::pair<std::optional<ManusHand>, std::optional<ManusHand>>;

    ManusReader(
        const std::string& left_address  = "tcp://localhost:8002",
        const std::string& right_address = "tcp://localhost:8003"
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

    std::thread thread_;
};
