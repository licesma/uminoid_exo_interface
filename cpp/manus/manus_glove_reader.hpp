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
    constexpr int POLL_MS = 100;
}

class ManusGloveReader {
public:
    ManusGloveReader(
        const std::string& address,
        HandSide side,
        const std::string& side_name,
        const std::function<void(const std::string&)>& raise_error = nullptr,
        const std::function<void()>& notify_change = nullptr
    );
    ~ManusGloveReader();

    ManusGloveReader(const ManusGloveReader&) = delete;
    ManusGloveReader& operator=(const ManusGloveReader&) = delete;

    std::optional<ManusHand> snapshot() const;
    std::optional<ManusHand> wait_for_next();

    uint64_t frame_seq() const;
    bool stopped() const;

    void stop();

private:
    static ManusHand parse_zmq(const std::string& msg, HandSide side);
    void read_loop();
    void notify_waiters();

    zmq::context_t ctx_;
    zmq::socket_t  sock_;
    HandSide       side_;
    std::string    side_name_;

    std::function<void(const std::string&)> raise_error_;
    std::function<void()>                   notify_change_;
    std::thread                             thread_;

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    std::optional<ManusHand> latest_;
    uint64_t                 frame_seq_{0};
    uint64_t                 consumed_seq_{0};
    bool                     stopped_{false};
};
