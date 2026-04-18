#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "manus_glove_reader.hpp"
#include "manus_hand.hpp"

namespace manus_defaults {
    inline const std::string LEFT_ADDRESS  = "tcp://localhost:8002";
    inline const std::string RIGHT_ADDRESS = "tcp://localhost:8003";
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
    void notify_change();
    bool has_new_data_locked() const;
    bool any_reader_stopped_locked() const;

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    uint64_t                left_consumed_seq_{0};
    uint64_t                right_consumed_seq_{0};
    ManusGloveReader        left_;
    ManusGloveReader        right_;
};
