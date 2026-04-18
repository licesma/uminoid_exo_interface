#include "manus_reader.hpp"

ManusReader::ManusReader(const std::string& left_address,
                         const std::string& right_address,
                         const std::function<void(const std::string&)>& raise_error
                        )
    : left_(left_address, HandSide::LEFT, "left", raise_error, [this] { notify_change(); }),
      right_(right_address, HandSide::RIGHT, "right", raise_error, [this] { notify_change(); })
{
}

ManusReader::~ManusReader() {
    stop();
}

ManusReader::ManusPose ManusReader::step() {
    return {left_.snapshot(), right_.snapshot()};
}

ManusReader::ManusPose ManusReader::wait_for_next() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return has_new_data_locked() || any_reader_stopped_locked(); });

    left_consumed_seq_ = left_.frame_seq();
    right_consumed_seq_ = right_.frame_seq();
    return step();
}

std::optional<ManusReader::ManusPose> ManusReader::wait_for_next(
    const std::function<bool()>& stop_requested
) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this, &stop_requested] {
        return has_new_data_locked() || any_reader_stopped_locked() || stop_requested();
    });

    if (stop_requested() || any_reader_stopped_locked()) {
        return std::nullopt;
    }

    left_consumed_seq_ = left_.frame_seq();
    right_consumed_seq_ = right_.frame_seq();
    return step();
}

void ManusReader::stop() {
    left_.stop();
    right_.stop();
    notify_change();
}

void ManusReader::notify_change() {
    std::lock_guard<std::mutex> lock(mutex_);
    cv_.notify_all();
}

bool ManusReader::has_new_data_locked() const {
    return left_.frame_seq() > left_consumed_seq_ || right_.frame_seq() > right_consumed_seq_;
}

bool ManusReader::any_reader_stopped_locked() const {
    return left_.stopped() || right_.stopped();
}
