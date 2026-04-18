#include "manus_glove_reader.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

ManusGloveReader::ManusGloveReader(
    const std::string& address,
    HandSide side,
    const std::string& side_name,
    const std::function<void(const std::string&)>& raise_error,
    const std::function<void()>& notify_change
)
    : ctx_(1),
      sock_(ctx_, zmq::socket_type::pull),
      side_(side),
      side_name_(side_name),
      raise_error_(raise_error),
      notify_change_(notify_change)
{
    if (address.empty()) {
        stopped_ = true;
        return;
    }

    try {
        sock_.set(zmq::sockopt::conflate, 1);
        sock_.connect(address);
    } catch (const zmq::error_t& e) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        notify_waiters();
        if (raise_error_) {
            raise_error_("[ManusGloveReader] " + side_name_ + " connect failed: " + e.what());
        }
        return;
    }

    thread_ = std::thread(&ManusGloveReader::read_loop, this);
}

ManusGloveReader::~ManusGloveReader() {
    stop();
}

std::optional<ManusHand> ManusGloveReader::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

std::optional<ManusHand> ManusGloveReader::wait_for_next() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return stopped_ || frame_seq_ > consumed_seq_; });

    if (frame_seq_ > consumed_seq_) {
        consumed_seq_ = frame_seq_;
        return latest_;
    }
    return std::nullopt;
}

uint64_t ManusGloveReader::frame_seq() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_seq_;
}

bool ManusGloveReader::stopped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_;
}

void ManusGloveReader::stop() {
    bool already_stopped = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        already_stopped = stopped_;
        stopped_ = true;
    }

    notify_waiters();

    if (already_stopped) {
        if (thread_.joinable()) thread_.join();
        return;
    }

    try {
        sock_.close();
    } catch (...) {}

    if (thread_.joinable()) thread_.join();
}

ManusHand ManusGloveReader::parse_zmq(const std::string& msg, HandSide side) {
    std::array<std::array<float, 3>, 25> landmarks{};
    std::istringstream ss(msg);
    std::string token;
    int idx = 0;

    while (std::getline(ss, token, ',') && idx < 25 * 7) {
        const int row = idx / 7;
        const int col = idx % 7;
        if (col < 3) {
            landmarks[row][col] = std::stof(token);
        }
        idx++;
    }
    return ManusHand(landmarks, side);
}

void ManusGloveReader::read_loop() {
    zmq::pollitem_t item[] = {
        { sock_.handle(), 0, ZMQ_POLLIN, 0 },
    };

    const auto stale_after = std::chrono::milliseconds(manus_defaults::POLL_MS);
    auto last_packet = std::chrono::steady_clock::now();

    try {
        while (true) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopped_) break;
            }

            (void)zmq::poll(item, 1, stale_after);
            const auto now = std::chrono::steady_clock::now();

            if (item[0].revents & ZMQ_POLLIN) {
                zmq::message_t msg;
                (void)sock_.recv(msg, zmq::recv_flags::none);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = parse_zmq(msg.to_string(), side_);
                    last_packet = now;
                    ++frame_seq_;
                }
                notify_waiters();
                continue;
            }

            if (now - last_packet > stale_after) {
                throw std::runtime_error(
                    "No " + side_name_ + " glove data for " + std::to_string(manus_defaults::POLL_MS) + "ms");
            }
        }
    } catch (const std::exception& e) {
        if (!stopped() && raise_error_) {
            raise_error_(std::string("[ManusGloveReader] ") + e.what());
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
    }
    notify_waiters();
}

void ManusGloveReader::notify_waiters() {
    cv_.notify_all();
    if (notify_change_) notify_change_();
}
