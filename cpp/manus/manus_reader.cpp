#include "manus_reader.hpp"
#include <sstream>
#include <stdexcept>

ManusReader::ManusReader(const std::string& left_address,
                         const std::string& right_address,
                         const std::function<void(const std::string&)>& raise_error
                        )
    : ctx_(1),
      left_sock_(ctx_, zmq::socket_type::pull),
      right_sock_(ctx_, zmq::socket_type::pull),
      raise_error_(raise_error)
{
    try {
        left_sock_.set(zmq::sockopt::conflate, 1);
        left_sock_.connect(left_address);

        right_sock_.set(zmq::sockopt::conflate, 1);
        right_sock_.connect(right_address);
    } catch (const zmq::error_t& e) {
        if (raise_error_) raise_error_(std::string("[ManusReader] connect failed: ") + e.what());
        return;
    }

    thread_ = std::thread(&ManusReader::loop, this);
}

ManusReader::~ManusReader() {
    stop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

ManusHand ManusReader::parse_zmq(const std::string& msg, HandSide side) {
    std::array<std::array<float, 3>, 25> landmarks{};
    std::istringstream ss(msg);
    std::string token;
    int idx = 0;

    while (std::getline(ss, token, ',') && idx < 25 * 7) {
        int row = idx / 7;
        int col = idx % 7;
        if (col < 3) {
            landmarks[row][col] = std::stof(token);
        }
        idx++;
    }
    return ManusHand(landmarks, side);
}

void ManusReader::loop() {
    zmq::pollitem_t items[] = {
        { left_sock_.handle(),  0, ZMQ_POLLIN, 0 },
        { right_sock_.handle(), 0, ZMQ_POLLIN, 0 },
    };

    try {
        while (running_.load()) {
            if (zmq::poll(items, 2, std::chrono::milliseconds(manus_defaults::POLL_MS)) == 0)
                throw std::runtime_error("No data for " + std::to_string(manus_defaults::POLL_MS) + "ms — publisher dead?");

            zmq::message_t msg;
            std::lock_guard<std::mutex> guard(lock_);

            if (items[0].revents & ZMQ_POLLIN) {
                (void)left_sock_.recv(msg, zmq::recv_flags::none);
                left_ = parse_zmq(msg.to_string(), HandSide::LEFT);
                new_data_ = true;
            }

            if (items[1].revents & ZMQ_POLLIN) {
                (void)right_sock_.recv(msg, zmq::recv_flags::none);
                right_ = parse_zmq(msg.to_string(), HandSide::RIGHT);
                new_data_ = true;
            }

            if (new_data_) {
                cv_.notify_one();
            }
        }
    } catch (const std::exception& e) {
        if (raise_error_) raise_error_(std::string("[ManusReader] ") + e.what());
    }
}

ManusReader::ManusPose ManusReader::step() {
    std::lock_guard<std::mutex> guard(lock_);
    return {left_, right_};
}

ManusReader::ManusPose ManusReader::wait_for_next() {
    std::unique_lock<std::mutex> guard(lock_);
    cv_.wait(guard, [this] { return new_data_ || !running_.load(); });
    new_data_ = false;
    return {left_, right_};
}

std::optional<ManusReader::ManusPose> ManusReader::wait_for_next(
    const std::function<bool()>& stop_requested
) {
    std::unique_lock<std::mutex> guard(lock_);
    cv_.wait(guard, [this, &stop_requested] {
        return new_data_ || !running_.load() || stop_requested();
    });

    if (stop_requested() || !new_data_) {
        return std::nullopt;
    }

    new_data_ = false;
    return ManusPose{left_, right_};
}

void ManusReader::stop() {
    running_.store(false);
    cv_.notify_all();
    try {
        left_sock_.close();
        right_sock_.close();
    } catch (...) {}
}
