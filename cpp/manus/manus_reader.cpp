#include "manus_reader.hpp"
#include <sstream>

ManusReader::ManusReader(const std::string& left_address,
                         const std::string& right_address)
    : ctx_(1),
      left_sock_(ctx_, zmq::socket_type::pull),
      right_sock_(ctx_, zmq::socket_type::pull)
{
    left_sock_.set(zmq::sockopt::conflate, 1);
    left_sock_.connect(left_address);

    right_sock_.set(zmq::sockopt::conflate, 1);
    right_sock_.connect(right_address);

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

    while (running_.load()) {
        try {
            zmq::poll(items, 2, std::chrono::milliseconds(100));
        } catch (const zmq::error_t&) {
            break;
        }

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
}

std::pair<std::optional<ManusHand>, std::optional<ManusHand>> ManusReader::step() {
    std::lock_guard<std::mutex> guard(lock_);
    return {left_, right_};
}

std::pair<std::optional<ManusHand>, std::optional<ManusHand>> ManusReader::wait_for_next() {
    std::unique_lock<std::mutex> guard(lock_);
    cv_.wait(guard, [this] { return new_data_ || !running_.load(); });
    new_data_ = false;
    return {left_, right_};
}

void ManusReader::stop() {
    running_.store(false);
    cv_.notify_all();
    try {
        left_sock_.close();
        right_sock_.close();
    } catch (...) {}
}
