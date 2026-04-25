#include "amo_zmq_helper.hpp"

#include <chrono>
#include <iostream>

namespace amo_zmq {

zmq::socket_t make_pub_bound(zmq::context_t& ctx, const std::string& endpoint) {
  zmq::socket_t sock(ctx, zmq::socket_type::pub);
  sock.set(zmq::sockopt::sndhwm, 1);
  sock.set(zmq::sockopt::linger, 0);
  sock.bind(endpoint);
  return sock;
}

zmq::socket_t make_sub_conflate(zmq::context_t& ctx, const std::string& endpoint) {
  zmq::socket_t sock(ctx, zmq::socket_type::sub);
  sock.set(zmq::sockopt::subscribe, "");
  sock.set(zmq::sockopt::conflate, 1);
  sock.set(zmq::sockopt::rcvhwm, 1);
  sock.set(zmq::sockopt::linger, 0);
  sock.connect(endpoint);
  return sock;
}

void send_dontwait(zmq::socket_t& sock, const void* data, size_t size) {
  zmq::message_t msg(data, size);
  (void)sock.send(msg, zmq::send_flags::dontwait);
}

void run_receive_loop(zmq::socket_t& sock,
                   const std::atomic<bool>& running,
                   const std::function<void(const uint8_t*, size_t)>& on_msg) {
  while (running.load()) {
    zmq::pollitem_t item[] = {{sock.handle(), 0, ZMQ_POLLIN, 0}};
    try {
      const int rc = zmq::poll(item, 1, std::chrono::milliseconds(50));
      if (rc <= 0) continue;
    } catch (const zmq::error_t& e) {
      if (!running.load()) return;
      std::cerr << "[amo_zmq] poll error: " << e.what() << "\n";
      continue;
    }

    zmq::message_t msg;
    try {
      auto rc = sock.recv(msg, zmq::recv_flags::none);
      if (!rc) continue;
    } catch (const zmq::error_t& e) {
      if (!running.load()) return;
      std::cerr << "[amo_zmq] receive error: " << e.what() << "\n";
      continue;
    }

    on_msg(static_cast<const uint8_t*>(msg.data()), msg.size());
  }
}

}  // namespace amo_zmq
