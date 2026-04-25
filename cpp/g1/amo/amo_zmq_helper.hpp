#pragma once

// Thin wrappers over cppzmq for the AMO bridge use case.
// Centralizes socket-option choices (HWM=1, CONFLATE on SUB, etc.) so the
// bridge can focus on packing/unpacking and joint mapping.

#include <atomic>
#include <functional>
#include <string>

#include <zmq.hpp>

namespace amo_zmq {

// PUB socket bound to `endpoint`. SNDHWM=1 so the socket drops the oldest
// frame when a slow peer can't keep up — control-loop friendly.
zmq::socket_t make_pub_bound(zmq::context_t& ctx, const std::string& endpoint);

// SUB socket connected to `endpoint`, subscribed to all topics, with
// CONFLATE=1 + RCVHWM=1 so we always read the newest frame and never queue.
zmq::socket_t make_sub_conflate(zmq::context_t& ctx, const std::string& endpoint);

// Non-blocking send. Drops the frame if the queue is full rather than block.
void send_dontwait(zmq::socket_t& sock, const void* data, size_t size);

// Run a poll/receive loop on `sock` until `running` goes false.
// `on_msg(data, size)` is invoked for every successfully received frame.
// `on_fatal(reason)` is invoked if any ZMQ error is hit while the loop is
// still meant to be running; the loop exits after invoking it. Errors that
// occur after `running` has gone false (shutdown path) are silent.
//
// Note: this fail-fast policy is intentional. Silently retrying on a ZMQ
// error means AMO stops updating leg targets, which is a control-loop
// hazard. Prefer to shut everything down on any anomaly. If a specific
// error class shows up frequently in practice and is genuinely benign, that
// is when we discuss adding a carve-out.
void run_receive_loop(zmq::socket_t& sock,
                      const std::atomic<bool>& running,
                      const std::function<void(const uint8_t*, size_t)>& on_msg,
                      const std::function<void(const std::string&)>& on_fatal);

}  // namespace amo_zmq
