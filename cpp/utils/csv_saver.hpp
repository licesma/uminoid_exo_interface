// Thread-safe CSV writer with asynchronous I/O.
//
// A background thread drains queued lines in batches, keeping the caller's
// hot path free of file-system latency. Periodic flushing.
//
// Move-only; default-constructed instances are inactive

#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

class CsvSaver {
public:
    CsvSaver() = default;

    explicit CsvSaver(const std::string& path, const std::string& header,
                      uint64_t flush_every = 1000)
        : state_(std::make_shared<State>()) {
        state_->file.open(path);
        if (!state_->file)
            throw std::runtime_error("Failed to open CSV: " + path);
        state_->file << header << "\n";
        state_->flush_every = flush_every;
        thread_ = std::thread(writer_loop, state_);
    }

    ~CsvSaver() { close(); }

    CsvSaver(CsvSaver&& o) noexcept
        : state_(std::move(o.state_)), thread_(std::move(o.thread_)) {}

    CsvSaver& operator=(CsvSaver&& o) noexcept {
        if (this != &o) {
            close();
            state_ = std::move(o.state_);
            thread_ = std::move(o.thread_);
        }
        return *this;
    }

    void write_line(std::string line) {
        if (!state_) return;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->queue.push_back(std::move(line));
        state_->cv.notify_one();
    }

    void close() {
        if (!state_) return;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->done = true;
        }
        state_->cv.notify_one();
        if (thread_.joinable()) thread_.join();
        state_.reset();
    }

    explicit operator bool() const { return state_ != nullptr; }

private:
    struct State {
        std::ofstream file;
        std::deque<std::string> queue;
        std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        uint64_t count = 0;
        uint64_t flush_every = 1000;
    };

    static void writer_loop(std::shared_ptr<State> s) {
        while (true) {
            std::deque<std::string> batch;
            {
                std::unique_lock<std::mutex> lock(s->mutex);
                s->cv.wait(lock, [&] { return s->done || !s->queue.empty(); });
                batch.swap(s->queue);
                if (s->done && batch.empty()) break;
            }
            for (const auto& line : batch) {
                s->file << line << "\n";
                if (++s->count % s->flush_every == 0)
                    s->file.flush();
            }
        }
        s->file.flush();
        s->file.close();
    }

    std::shared_ptr<State> state_;
    std::thread thread_;
};
