#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

// ─── Data ────────────────────────────────────────────────────────────────────

enum class CollectionStatus { InProgress, Completed, Cancelled, Next };

struct CollectionRecord {
    int                                   id;
    CollectionStatus                      status;
    std::chrono::steady_clock::time_point start_time;
    int                                   final_seconds = -1;
};

// ─── UI ──────────────────────────────────────────────────────────────────────

namespace ui {
    inline std::mutex                    mtx;
    inline std::vector<CollectionRecord> records;
    inline int                           drawn_lines = 0;

    inline void redraw() {
        std::lock_guard<std::mutex> lock(mtx);

        if (drawn_lines > 0)
            std::cout << "\033[" << drawn_lines << "A";

        auto now   = std::chrono::steady_clock::now();
        int  lines = 0;

        for (const auto& r : records) {
            std::string status_str;
            std::string dur_str;

            if (r.status == CollectionStatus::InProgress) {
                int s      = std::chrono::duration_cast<std::chrono::seconds>(now - r.start_time).count();
                status_str = "In Progress";
                dur_str    = std::to_string(s) + "s";
            } else if (r.status == CollectionStatus::Completed) {
                status_str = "Completed";
                dur_str    = std::to_string(r.final_seconds) + "s";
            } else if (r.status == CollectionStatus::Cancelled) {
                status_str = "Cancelled";
                dur_str    = std::to_string(r.final_seconds) + "s";
            } else {
                status_str = "Next";
                dur_str    = "-";
            }

            std::cout << "\r\033[K"
                      << std::left
                      << "  Collection " << std::setw(6) << r.id
                      << std::setw(18) << status_str
                      << dur_str << "\n";
            ++lines;
        }

        std::cout.flush();
        drawn_lines = lines;
    }

    inline void add_collection(int id) {
        std::lock_guard<std::mutex> lock(mtx);
        records.push_back({id, CollectionStatus::InProgress, std::chrono::steady_clock::now()});
    }

    inline void add_next(int id) {
        std::lock_guard<std::mutex> lock(mtx);
        records.push_back({id, CollectionStatus::Next, std::chrono::steady_clock::now()});
    }

    // Transitions the queued Next collection to InProgress and resets its timer.
    // Returns false if there is no Next collection.
    inline bool start_next_collection() {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto& r : records) {
            if (r.status == CollectionStatus::Next) {
                r.status     = CollectionStatus::InProgress;
                r.start_time = std::chrono::steady_clock::now();
                return true;
            }
        }
        return false;
    }

    inline bool has_next() {
        std::lock_guard<std::mutex> lock(mtx);
        for (const auto& r : records)
            if (r.status == CollectionStatus::Next) return true;
        return false;
    }

    inline void complete_current(int id) {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto& r : records) {
            if (r.id == id && r.status == CollectionStatus::InProgress) {
                int s           = std::chrono::duration_cast<std::chrono::seconds>(
                                      std::chrono::steady_clock::now() - r.start_time).count();
                r.final_seconds = s;
                r.status        = CollectionStatus::Completed;
                return;
            }
        }
    }

    inline void cancel_current(int id) {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto& r : records) {
            if (r.id == id && r.status == CollectionStatus::InProgress) {
                int s           = std::chrono::duration_cast<std::chrono::seconds>(
                                      std::chrono::steady_clock::now() - r.start_time).count();
                r.final_seconds = s;
                r.status        = CollectionStatus::Cancelled;
                return;
            }
        }
    }
} // namespace ui

// ─── Terminal raw mode ────────────────────────────────────────────────────────

struct RawMode {
    struct termios orig;
    RawMode() {
        tcgetattr(STDIN_FILENO, &orig);
        struct termios raw = orig;
        raw.c_lflag &= ~(ECHO | ICANON | ISIG);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 2;  // 200 ms timeout
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    }
    ~RawMode() { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig); }
};

