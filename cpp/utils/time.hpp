#pragma once

#include <chrono>
#include <cstdint>

class Time {
public:
    static void init() {
        origin() = std::chrono::steady_clock::now();
    }

    static int64_t ts() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - origin()
        ).count();
    }

private:
    static std::chrono::steady_clock::time_point& origin() {
        static std::chrono::steady_clock::time_point t;
        return t;
    }
};
