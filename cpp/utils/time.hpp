#pragma once

#include <chrono>
#include <cstdint>

class Time {
public:
    static uint64_t ts() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - origin()
        ).count();
    }

private:
    static const std::chrono::steady_clock::time_point& origin() {
        static const auto t = std::chrono::steady_clock::now();
        return t;
    }
};
