#pragma once

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

inline std::string generate_recording_label() {
    std::time_t t = std::time(nullptr);
    std::tm* tm = std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(tm, "%B_%d_%H:%M");
    std::string s = oss.str();
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}
