#include "inspire_port_resolver.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <unordered_map>
#include <vector>

namespace inspire_port_resolver {
namespace {

// HAND_ID register lives at 0x03E8. Frame layout follows the Inspire RH56
// proprietary protocol (see user manual §2.2 / inspire.h).
constexpr uint8_t  HAND_ID_ADDR_L    = 0xE8;
constexpr uint8_t  HAND_ID_ADDR_H    = 0x03;
constexpr speed_t  BAUDRATE          = B115200;
constexpr int      PROBE_TIMEOUT_MS  = 50;

// USB vendor ID of the CH340 (WCH) USB-serial chip used by the Inspire hands.
// Filtering on this prevents probing (and clobbering) FTDI-based adapters.
constexpr const char* INSPIRE_USB_VENDOR = "1a86";

uint8_t checksum(const uint8_t* data, size_t len) {
    // Low byte of sum over data[2 .. len-2] (skip 0xEB 0x90 header and cksum slot).
    uint8_t sum = 0;
    for (size_t i = 2; i + 1 < len; ++i) sum += data[i];
    return sum;
}

struct ScopedFd {
    int fd;
    explicit ScopedFd(int f) : fd(f) {}
    ~ScopedFd() { if (fd >= 0) ::close(fd); }
    ScopedFd(const ScopedFd&) = delete;
    ScopedFd& operator=(const ScopedFd&) = delete;
};

int open_port(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    termios opt{};
    tcgetattr(fd, &opt);
    opt.c_oflag = 0;
    opt.c_lflag = 0;
    opt.c_iflag = 0;
    cfsetispeed(&opt, BAUDRATE);
    cfsetospeed(&opt, BAUDRATE);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN]  = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &opt);
    return fd;
}

// Send a read-HAND_ID request targeting `expected_id`. Only the hand with
// matching ID will reply; other hands stay silent. On success returns the
// hand ID echoed by the response (byte[2]).
std::optional<uint8_t> probe_hand_id(int fd, uint8_t expected_id) {
    uint8_t req[9] = {
        0xEB, 0x90, expected_id,
        0x04, 0x11, HAND_ID_ADDR_L, HAND_ID_ADDR_H, 0x01, 0x00
    };
    req[8] = checksum(req, sizeof(req));

    tcflush(fd, TCIFLUSH);
    if (::write(fd, req, sizeof(req)) != (ssize_t)sizeof(req)) return std::nullopt;

    uint8_t resp[9] = {0};
    size_t total = 0;
    while (total < sizeof(resp)) {
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd, &rset);
        timeval tv{PROBE_TIMEOUT_MS / 1000, (PROBE_TIMEOUT_MS % 1000) * 1000};
        int rv = ::select(fd + 1, &rset, nullptr, nullptr, &tv);
        if (rv <= 0) break;
        ssize_t n = ::read(fd, resp + total, sizeof(resp) - total);
        if (n <= 0) break;
        total += static_cast<size_t>(n);
    }
    if (total < sizeof(resp))               return std::nullopt;
    if (resp[0] != 0x90 || resp[1] != 0xEB) return std::nullopt;
    return resp[2];
}

// Read the USB vendor ID of the device backing /dev/ttyUSBx via sysfs.
std::string read_usb_vendor(const std::string& tty_name) {
    std::ifstream f("/sys/class/tty/" + tty_name + "/device/../../idVendor");
    std::string vendor;
    f >> vendor;
    return vendor;
}

std::vector<std::string> list_inspire_ports() {
    std::vector<std::string> ports;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator("/dev", ec)) {
        const auto name = entry.path().filename().string();
        if (name.rfind("ttyUSB", 0) != 0)                   continue;
        if (read_usb_vendor(name) != INSPIRE_USB_VENDOR)    continue;
        ports.push_back(entry.path().string());
    }
    std::sort(ports.begin(), ports.end());
    return ports;
}

}  // namespace

Assignment resolve(
    std::optional<uint8_t> left_id,
    std::optional<uint8_t> right_id,
    const std::function<void(const std::string&)>& raise_error
) {
    auto fail = [&](const std::string& msg) -> Assignment {
        if (raise_error) raise_error("[inspire_port_resolver] " + msg);
        return {};
    };

    if (!left_id && !right_id) return {};

    if (left_id && right_id && *left_id == *right_id) {
        return fail("left_id and right_id must differ");
    }

    std::vector<uint8_t> wanted_ids;
    if (left_id)  wanted_ids.push_back(*left_id);
    if (right_id) wanted_ids.push_back(*right_id);

    std::unordered_map<uint8_t, std::string> port_for_id;

    for (const auto& port : list_inspire_ports()) {
        ScopedFd fd(open_port(port));
        if (fd.fd < 0) continue;

        std::optional<uint8_t> id;
        for (uint8_t try_id : wanted_ids) {
            if ((id = probe_hand_id(fd.fd, try_id))) break;
        }
        if (!id) continue;

        auto [it, inserted] = port_for_id.emplace(*id, port);
        if (!inserted) {
            return fail("HAND_ID=" + std::to_string(*id) +
                        " responded on both " + it->second + " and " + port +
                        " — reassign unique IDs with set_inspire_id.py");
        }
    }

    auto find = [&](uint8_t id) -> std::string {
        auto it = port_for_id.find(id);
        if (it == port_for_id.end()) return {};
        return it->second;
    };

    std::string left  = left_id  ? find(*left_id)  : std::string{};
    std::string right = right_id ? find(*right_id) : std::string{};

    std::string missing;
    if (left_id  && left.empty())  missing += " left(HAND_ID=" + std::to_string(*left_id)  + ")";
    if (right_id && right.empty()) missing += " right(HAND_ID=" + std::to_string(*right_id) + ")";
    if (!missing.empty()) {
        return fail("no hand found on /dev/ttyUSB* for:" + missing);
    }

    return Assignment{std::move(left), std::move(right)};
}

}  // namespace inspire_port_resolver
