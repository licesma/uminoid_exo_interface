#include "serial_value_reader.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

bool ConfigureSerial9600(int fd) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }

  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // 1 s timeout (tenths of second)

  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

}  // namespace

SerialValueReader::SerialValueReader(const std::string& device_path,
                                     double default_value)
    : device_path_(device_path), last_value_(default_value) {
  fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    return;
  }
  if (!ConfigureSerial9600(fd_)) {
    close(fd_);
    fd_ = -1;
    return;
  }
  ok_ = true;
  thread_ = std::thread(&SerialValueReader::ReaderLoop, this);
}

SerialValueReader::~SerialValueReader() {
  running_.store(false);
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SerialValueReader::ReaderLoop() {
  std::string line;
  line.reserve(64);
  char buf[256];

  while (running_.load() && fd_ >= 0) {
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) continue;

    for (ssize_t i = 0; i < n && running_.load(); ++i) {
      char c = buf[i];
      if (c == '\n' || c == '\r') {
        if (!line.empty()) {
          char* end = nullptr;
          double v = std::strtod(line.c_str(), &end);
          if (end != line.c_str()) {
            last_value_.store(v);
          }
          line.clear();
        }
      } else {
        if (line.size() < line.capacity()) {
          line += c;
        }
      }
    }
  }
}
