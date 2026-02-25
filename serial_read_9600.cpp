#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>

static const char* const DEFAULT_SERIAL_DEVICE = "/dev/ttyACM0";

static bool configure_serial_9600(int fd) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "tcgetattr: " << std::strerror(errno) << std::endl;
    return false;
  }

  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  tty.c_cflag &= ~PARENB;         // no parity
  tty.c_cflag &= ~CSTOPB;          // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;              // 8 data bits
  tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;   // enable read, ignore modem lines

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw input
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // no software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // 1 s timeout (tenths of second)

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "tcsetattr: " << std::strerror(errno) << std::endl;
    return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  const char* device = (argc >= 2) ? argv[1] : DEFAULT_SERIAL_DEVICE;
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    std::cerr << "open " << device << ": " << std::strerror(errno) << std::endl;
    return 1;
  }

  if (!configure_serial_9600(fd)) {
    close(fd);
    return 1;
  }

  std::cout << "Reading from " << device << " at 9600 baud. Ctrl+C to stop.\n";

  std::string line;
  char buf[256];
  while (true) {
    ssize_t n = read(fd, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      std::cerr << "read: " << std::strerror(errno) << std::endl;
      break;
    }
    if (n == 0) continue;

    for (ssize_t i = 0; i < n; ++i) {
      char c = buf[i];
      if (c == '\n' || c == '\r') {
        if (!line.empty()) {
          std::cout << "received: \"" << line << "\"\n";
          line.clear();
        }
      } else {
        line += c;
      }
    }
  }

  close(fd);
  return 0;
}
