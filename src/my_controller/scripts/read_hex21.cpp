#include <array>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr speed_t kBaudRate = B2000000;
constexpr size_t kFrameSize = 28;
constexpr int kDefaultPrintEvery = 50;

class SerialPort {
public:
  explicit SerialPort(const std::string & port) {
    fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open serial port '" + port + "'.");
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("Failed to read serial settings for '" + port + "'.");
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, kBaudRate);
    cfsetospeed(&tty, kBaudRate);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cc[VMIN] = static_cast<cc_t>(kFrameSize);
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("Failed to configure serial port '" + port + "'.");
    }
    tcflush(fd_, TCIFLUSH);
  }

  ~SerialPort() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool readFrame(std::array<unsigned char, kFrameSize> & frame) {
    size_t offset = 0;
    while (offset < frame.size()) {
      const ssize_t n = ::read(fd_, frame.data() + offset, frame.size() - offset);
      if (n <= 0) {
        return false;
      }
      offset += static_cast<size_t>(n);
    }
    return true;
  }

private:
  int fd_{-1};
};

std::array<float, 6> parseWrench(const std::array<unsigned char, kFrameSize> & frame) {
  std::array<float, 6> wrench{};
  for (size_t i = 0; i < wrench.size(); ++i) {
    std::memcpy(&wrench[i], frame.data() + i * sizeof(float), sizeof(float));
  }
  return wrench;
}

}  // namespace

int main(int argc, char ** argv) {
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
  const int print_every = argc > 2 ? std::stoi(argv[2]) : kDefaultPrintEvery;

  try {
    SerialPort serial(port);
    std::array<unsigned char, kFrameSize> frame{};
    int count = 0;

    std::cout << "Reading HEX21 from " << port << " at 2 Mbaud.\n"
              << "Printing every " << print_every << " frames. Press Ctrl+C to stop.\n";
    std::cout << std::fixed << std::setprecision(4);

    while (true) {
      if (!serial.readFrame(frame)) {
        std::cerr << "Serial read failed." << std::endl;
        return 1;
      }

      const auto wrench = parseWrench(frame);
      ++count;
      if (count % print_every == 0) {
        std::cout << "F[N]=[" << wrench[0] << ", " << wrench[1] << ", " << wrench[2]
                  << "] T[Nm]=[" << wrench[3] << ", " << wrench[4] << ", " << wrench[5]
                  << "]" << std::endl;
      }
    }
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
}
