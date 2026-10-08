/**
 * @file uart.h  POSIX termios 串口封装
 */
#pragma once
#include <string>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

namespace drv {

class Uart {
public:
    Uart() : fd_(-1) {}
    ~Uart() { close_dev(); }

    bool open_dev(const char *dev, int baud) {
        close_dev();
        fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { perror("open uart"); return false; }
        if (!config(baud)) { close_dev(); return false; }
        return true;
    }

    void close_dev() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    int  fd() const { return fd_; }

    ssize_t read_bytes(uint8_t *buf, size_t n) {
        return ::read(fd_, buf, n);
    }
    ssize_t write_bytes(const uint8_t *buf, size_t n) {
        return ::write(fd_, buf, n);
    }
    /// 阻塞式完整写入（命令帧很小，可直接 write 循环）
    bool write_all(const uint8_t *buf, size_t n) {
        size_t off = 0;
        while (off < n) {
            ssize_t w = ::write(fd_, buf + off, n - off);
            if (w < 0) { if (errno == EAGAIN) continue; return false; }
            off += (size_t)w;
        }
        return true;
    }
    /// 便捷写字符串（AT 指令等）
    ssize_t write_str(const std::string &s) { return write_bytes((const uint8_t*)s.data(), s.size()); }

private:
    bool config(int baud) {
        struct termios tcfg{};
        if (::tcgetattr(fd_, &tcfg) < 0) { perror("tcgetattr"); return false; }
        ::cfmakeraw(&tcfg);
        speed_t spd;
        switch (baud) {
            case 9600:   spd = B9600;   break;
            case 19200:  spd = B19200;  break;
            case 38400:  spd = B38400;  break;
            case 57600:  spd = B57600;  break;
            case 115200: spd = B115200; break;
            case 230400: spd = B230400; break;
            case 460800: spd = B460800; break;
            case 921600: spd = B921600; break;
            default:     spd = B115200; break;
        }
        ::cfsetispeed(&tcfg, spd);
        ::cfsetospeed(&tcfg, spd);
        tcfg.c_cflag |= (CLOCAL | CREAD);
        tcfg.c_cflag &= ~CRTSCTS;     // 无硬件流控
        tcfg.c_cc[VMIN]  = 0;
        tcfg.c_cc[VTIME] = 0;         // 纯 epoll 调度
        if (::tcsetattr(fd_, TCSANOW, &tcfg) < 0) { perror("tcsetattr"); return false; }
        ::tcflush(fd_, TCIOFLUSH);
        return true;
    }

    int fd_;
};

} // namespace drv
