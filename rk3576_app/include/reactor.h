/**
 * @file reactor.h
 * @brief epoll Reactor 事件循环：fd -> handler 回调
 *        单线程负责 UART 收发 + 定时器节拍 + socket 事件调度。
 */
#pragma once
#include <functional>
#include <unordered_map>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <cstdio>

namespace drv {

class Reactor {
public:
    using Handler = std::function<void(int fd, uint32_t events)>;

    Reactor() : epfd_(-1), running_(false) {}
    ~Reactor() { if (epfd_ >= 0) ::close(epfd_); }

    bool init(int max_events = 64) {
        epfd_ = ::epoll_create1(0);
        if (epfd_ < 0) { perror("epoll_create1"); return false; }
        max_events_ = max_events;
        return true;
    }

    /// 注册 fd 与回调；ptr 直接存 handler 索引
    bool add(int fd, uint32_t events, Handler cb) {
        handlers_[fd] = std::move(cb);
        struct epoll_event ev{};
        ev.events = events;
        ev.data.fd = fd;
        if (::epoll_ctl(epfd_, EPOLL_CTL_ADD, fd, &ev) < 0) { perror("epoll_ctl ADD"); return false; }
        return true;
    }
    bool mod(int fd, uint32_t events) {
        struct epoll_event ev{};
        ev.events = events; ev.data.fd = fd;
        return ::epoll_ctl(epfd_, EPOLL_CTL_MOD, fd, &ev) == 0;
    }
    void remove(int fd) {
        ::epoll_ctl(epfd_, EPOLL_CTL_DEL, fd, nullptr);
        handlers_.erase(fd);
    }

    /// 周期定时器：创建 timerfd 并注册，返回fd（调用方不close，可remove）
    int add_timer_task(int interval_ms, Handler cb, bool oneshot = false) {
        int tfd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
        if (tfd < 0) { perror("timerfd_create"); return -1; }
        struct itimerspec its{};
        its.it_interval.tv_sec  = oneshot ? 0 : interval_ms / 1000;
        its.it_interval.tv_nsec = oneshot ? 0 : (interval_ms % 1000) * 1000000LL;
        its.it_value.tv_sec  = interval_ms / 1000;
        its.it_value.tv_nsec = (interval_ms % 1000) * 1000000LL;
        ::timerfd_settime(tfd, 0, &its, nullptr);
        add(tfd, EPOLLIN, std::move(cb));
        return tfd;
    }

    void run() {
        running_ = true;
        std::vector<struct epoll_event> evts(max_events_);
        while (running_) {
            int n = ::epoll_wait(epfd_, evts.data(), max_events_, -1);
            if (n < 0) { if (errno == EINTR) continue; perror("epoll_wait"); break; }
            for (int i = 0; i < n; ++i) {
                int fd = evts[i].data.fd;
                auto it = handlers_.find(fd);
                if (it != handlers_.end()) it->second(fd, evts[i].events);
            }
        }
    }

    void stop() { running_ = false; }

private:
    int epfd_;
    int max_events_ = 64;
    bool running_;
    std::unordered_map<int, Handler> handlers_;
};

} // namespace drv
