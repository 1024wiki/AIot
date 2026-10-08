/**
 * @file spsc_queue.h
 * @brief 单生产者单消费者无锁环形队列
 *        UART_RX 线程 push，算法 Worker 线程 pop；
 *        读侧和写侧各自只动自己的 head/tail，无需 mutex。
 */
#pragma once
#include <array>
#include <atomic>
#include <cstddef>

namespace drv {

template <typename T, size_t N>
class SpscQueue {
public:
    SpscQueue() : head_(0), tail_(0) {}

    /// 生产者 push；满了返回 false（丢弃，上层可计数丢包）
    bool push(const T &v) {
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t next = (t + 1) % N;
        if (next == head_.load(std::memory_order_acquire)) return false;
        buf_[t] = v;
        tail_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T &out) {
        size_t h = head_.load(std::memory_order_relaxed);
        if (h == tail_.load(std::memory_order_acquire)) return false;
        out = buf_[h];
        head_.store((h + 1) % N, std::memory_order_release);
        return true;
    }

    bool empty() const { return head_.load() == tail_.load(); }
    size_t size() const {
        size_t h = head_.load(), t = tail_.load();
        return t >= h ? t - h : N - h + t;
    }

private:
    std::array<T, N> buf_;
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

} // namespace drv
