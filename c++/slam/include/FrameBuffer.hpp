#pragma once
#include <condition_variable>
#include <mutex>
#include <deque>
//Thread safe, FIFO, for frame buffer

template <typename T>
class FrameBuffer
{
    private:
        unsigned int m_size;
        std::mutex m_mutex;
        std::condition_variable m_cv;
        std::deque<T> m_buffer;
    public:
        FrameBuffer(unsigned int sz): m_size(sz) {}
        void push(T data) {
            std::unique_lock<std::mutex> u_lock(m_mutex);
            m_cv.wait(u_lock, [this]() { return m_buffer.size() < m_size; });
            m_buffer.push_back(data);
            u_lock.unlock();
            m_cv.notify_all();
        }
        T pop() {
            std::unique_lock<std::mutex> u_lock(m_mutex);
            m_cv.wait(u_lock, [this]() { return m_buffer.size() > 0; });
            T t_data = m_buffer.front();
            m_buffer.pop_front();
            u_lock.unlock();
            m_cv.notify_all();
            return t_data;
        }
};