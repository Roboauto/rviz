#pragma once

#include <list>

#include <ros/time.h>

//TODO: locks!

template<class T>
class MsgSync {
public:
    MsgSync(std::size_t size) : buffer_size_(size) {

    }

    void setSize(std::size_t size) {
        buffer_size_ = size;
    }

    T get_nearest(const ros::Time &time) const;

    T getLast() const {
        return buffer_.back();
    }

    void add(const T &data) {
        while (buffer_.size() >= buffer_size_) {
            buffer_.pop_front();
        }
        buffer_.push_back(data);
        // todo: check size + add
    }

    bool empty() const {
        return buffer_.empty();
    }

private:
    std::size_t buffer_size_;
    std::list<T> buffer_;
};

template<class T>
T MsgSync<T>::get_nearest(const ros::Time &time) const {
    auto iter = std::find_if(buffer_.begin(), buffer_.end(), [time](const T &x) { return x->header.stamp >= time; });

    if (iter == buffer_.end()) {
        return buffer_.back();
    } else {
        return *iter;
    }
}

