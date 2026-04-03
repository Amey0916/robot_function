#pragma once
#include <tm_driver/common/common_header.h>
namespace timoo
{
namespace lidar
{
template <typename T>
class Queue
{
public:
  inline Queue():is_task_finished_(true)
  {
  }

  inline T front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.front();
  }

  inline void push(const T& value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
  }

  inline void pop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty())
    {
      queue_.pop();
    }
  }

  inline T popFront()
  {
    T value;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty())
    {
      value = std::move(queue_.front());
      queue_.pop();
    }
    return value;
  }

  inline void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lock(mutex_);
    swap(empty, queue_);
  }

  inline size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

public:
  std::queue<T> queue_;
  std::atomic<bool> is_task_finished_;

private:
  mutable std::mutex mutex_;
};
}  // namespace lidar
}  // namespace timoo