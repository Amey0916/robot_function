#pragma once
#include <tm_driver/common/common_header.h>
namespace timoo
{
namespace lidar
{
constexpr uint16_t MAX_THREAD_NUM = 4;
struct Thread
{
  Thread() : start_(false)
  {
  }
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> start_;
};
class ThreadPool
{
public:
  typedef std::shared_ptr<ThreadPool> Ptr;
  inline ThreadPool() : stop_flag_(false), idl_thr_num_(MAX_THREAD_NUM)
  {
    for (int i = 0; i < idl_thr_num_; ++i)
    {
      pool_.emplace_back([this] {
        while (!this->stop_flag_)
        {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock{ this->mutex_ };
            this->cv_task_.wait(lock, [this] { return this->stop_flag_.load() || !this->tasks_.empty(); });
            if (this->stop_flag_ && this->tasks_.empty())
              return;
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }
          idl_thr_num_--;
          task();
          idl_thr_num_++;
        }
      });
    }
  }

  inline ~ThreadPool()
  {
    stop_flag_.store(true);
    cv_task_.notify_all();
    for (std::thread& thread : pool_)
    {
      thread.join();
    }
  }

public:
  template <class F, class... Args>
  inline void commit(F&& f, Args&&... args)
  {
    if (stop_flag_.load())
      throw std::runtime_error("Commit on LiDAR threadpool is stopped.");
    using RetType = decltype(f(args...));
    auto task =
        std::make_shared<std::packaged_task<RetType()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    {
      std::lock_guard<std::mutex> lock{ mutex_ };
      tasks_.emplace([task]() { (*task)(); });
    }
    cv_task_.notify_one();
  }

private:
  using Task = std::function<void()>;
  std::vector<std::thread> pool_;
  std::queue<Task> tasks_;
  std::mutex mutex_;
  std::condition_variable cv_task_;
  std::atomic<bool> stop_flag_;
  std::atomic<int> idl_thr_num_;
};
}  // namespace lidar
}  // namespace timoo