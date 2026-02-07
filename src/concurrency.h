#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

class TaskManager {
public:
  TaskManager(int numThreads, int maxTasks);

  template <typename Callable, typename... Args>
  bool add(Callable &&func, Args &&...args) {
    {
      std::lock_guard<std::mutex> lock(tasksMutex_);
      if (!isRunning_ || tasks_.size() >= maxTasks_) {
        return false;
      }

      tasks_.emplace_back(
          [func = std::forward<Callable>(func),
           ... args = std::forward<Args>(args)]() mutable { func(args...); });
    }

    condition_.notify_one();
    return true;
  }

  void stop();

private:
  std::mutex tasksMutex_;
  std::deque<std::function<void()>> tasks_;
  int maxTasks_;

  std::vector<std::thread> threads_;
  std::condition_variable condition_;
  bool isRunning_;
};
