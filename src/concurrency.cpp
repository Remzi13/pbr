#include "concurrency.h"

TaskManager::TaskManager(int numThreads, int maxTasks) : maxTasks_(maxTasks)
{
    isRunning_ = true;
    for (int i = 0; i < numThreads; ++i)
    {
        threads_.push_back(std::thread([this]() {
            while (true)
            {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(tasksMutex_);
                    condition_.wait(lock, [this] {
                        return !tasks_.empty() || !isRunning_;
                        });

                    if (!isRunning_ && tasks_.empty())
                    {
                        return; 
                    }
                    if (!tasks_.empty())
                    {
                        task = std::move(tasks_.front());
                        tasks_.pop_front();
                    }
                }
                if (task)
                {
                    task();
                }
            }
        }));
    }
}

void TaskManager::stop()
{    
    {
        std::lock_guard<std::mutex> lock(tasksMutex_);
        if (!isRunning_) return;
        isRunning_ = false;
    }

    condition_.notify_all();

    for (auto& thread : threads_)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
}