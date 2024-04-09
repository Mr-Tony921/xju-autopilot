/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace xju {
namespace pnc {

// Thread safe implementation of a Queue using a std::queue
template <typename T>
class SafeQueue {
 private:
  std::queue<T> m_queue;  
  std::mutex m_mutex;     
 public:
  SafeQueue() {}
  SafeQueue(SafeQueue &&other) {}
  ~SafeQueue() {}
  bool empty()  
  {
    std::unique_lock<std::mutex> lock(
        m_mutex);  
    return m_queue.empty();
  }
  int size() {
    std::unique_lock<std::mutex> lock(
        m_mutex);  
    return m_queue.size();
  }

  void enqueue(T &t) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.emplace(t);
  }

  bool dequeue(T &t) {
    std::unique_lock<std::mutex> lock(m_mutex);  
    if (m_queue.empty()) return false;
    t = std::move(
        m_queue.front());  
    m_queue.pop();  
    return true;
  }
};

class ThreadPool {
 private:
  class ThreadWorker  
  {
   private:
    int m_id;            
    ThreadPool *m_pool;  
   public:
    
    ThreadWorker(ThreadPool *pool, const int id) : m_pool(pool), m_id(id) {}

    void operator()() {
      std::function<void()> func;  
      bool dequeued;               
      while (!m_pool->m_shutdown) {
        {
          std::unique_lock<std::mutex> lock(m_pool->m_conditional_mutex);
          if (m_pool->m_queue.empty()) {
            m_pool->m_conditional_lock.wait(
                lock);  
          }

          dequeued = m_pool->m_queue.dequeue(func);
        }

        if (dequeued) func();
      }
    }
  };
  bool m_shutdown;                           
  SafeQueue<std::function<void()>> m_queue;  
  std::vector<std::thread> m_threads;  
  std::mutex m_conditional_mutex;      
  std::condition_variable
      m_conditional_lock;  
  // Inits thread pool
  void init() {
    for (int i = 0; i < m_threads.size(); ++i) {
      m_threads.at(i) = std::thread(ThreadWorker(this, i));  
    }
  }

  // Waits until threads finish their current task and shutdowns the pool
  void shutdown() {
    m_shutdown = true;
    m_conditional_lock.notify_all();  
    for (int i = 0; i < m_threads.size(); ++i) {
      if (m_threads.at(i).joinable())  
      {
        m_threads.at(i).join();  
      }
    }
  }

 public:
  ThreadPool(const int n_threads = 6)
      : m_threads(std::vector<std::thread>(n_threads)), m_shutdown(false) {
    init();
    std::cout << "ThreadPool Constructor" << std::endl;
  }

  static std::shared_ptr<ThreadPool> Instance() {
    static auto instance = std::make_shared<ThreadPool>();
    return instance;
  }

  // DECLARE_SINGLETON(ThreadPool);

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool(ThreadPool &&) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;
  ThreadPool &operator=(ThreadPool &&) = delete;
  ~ThreadPool() {
    shutdown();
    std::cout << "ThreadPool Desconstrution" << std::endl;
  }

  // Submit a function to be executed asynchronously by the pool
  template <typename F, typename... Args>
  auto submit(F &&f, Args &&...args) -> std::future<decltype(f(args...))> {
    // Create a function with bounded parameter ready to execute
    std::function<decltype(f(args...))()> func = std::bind(
        std::forward<F>(f),
        std::forward<Args>(
            args)...);  

    // Encapsulate it into a shared pointer in order to be able to copy
    // construct
    auto task_ptr =
        std::make_shared<std::packaged_task<decltype(f(args...))()>>(func);

    // Warp packaged task into void function
    std::function<void()> warpper_func = [task_ptr]() { (*task_ptr)(); };

    m_queue.enqueue(warpper_func);

    m_conditional_lock.notify_one();

    return task_ptr->get_future();
  }
};

}  // namespace pnc
}  // namespace xju
