
#ifndef _TASK_THREAD_HPP
#define _TASK_THREAD_HPP

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

namespace task_thread
{
  // Items are moved into the queue. In other words, The queue takes
  // ownership of the items placed in it. TItem must be movable but
  // it does not need a default constructor.
  //
  // Sample code:
  //  task_thread::ConcurrentQueue<std::unique_ptr<int>> cqi{};
  //  cqi.push(std::make_unique<int>(5));
  //  auto i6 = std::make_unique<int>(6);
  //  cqi.push(std::move(i6));
  //  std::unique_ptr<int> pi5;
  //  cqi.try_pop(pi5);
  //  std::cout << *pi5 << " " << *cqi.pop() << std::endl;
  //
  template<typename TItem>
  class ConcurrentQueue
  {
    std::queue<TItem> q_{};
    std::mutex m_{};
    std::condition_variable cv_{};

    // This class is needed in order to return
    // an object with no default constructor from
    // the pop() method.
    class Popper
    {
      std::queue<TItem> &q_;

    public:
      Popper(std::queue<TItem> &q) : q_{q}
      {}

      ~Popper()
      { q_.pop(); } // Do the pop after the item has been moved from the queue.

      TItem pop()
      { return std::move(q_.front()); }
    };

  public:
    void push(TItem item)
    {
      std::unique_lock<std::mutex> lock{m_};
      q_.push(std::move(item));
      lock.unlock(); // Unlock so the notify-ee can access the resource immediately
      cv_.notify_one();
    }

    TItem pop()
    {
      std::unique_lock<std::mutex> lock{m_};
      while (q_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(250));
      }
      return Popper(q_).pop();
    }

    bool try_pop(TItem &popped_item)
    {
      std::unique_lock<std::mutex> lock{m_};
      if (q_.empty()) {
        return false;
      }

      popped_item = std::move(Popper(q_).pop());
      return true;
    }

    bool pop_or_abort(TItem &popped_item, volatile int &abort)
    {
      std::unique_lock<std::mutex> lock{m_};
      while (!abort && q_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(250));
      }
      if (abort) {
        return false;
      }

      popped_item = std::move(Popper(q_).pop());
      return true;
    }

    void notify_one()
    {
      cv_.notify_one();
    }

    bool empty()
    {
      std::unique_lock<std::mutex> lock{m_};
      return q_.empty();
    }

    std::size_t size()
    {
      std::unique_lock<std::mutex> lock{m_};
      return q_.size();
    }
  };

  // TaskThread is instantiated with a work object. The functors in the queue
  // contain code that operates on the work object. The functors are executed
  // on a thread. A convenient way to return results from a functor calculation
  // is with another ConcurrentQueue. Or promise/future can be used to return
  // results. NOTE: std::packaged_task objects are used to hold the callable
  // object instead of std::function objects. This is because std::function
  // is not movable. The future functionality of std::packaged_task is not used.
  //
  // Sample code:
  //  task_thread::ConcurrentQueue<int> out_q{}; // The output queue must have a longer life than the TaskThread
  //  auto work = std::make_unique<int>(5);
  //  task_thread::TaskThread<int> tti{std::move(work)};
  //  tti.push([&out_q](int &i)
  //           {
  //             i += 2;
  //             out_q.push(i);
  //           });
  //  tti.push([&out_q](int &i)
  //           {
  //             i += 3;
  //             out_q.push(i);
  //           });
  //  std::cout << out_q.pop() << " " << out_q.pop() << std::endl;
  //
  template<class TWork>
  class TaskThread
  {
    ConcurrentQueue<std::packaged_task<void(TWork &)>> q_{};
    std::unique_ptr<TWork> work_;
    std::thread thread_;
    bool run_on_start_thread_; // for debugging - execute task on the starting thread.
    volatile int abort_{0};

    static void run(TaskThread *tt)
    {
      // Occasionally when running under the debugger, this thread will hang
      // and not get started. The following line seems to fix the problem.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      std::packaged_task<void(TWork &)> task{};
      while (tt->q_.pop_or_abort(task, tt->abort_)) {
        task(*tt->work_);
      }
    }

  public:
    TaskThread(std::unique_ptr<TWork> work, bool run_on_start_thread = false) :
      work_{std::move(work)}, thread_{run, this}, run_on_start_thread_{run_on_start_thread}
    {}

    ~TaskThread()
    {
      abort_ = 1;
      q_.notify_one();
      thread_.join();
    }

    void abort()
    {
      abort_ = 1;
      q_.notify_one();
    }

    template <class TTask>
    void push(TTask task)
    {
      if (run_on_start_thread_) {
        task(*work_);
      } else {
        q_.push(std::packaged_task<void(TWork &)>{std::move(task)});
      }
    }

    bool empty()
    {
      return q_.empty();
    }

    std::size_t tasks_in_queue()
    {
      return q_.size();
    }

    void wait_until_empty()
    {
      while (!empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  };
}
#endif //_TASK_THREAD_HPP
