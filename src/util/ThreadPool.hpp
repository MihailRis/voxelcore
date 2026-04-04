#pragma once

#include <atomic>
#include <chrono>
#include <optional>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <queue>
#include <thread>
#include <utility>

#include "debug/Logger.hpp"
#include "delegates.hpp"
#include "interfaces/Task.hpp"

namespace util {

    template <class J, class T>
    struct ThreadPoolResult {
        J job;
        std::condition_variable& variable;
        int workerIndex;
        bool& locked;
        T entry;
    };

    template <class T, class R>
    class Worker {
    public:
        Worker() = default;
        virtual ~Worker() = default;
        virtual R operator()(const T&) = 0;
    };

    template <class T, class R>
    class ThreadPool : public Task {
        debug::Logger logger;
        std::queue<T> jobs;
        std::queue<ThreadPoolResult<T, R>> results;
        std::mutex resultsMutex;
        std::vector<std::thread> threads;
        std::condition_variable jobsMutexCondition;
        std::mutex jobsMutex;
        std::vector<std::unique_lock<std::mutex>> workersBlocked;
        consumer<R&&> resultConsumer;
        consumer<T&> onJobFailed = nullptr;
        runnable onComplete = nullptr;
        std::atomic<int> busyWorkers = 0;
        std::atomic<uint> jobsDone = 0;
        std::atomic<bool> working = true;
        supplier<std::optional<T>> jobsSource = nullptr;
        bool failed = false;
        bool standaloneResults = true;
        bool stopOnFail = true;

        void threadLoop(int index, std::unique_ptr<Worker<T, R>> worker) {
            std::condition_variable variable;
            std::mutex mutex;
            bool locked = false;
            while (working) {
                T job;
                {
                    std::unique_lock<std::mutex> lock(jobsMutex);
                    jobsMutexCondition.wait(lock, [this] {
                        return !jobs.empty() || !working;
                    });
                    if (!working || failed) {
                        break;
                    }
                    job = std::move(jobs.front());
                    jobs.pop();

                    busyWorkers++;
                }
                try {
                    R result = (*worker)(job);
                    {
                        std::lock_guard<std::mutex> lock(resultsMutex);
                        results.push(ThreadPoolResult<T, R> {
                            job, variable, index, locked, std::move(result)});
                        if (!standaloneResults) {
                            locked = true;
                        }
                        busyWorkers--;
                    }
                    if (!standaloneResults) {
                        std::unique_lock<std::mutex> lock(mutex);
                        variable.wait(lock, [&] {
                            return !working || !locked;
                        });
                    }
                } catch (std::exception& err) {
                    busyWorkers--;
                    if (onJobFailed) {
                        onJobFailed(job);
                    }
                    if (stopOnFail) {
                        std::lock_guard<std::mutex> lock(jobsMutex);
                        failed = true;
                    }
                    logger.error() << "uncaught exception: " << err.what();
                }
                jobsDone++;
            }
        }
    public:
        static constexpr int UNLIMITED = 0;
        static constexpr int HALF = -2;
        static constexpr int QUARTER = -4;

        /// @brief Main thread pool constructor
        /// @param name thread pool name (used in logger)
        /// @param workersSupplier workers factory function
        /// @param resultConsumer workers results consumer function
        /// @param maxWorkers max number of workers. Special values: 0 is 
        /// unlimited, -2 is half of auto count, -4 is quarter.
        ThreadPool(
            std::string name,
            supplier<std::unique_ptr<Worker<T, R>>> workersSupplier,
            consumer<R&&> resultConsumer,
            int maxWorkers=UNLIMITED
        )
            : logger(std::move(name)), resultConsumer(resultConsumer) {
            uint numThreads = std::thread::hardware_concurrency();
            switch (maxWorkers) {
                case UNLIMITED:
                    break;
                case HALF:
                    numThreads = std::max(1U, numThreads);
                    break;
                case QUARTER:
                    numThreads = std::max(1U, numThreads / 4);
                    break;
                default:
                    numThreads = std::max(
                        1U, std::min(numThreads, static_cast<uint>(maxWorkers))
                    );
                    break;
            }
            for (uint i = 0; i < numThreads; i++) {
                threads.emplace_back(
                    &ThreadPool<T, R>::threadLoop, this, i, workersSupplier()
                );
                workersBlocked.emplace_back();
            }
        }
        ~ThreadPool() {
            terminate();
        }

        bool isActive() const override {
            return working;
        }

        void setJobsSource(supplier<std::optional<T>>&& source) {
            jobsSource = std::move(source);
        }

        void terminate() override {
            if (!working) {
                return;
            }
            {
                std::lock_guard<std::mutex> lock(jobsMutex);
                working = false;
            }
            {
                std::lock_guard<std::mutex> lock(resultsMutex);
                while (!results.empty()) {
                    ThreadPoolResult<T, R> entry = results.front();
                    results.pop();
                    if (!standaloneResults) {
                        entry.locked = false;
                        entry.variable.notify_all();
                    }
                }
            }

            jobsMutexCondition.notify_all();
            for (auto& thread : threads) {
                thread.join();
            }
        }

        void update() override {
            pullResults();
        }

        size_t pullResults(size_t maxResults = -1) {
            if (!working) {
                return 0;
            }
            if (failed) {
                throw std::runtime_error("some job failed");
            }

            bool complete = false;
            size_t resultsProcessed = 0;
            {
                std::lock_guard<std::mutex> lock(resultsMutex);
                while (!results.empty() && resultsProcessed < maxResults) {
                    ThreadPoolResult<T, R> entry = std::move(results.front());
                    results.pop();

                    ++resultsProcessed;
                    try {
                        resultConsumer(std::move(entry.entry));
                    } catch (std::exception& err) {
                        logger.error() << err.what();
                        if (onJobFailed) {
                            onJobFailed(entry.job);
                        }
                        if (stopOnFail) {
                            std::lock_guard<std::mutex> jobsLock(jobsMutex);
                            failed = true;
                            complete = false;
                        }
                        break;
                    }

                    if (!standaloneResults) {
                        entry.locked = false;
                        entry.variable.notify_all();
                    }
                }

                if (onComplete && busyWorkers == 0 && results.empty()) {
                    std::lock_guard<std::mutex> jobsLock(jobsMutex);
                    if (jobs.empty()) {
                        onComplete();
                        complete = true;
                    }
                }
            }
            if (jobsSource) {
                bool jobsAdded = false;
                std::lock_guard<std::mutex> jobsLock(jobsMutex);
                while (true) {
                    auto job = jobsSource();
                    if (job.has_value()) {
                        jobs.push(std::move(job.value()));
                        jobsAdded = true;
                    } else {
                        break;
                    }
                }
                if (jobsAdded) {
                    jobsMutexCondition.notify_one();
                }
            }
            if (failed) {
                throw std::runtime_error("some job failed");
            }
            if (complete) {
                terminate();
            }
            return resultsProcessed;
        }

        void enqueueJob(T&& job) {
            {
                std::lock_guard<std::mutex> lock(jobsMutex);
                jobs.push(std::move(job));
            }
            jobsMutexCondition.notify_one();
        }

        void clearQueue() {
            std::lock_guard<std::mutex> lock(jobsMutex);
            jobs = {};
        }

        /// @brief If false: worker will be blocked until it's result performed
        void setStandaloneResults(bool flag) {
            standaloneResults = flag;
        }

        void setStopOnFail(bool flag) {
            stopOnFail = flag;
        }

        /// @brief onJobFailed called on exception thrown in worker thread.
        /// Use engine.postRunnable when calling terminate()
        void setOnJobFailed(consumer<T&> callback) {
            this->onJobFailed = callback;
        }

        /// @brief onComplete called in ThreadPool.update() when all jobs done
        /// if ThreadPool was not terminated
        void setOnComplete(runnable callback) {
            this->onComplete = callback;
        }

        uint getWorkTotal() const override {
            return jobs.size() + jobsDone + busyWorkers;
        }

        uint getWorkDone() const override {
            return jobsDone;
        }

        virtual void waitForEnd() override {
            using namespace std::chrono_literals;
            while (working) {
                std::this_thread::sleep_for(2ms);
                update();
            }
        }

        uint getWorkersCount() const {
            return threads.size();
        }
    };

}  // namespace util
