#pragma once
#include <atomic>

class RunnableThread {
    protected:
    std::atomic<bool> isReady{false};
    std::atomic<bool> isStarted{false};
    std::atomic<bool> shouldStop{false};
    public:
    virtual void run() = 0;
    virtual bool isThreadReady() const { return isReady; }
    virtual void start() { isStarted = true; }
    virtual void stop() { shouldStop = true; }
    virtual ~RunnableThread() = default;
};