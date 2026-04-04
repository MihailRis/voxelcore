#include "timeutil.hpp"

#include <iostream>

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

timeutil::Timer::Timer() {
    start = high_resolution_clock::now();
}
int64_t timeutil::Timer::stop() {
    return duration_cast<microseconds>(high_resolution_clock::now() - start)
        .count();
}

int64_t timeutil::Timer::stopNs() {
    return duration_cast<nanoseconds>(high_resolution_clock::now() - start)
        .count();
}

timeutil::ScopeLogTimer::ScopeLogTimer(long long id) : scopeid_(id) {
}

timeutil::ScopeLogTimer::~ScopeLogTimer() {
    std::cout << "Scope " << scopeid_ << " finished in "
              << ScopeLogTimer::stop() << " micros. \n";
}

void timeutil::from_value(float value, int& hour, int& minute, int& second) {
    value *= 24;
    hour = value;
    value *= 60;
    minute = int(value) % 60;
    value *= 60;
    second = int(value) % 60;
}
