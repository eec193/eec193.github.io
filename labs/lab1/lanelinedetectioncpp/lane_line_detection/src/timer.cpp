#include <iostream>
#include "timer.hpp"

Timer::Timer() {
  this->start = std::chrono::high_resolution_clock::now();
}

Timer::~Timer() {}

void Timer::reset() {
  this->start = std::chrono::high_resolution_clock::now();
}

double Timer::time() {
  this->end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double, std::micro>(end - start).count()/1000000.0;
}
