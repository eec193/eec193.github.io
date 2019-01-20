#ifndef TIMER_HPP
#define TIMER_HPP
#include <chrono> // NOLINT (build/c++11)

class Timer {
 private:
  // start time of timer
  std::chrono::high_resolution_clock::time_point start;

  // end time of timer
  std::chrono::high_resolution_clock::time_point end;

 public:

  /*
   * Constructor
   */
  Timer();

  /*
   * Destructor
   */
  ~Timer();

  /*
   * Resets the timer
   */
  void reset();

  /*
   * Returns time elapsed in seconds
   */
  double time();


};

#endif /* TIMER_HPP */
