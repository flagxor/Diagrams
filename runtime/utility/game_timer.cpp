#include "game_timer.h"
#include <iostream>

namespace {
const std::chrono::milliseconds k050ms = std::chrono::milliseconds(50);
const std::chrono::milliseconds k000ms = std::chrono::milliseconds(0);
}

namespace diagrammar {

GameTimer::GameTimer() {
}
void GameTimer::Initialize() {
  // is the time monotonic?
  init_time_ = std::chrono::steady_clock::now();
  last_time_ = std::chrono::steady_clock::now();
  accu_time_ = std::chrono::microseconds::zero();
}

// this call takes less than 1 us to finish
int32_t GameTimer::BeginNextFrame() {
  std::chrono::steady_clock::time_point curr_time =
      std::chrono::steady_clock::now();
  elap_time_ = std::chrono::duration_cast<TimeUnit>(curr_time - last_time_);
  elap_time_ = elap_time_ > k000ms ? elap_time_ : k000ms;
  elap_time_ = elap_time_ < k050ms ? elap_time_ : tick_time_;
  int32_t num_ticks = 0;
  accu_time_ += elap_time_;
  while (accu_time_ >= tick_time_) {
    accu_time_ -= tick_time_;
    ++num_ticks;
  }
  last_time_ = curr_time;
  if (accu_ticks_ < INT64_MAX - num_ticks)
    accu_ticks_ += num_ticks;
  else
    accu_ticks_ = 0;
  return num_ticks;
}

// warning, must be in unit second
double GameTimer::tick_time() const {
  std::chrono::duration<float> fs = tick_time_;
  return fs.count();
}

double GameTimer::now() const {
  std::chrono::duration<float> fs = std::chrono::steady_clock::now() - init_time_;
  return fs.count();
}

int64_t GameTimer::ticks() const {
  return accu_ticks_;
}

}