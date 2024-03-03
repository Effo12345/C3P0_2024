#pragma once

#include <cmath>
#include <algorithm>

namespace starlib {

//Rate limiter used in pure pursuit algorithm
class rateLimiter {
  float output = 0.0;      //Output value (must persist between function calls)
public:
  float constrain(float input, float maxRateChange);
  void reset();
};


template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

extern float degToRad(float deg);
extern float radToDeg(float rad);

}