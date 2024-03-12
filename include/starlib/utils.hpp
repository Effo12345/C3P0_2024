#pragma once

#include <cmath>
#include <algorithm>
#include <FEHUtility.h>

namespace starlib {

//Rate limiter used in pure pursuit algorithm
class rateLimiter {
  float output = 0.0f;      //Output value (must persist between function calls)
  float prevTime {};
public:
  float constrain(float input, float maxRateChange);
  void reset();
  float getDeltaTime();
};


template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

extern float degToRad(float deg);
extern float radToDeg(float rad);

}