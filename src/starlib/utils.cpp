#include <starlib/utils.hpp>

namespace starlib {

float rateLimiter::constrain(float input, float maxRateChange) {
    float maxChange = (TimeNow() - prevTime) * maxRateChange;
    prevTime = TimeNow();
    float temp = input - output;
    temp = clamp(temp, -maxChange, maxChange);

    output += temp;
    return output;
}

void rateLimiter::reset() {
    output = 0.0f;
    prevTime = TimeNow();
}

float rateLimiter::getDeltaTime() {
    return TimeNow() - prevTime;
}

/**
 * Returns the input degrees in radians
 */
float degToRad(float deg) {
    return deg * (M_PI / 180);
}

/**
 * Returns the input radians in degrees
 */
float radToDeg(float rad) {
    return rad / (M_PI / 180);
}

}