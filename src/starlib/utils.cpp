#include <starlib/utils.hpp>

namespace starlib {

float rateLimiter::constrain(float input, float maxRateChange) {
    float maxChange = (0.0144f) * maxRateChange;
    float temp = input - output;
    temp = clamp(temp, -maxChange, maxChange);

    output += temp;
    return output;
}

void rateLimiter::reset() {
    output = 0.0;
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