#pragma once

#include <starlib/chassis/odometry.hpp>

namespace starlib {


class SettledUtil {
    const float velTarget;
    const float targetTimeSettled;
    float settledStartTime {};

public:
    SettledUtil(const float velThreshold, const float minSettledTime);
    bool isSettled(const Odom::Velocity& wheelVels);
    void reset();
};


}