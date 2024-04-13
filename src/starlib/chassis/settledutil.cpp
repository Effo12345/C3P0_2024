#include <starlib/chassis/settledutil.hpp>

namespace starlib {

/**
 * Sets internal state variables to define settled conditions
 * 
 * @param velThreshold Minimum wheel speed (rpm) for the robot to be considered settled
 * @param minSettledTime Time the robot must be below velThreshold to be considered settled
*/
SettledUtil::SettledUtil(const float velThreshold, const float minSettledTime)
: velTarget{velThreshold}, targetTimeSettled{minSettledTime} {}

/**
 * Determines if the robot is settled if the wheel velocities are below the
 * target velocity for at least the minimum length of time
 * 
 * @param wheelVels Pair of left and right wheel velocities
 * @return Whether the robot is settled
*/
bool SettledUtil::isSettled(const Odom::Velocity& wheelVels) {
    // Reset length of time settled for if either wheel crosses threshold
    if(std::fabs(wheelVels.leftVel) > velTarget || std::fabs(wheelVels.rightVel) > velTarget)
        settledStartTime = TimeNow();
        
    // Return false until both wheels are below threshold for target time
    return (TimeNow() - settledStartTime) > targetTimeSettled;
}

/**
 * Tare settled time at the start of a new movement
*/
void SettledUtil::reset() {
    settledStartTime = TimeNow();
}

} // namespace starlib