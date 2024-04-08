#include <starlib/chassis/settledutil.hpp>

namespace starlib {

    SettledUtil::SettledUtil(const float velThreshold, const float minSettledTime)
    : velTarget{velThreshold}, targetTimeSettled{minSettledTime} {}

    bool SettledUtil::isSettled(const Odom::Velocity& wheelVels) {
        if(wheelVels.leftVel > velTarget || wheelVels.rightVel > velTarget)
            settledStartTime = TimeNow();
            
        return (TimeNow() - settledStartTime) > targetTimeSettled;
    }

    void SettledUtil::reset() {
        settledStartTime = TimeNow();
    }

}