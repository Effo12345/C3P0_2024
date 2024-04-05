#include <starlib/chassis/odometry.hpp>

namespace starlib {

void Odom::step() {
    // Get sensor data
    float leftWheel = leftEncoder->distanceTraveled();
    float rightWheel = rightEncoder->distanceTraveled();

    // Compute deltas
    float deltaLeft = leftWheel - prevLeftEncoder;
    float deltaRight = rightWheel - prevRightEncoder;
    
    // Update previous sensor data
    prevLeftEncoder = leftWheel;
    prevRightEncoder = rightWheel;

    // Calculate heading
    float heading = pos.a;
    heading -= (deltaLeft - deltaRight) / (leftOffset - rightOffset);

    float deltaHeading = heading - pos.a;
    float avgHeading = pos.a + deltaHeading / 2;

    float localX = 0.0f;   // localX will always be zero since only y-movement in body frame
    float localY = 0.0f;
    if(deltaHeading == 0.0f) {
        localY = deltaLeft;
    }
    else {
        localY = 2 * sin(deltaHeading / 2) * (deltaLeft / deltaHeading + leftOffset);
    }

    // Convert x and y to world frame
    pos.p.x += localY * sin(avgHeading);
    pos.p.y += localY * cos(avgHeading);
    pos.p.x += localX * -cos(avgHeading);
    pos.p.y += localX * sin(avgHeading);
    pos.a = heading;

    // Compute average wheel speeds since last update
    float lDeg = leftEncoder->degrees();
    float rDeg = rightEncoder->degrees();
    float deltaLDeg = lDeg - prevLeftDeg;
    float deltaRDeg = rDeg - prevRightDeg;

    float velocityDT = TimeNow() - lastVelocityTime;
    wheelVel.leftVel = ((deltaLDeg / 360) / velocityDT) * 60;  // in rpm
    wheelVel.rightVel = ((deltaRDeg / 360) / velocityDT) * 60;

    lastVelocityTime = TimeNow();
    prevLeftDeg = lDeg;
    prevRightDeg = rDeg;

    // Telemtry
    // std::string pose = std::to_string(pos.p.x) + " " + std::to_string(pos.p.y);
    // std::string headingStr = std::to_string(pos.a);

    // LCD.WriteAt((pose + " " + headingStr).c_str(), 0, 0);
    // LCD.WriteAt(headingStr.c_str(), 0, 20);
}


void Odom::withSensors(const QuadEncoder trackL, const QuadEncoder trackR) {
    leftEncoder = std::make_shared<QuadEncoder>(trackL);
    rightEncoder = std::make_shared<QuadEncoder>(trackR);
}

void Odom::withOffsets(const float offsetL, const float offsetR) {
    leftOffset = offsetL;
    rightOffset = offsetR;
}

void Odom::setPos(Pose pose, bool radians) {
    if(!radians)
        pose.a = degToRad(pose.a);

    if(pose.p.x != FLT_MAX)
        pos.p.x = pose.p.x;
    if(pose.p.y != FLT_MAX)
        pos.p.y = pose.p.y;
    if(pose.a != FLT_MAX)
        pos.a = pose.a;
}

Odom::Pose Odom::getPos(bool radians) {
    if (radians) 
        return pos;
    else
        return {pos.p, radToDeg(pos.a)};
}

Odom::Velocity Odom::getVel() {
    return wheelVel;
}

void Odom::lockAxes(bool x, bool y, bool theta) {
    lockX = x;
    lockY = y;
    lockTheta = theta;
}

}