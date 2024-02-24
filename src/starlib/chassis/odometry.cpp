#include <starlib/chassis/odometry.hpp>

namespace starlib {

/**
 * Returns the input degrees in radians
 */
float Odom::degToRad(float deg) {
    return deg * (M_PI / 180);
}

/**
 * Returns the input radians in degrees
 */
float Odom::radToDeg(float rad) {
    return rad / (M_PI / 180);
}


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
    float velocityDT = TimeNow() - lastVelocityTime;
    wheelVel.leftVel = (deltaLeft / velocityDT) * 60;  // in rpm
    wheelVel.rightVel = (deltaRight / velocityDT) * 60;
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
    if(radians)
        pos = pose;
    else
        pos = {pose.p, degToRad(pose.a)};
}

void Odom::tareWheelVelocity() {
    lastVelocityTime = TimeNow();
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

}