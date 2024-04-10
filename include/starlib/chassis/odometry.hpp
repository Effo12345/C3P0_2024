#pragma once

#include <FEHIO.h>
#include <FEHUtility.h>
#include <starlib/chassis/point.hpp>
#include <memory>
#include <starlib/utils.hpp>
#include <float.h>

namespace starlib {

class Odom {
public:
    struct Pose {
        Point p {0, 0};
        float a {0};
    };

    struct Velocity {
        float leftVel;
        float rightVel;

        Velocity operator+(const Velocity& v) {
            return {leftVel + v.leftVel, rightVel + v.rightVel};
        } 

        Velocity operator-(const Velocity& v) {
            return {leftVel - v.leftVel, rightVel - v.rightVel};
        }

        Velocity operator*(const float f) {
            return {leftVel * f, rightVel * f};
        }

        Velocity operator/(const float f) {
            return {leftVel / f, rightVel / f};
        }
    };

private:
    float leftOffset;
    float rightOffset;

    float prevLeftEncoder {};
    float prevRightEncoder {};

    float prevLeftDeg {};
    float prevRightDeg {};

    float lastVelocityTime {};
    Velocity wheelVel;

    bool lockX = false;
    bool lockY = false;
    bool lockTheta = false;

    std::shared_ptr<QuadEncoder> leftEncoder;
    std::shared_ptr<QuadEncoder> rightEncoder;

    Pose pos {{0.0f, 0.0f}, 0.0f};

public:
    void withSensors(const QuadEncoder trackL, const QuadEncoder trackR);
    void withOffsets(const float offsetL, const float offsetR);
    void setPos(Pose pose, bool radians = false);

    void step();
    void lockAxes(bool x, bool y, bool theta);

    Pose getPos(bool radians = false);
    Velocity getVel();
    std::pair<float, float> getRawEncVals();

    void tarSensors();
};

}