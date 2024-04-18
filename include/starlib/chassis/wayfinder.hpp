#pragma once

#include <starlib/chassis/point.hpp>
#include <starlib/utils.hpp>
#include <starlib/chassis/odometry.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <FEHLCD.h>
#include <FEHSD.h>

namespace starlib { 

class Wayfinder {
    // Current path data
    std::vector<Point> points {};
    std::vector<float> velocity {};

    float lookaheadDistance = 7.0f;
    float maxRateChange = 800.0f;  // 300.0f for super speed
    float trackWidth = 6.9375f;
    bool reversed = false;
    bool useRateLimiter = true;
    float absoluteVelocityLimit = 200.0f;

    // Tuning constants (set in constructor)
    float kV {};
    float kA {};
    float kP {};

    rateLimiter limit;

    int lastClosestPointIndex = 0;
    float lastFractionalIndex = 0.0f;
    Point lastLookahead {0.0f, 0.0f};
    Odom::Velocity lastVelocities {0.0f, 0.0f};

    float error = 0.0f;

    //Generalized template function to get the sign of a number
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    int closestPoint = 1;
    int intersectIndex = 0;
    float fractionalIndex = 0.0f;

    Point lookaheadPoint;
    
    int findClosestPointIndex(std::vector<Point>& path, Point pos, int startIndex = 0);

    //BE CAREFUL: paramater fractionalIndex has a side effect and is modified as a reference
    Point findLookaheadPoint(std::vector<Point>& path, Point pos, float lookaheadDistance, float& fractionalIndex);

    float findArcCurvature(Point pos, float heading, Point lookahead, float lookaheadDistance);

    Odom::Velocity calculateWheelVelocities(float targetVelocity, float curvature, float trackWidth, bool isReversed);

    Odom::Velocity followPath(Odom::Pose pos, Odom::Velocity measuredVel);

public:
    Wayfinder(float v, float a, float p);

    Odom::Velocity step(Odom::Pose pos, Odom::Velocity vel);

    void setNewPath(std::vector<Point>& path, std::vector<float>& vel, Odom::Pose pos, bool isReversed = false);
};

} // namespace starlib