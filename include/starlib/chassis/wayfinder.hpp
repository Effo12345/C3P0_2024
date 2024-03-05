#pragma once

#include <starlib/chassis/point.hpp>
#include <starlib/utils.hpp>
#include <starlib/chassis/odometry.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <FEHLCD.h>

namespace starlib { 

class Wayfinder {
    // Current path data
    std::vector<Point> points = {
        {0.08521028037383177, -0.1261804906542056},
        {0.12732427562448018, 1.8635830470146084},
        {0.16712399832589855, 3.851844186703226},
        {0.20413341616280467, 5.8369044487609045},
        {0.23627169181838598, 7.816402608260679},
        {0.26094161843139396, 9.787360363735042},
        {0.27421477165598157, 11.745293488476316},
        {0.2704393009977883, 13.68373783300089 },
        {0.24135744600151993, 15.593241399283205},
        {0.1746977951686979, 17.45977115652876},
        {0.05227361330673269, 19.262538965580024},
        {-0.15278334253269008, 20.97082971873033},
        {-0.48019853439303806, 22.53922300556921},
        {-0.9887842193200317, 23.900472806512166 },
        {-1.7656411560127092, 24.95498936381949},
        {-2.9379109935271606, 25.62953555517024},
        {-4.376950578653442, 26.047727344111077},
        {-5.995757083844992, 26.29317737205632},
        {-7.735345033666368, 26.422555653017717},
        {-9.555647555624182, 26.474378718432167},
        {-11.42912294951528, 26.475104807955695 },
        {-13.336578553853444, 26.443174821976378 },
        {-15.26412937166949, 26.391935866471275 },
        {-17.20199697023038, 26.331745512874313}
    };
    std::vector<float> velocity = {
        87.45773228453702,
        84.21161659696135,
        80.83793494408675,
        77.32307715461478,
        73.65137969632625,
        69.80411399919976,
        65.76002173892284,
        61.49483728867089,
        56.98150585601049,
        52.1912875504121,
        47.09557229399347,
        41.66818742667563,
        35.88343176386594,
        29.67721654970499,
        22.6715690048485,
        41.7759548685997,
        59.98560323716433,
        56.03415139219023,
        51.49171039928594,
        46.27639811971093,
        40.21109142074771,
        32.90546276962026,
        23.29945486582300,
        0
    };

    float lookaheadDistance = 7.0f;
    float maxRateChange = 200.0f;
    float trackWidth = 5.502f;
    bool reversed = false;
    bool useRateLimiter = true;
    float absoluteVelocityLimit = 200.0f;

    // Tuning constants
    float kV = 1.2f;
    float kA = 0.001f;
    float kP = 0.1f;

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

    //BE CAREFUL paramater fractionalIndex has a side effect and is modified as a reference
    Point findLookaheadPoint(std::vector<Point>& path, Point pos, float lookaheadDistance, float& fractionalIndex);

    float findArcCurvature(Point pos, float heading, Point lookahead, float lookaheadDistance);

    Odom::Velocity calculateWheelVelocities(float targetVelocity, float curvature, float trackWidth, bool isReversed);

    Odom::Velocity followPath(Odom::Pose pos, Odom::Velocity measuredVel);

public:
    Wayfinder(float v, float a, float p);

    Odom::Velocity step(Odom::Pose pos, Odom::Velocity vel);

    bool isSettled();

    // void setNewPath(std::vector<Point> path, std::vector<float> vel, Odom::Pose pos);
};

}