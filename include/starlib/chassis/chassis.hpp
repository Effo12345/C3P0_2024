#pragma once

#include <FEHMotor.h>
#include <FEHIO.h>

#include <starlib/chassis/odometry.hpp>
#include <starlib/chassis/wayfinder.hpp>
#include <starlib/display/interface.hpp>
#include <starlib/chassis/settledutil.hpp>

using encoderPair = std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin>;
using offsetPair = std::pair<float, float>;

namespace starlib {
    
class Chassis {
    std::shared_ptr<FEHMotor> driveL;
    std::shared_ptr<FEHMotor> driveR;

    std::shared_ptr<Odom> odometer = std::make_shared<Odom>();
    std::shared_ptr<Wayfinder> pather;

    std::shared_ptr<SettledUtil> settled;

    std::shared_ptr<Interface> gui;

    float kP, kI, kD;

    offsetPair turnOffsets;
    offsetPair driveOffsets;

    void updateGui();
    void updateGui(Odom::Pose position);

public:
    Chassis(FEHMotor::FEHMotorPort leftMotor, FEHMotor::FEHMotorPort rightMotor, float motorVoltage,
            encoderPair encoderL, float diamL, 
            encoderPair encoderR, float diamR,
            offsetPair drive, offsetPair turn,
            float settledVel, float settledTime,
            const std::shared_ptr<Interface> interface);

    void setPIDConstants(float pConst, float iConst, float dConst);
    void setPPConstants(float kV, float kA, float kP);

    std::shared_ptr<Odom> getOdomModel();

    void followNewPath(std::vector<Point> path, std::vector<float> vel, bool isReversed = false);
    void turn(float angle, float timeOut = 1.0f);
    void drive(float leftPct, float rightPct);
    void driveFor(float pwr, float time);

    void awaitSettled();
    void resetSettled();
};

}