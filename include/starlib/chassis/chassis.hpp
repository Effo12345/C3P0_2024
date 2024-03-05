#pragma once

#include <FEHMotor.h>
#include <FEHIO.h>

#include <starlib/chassis/odometry.hpp>
#include <starlib/chassis/wayfinder.hpp>

namespace starlib {
    
class Chassis {
    std::shared_ptr<FEHMotor> driveL;
    std::shared_ptr<FEHMotor> driveR;

    std::shared_ptr<Odom> odometer = std::make_shared<Odom>();
    std::shared_ptr<Wayfinder> pather;

    float kP, kI, kD;

    void drive(float leftPct, float rightPct);

public:
    Chassis(FEHMotor::FEHMotorPort leftMotor, FEHMotor::FEHMotorPort rightMotor, float motorVoltage,
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderL, float diamL, float offsetL, 
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderR, float diamR, float offsetR);

    void setPIDConstants(float pConst, float iConst, float dConst);
    void setPPConstants(float kV, float kA, float kP);

    void followNewPath(std::vector<Point>& path, std::vector<float>& vel);
    void turn(float angle);
};

}