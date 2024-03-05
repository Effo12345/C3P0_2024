#include <starlib/chassis/chassis.hpp>

namespace starlib {

Chassis::Chassis(FEHMotor::FEHMotorPort leftMotor, FEHMotor::FEHMotorPort rightMotor, float motorVoltage, 
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderL, float diamL, float offsetL, 
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderR, float diamR, float offsetR) {
    driveL = std::make_shared<FEHMotor>(leftMotor, motorVoltage);
    driveR = std::make_shared<FEHMotor>(rightMotor, motorVoltage);

    odometer->withSensors({encoderL.first, encoderL.second, diamL}, {encoderR.first, encoderR.second, diamR});
    odometer->withOffsets(offsetL, offsetR);
}

void Chassis::setPIDConstants(float pConst, float iConst, float dConst) {
    kP = pConst;
    kI = iConst;
    kD = dConst;
}

void Chassis::setPPConstants(float kV, float kA, float kP) {
    pather = std::make_shared<Wayfinder>(kV, kA, kP);
}

void Chassis::followNewPath(std::vector<Point>& path, std::vector<float>& vel) {
    pather->setNewPath(path, vel, odometer->getPos());
    odometer->tareWheelVelocity();

    do {
        odometer->step();
        Odom::Velocity pwr = pather->step(odometer->getPos(), odometer->getVel());

        drive(pwr.leftVel, pwr.rightVel);

        Sleep(10);
    } while(true);   // Todo: add settled util

    drive(0.0f, 0.0f);
}

void Chassis::turn(float setpoint) {
    float error = 2.0f;
    float prevError;
    float integral;

    float start = TimeNow();
    while(true/*std::fabs(error) > 1.0 && TimeNow() - start < 100000*/) {
        const float Ki_active_t = 10;
        const float Ki_limit_t  = 100000;

        odometer->step();

        float adj_heading = odometer->getPos().a;

        error = setpoint - adj_heading; 

        if(std::fabs(error) < Ki_active_t)
            integral = integral + error;
        else
            integral = 0;

        if(integral > Ki_limit_t)
            integral = Ki_limit_t;
        else if(integral < -1 * Ki_limit_t)
            integral = -1 * Ki_limit_t;

        float derivative = error - prevError;
        float power = (error * kP) + (integral * kI) + (derivative * kD);
        prevError = error;

        std::string aOut = "A: " + std::to_string(adj_heading);
        std::string errorOut = "Error: " + std::to_string(error);
        std::string powerOut = "Power: " + std::to_string(power);
        // LCD.WriteAt(aOut.c_str(), 0, 0);
        // LCD.WriteAt(errorOut.c_str(), 0, 20);
        // LCD.WriteAt(powerOut.c_str(), 0, 40);

        //drive(power, -power);

        Sleep(10);
    }

    drive(0.0f, 0.0f);
    //tracking.reset();
}

void Chassis::drive(float leftPct, float rightPct) {
    driveL->SetPercent(clamp(leftPct, -100.0f, 100.0f));
    driveR->SetPercent(clamp(rightPct, -100.0f, 100.0f));
}

}