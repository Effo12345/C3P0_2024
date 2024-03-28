#include <starlib/chassis/chassis.hpp>

namespace starlib {

Chassis::Chassis(FEHMotor::FEHMotorPort leftMotor, FEHMotor::FEHMotorPort rightMotor, float motorVoltage, 
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderL, float diamL, float offsetL, 
            std::pair<FEHIO::FEHIOPin, FEHIO::FEHIOPin> encoderR, float diamR, float offsetR,
            const std::shared_ptr<Interface> interface) {
    driveL = std::make_shared<FEHMotor>(leftMotor, motorVoltage);
    driveR = std::make_shared<FEHMotor>(rightMotor, motorVoltage);

    odometer->withSensors({encoderL.first, encoderL.second, diamL}, {encoderR.first, encoderR.second, diamR});
    odometer->withOffsets(offsetL, offsetR);

    gui = interface;
}

void Chassis::setPIDConstants(float pConst, float iConst, float dConst) {
    kP = pConst;
    kI = iConst;
    kD = dConst;
}

void Chassis::setPPConstants(float kV, float kA, float kP) {
    pather = std::make_shared<Wayfinder>(kV, kA, kP);
}

std::shared_ptr<Odom> Chassis::getOdomModel() {
    return odometer;
}

void Chassis::followNewPath(std::vector<Point> path, std::vector<float> vel, bool isReversed) {
    Odom::Pose startPos = odometer->getPos();
    pather->setNewPath(path, vel, startPos, isReversed);
    
    Odom::Velocity velocity;
    Odom::Pose pos;

    do {
        odometer->step();
        velocity = odometer->getVel();
        pos = odometer->getPos();
        Odom::Velocity pwr = pather->step(pos, velocity);

        drive(pwr.leftVel, pwr.rightVel); // Motor speeds sent to gui here

        gui->setPos(pos);
        gui->update();

        Sleep(10);
    } while(std::fabs(velocity.leftVel) > 2.0f || pos.p.distanceTo(startPos.p) < 2.0f);   // Todo: add settled util

    drive(0.0f, 0.0f);

    gui->clear();
    gui->update(true);
}

void Chassis::turn(float setpoint, float timeOut) {
    float error = 2.0f;
    float prevError;
    float integral;

    float start = TimeNow();
    while(std::fabs(error) > 1.0f && TimeNow() - start < timeOut) {
        const float Ki_active_t = 10;
        const float Ki_limit_t  = 100000;

        odometer->step();

        Odom::Pose position = odometer->getPos();
        float adj_heading = position.a;

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

        // std::string aOut = "A: " + std::to_string(adj_heading);
        // std::string errorOut = "Error: " + std::to_string(error);
        // std::string powerOut = "Power: " + std::to_string(power);
        // LCD.WriteAt(aOut.c_str(), 0, 0);
        // LCD.WriteAt(errorOut.c_str(), 0, 20);
        // LCD.WriteAt(powerOut.c_str(), 0, 40);

        drive(power, -power); // Writes motor powers to gui

        gui->setPos(position);
        gui->update();

        Sleep(10);
    }

    drive(0.0f, 0.0f);

    gui->clear();
    gui->update(true);
}

void Chassis::drive(float leftPct, float rightPct) {
    driveL->SetPercent(clamp(leftPct, -100.0f, 100.0f));
    driveR->SetPercent(clamp(rightPct, -100.0f, 100.0f));

    gui->setMotorSpeeds({leftPct, rightPct});
}

void Chassis::driveFor(float pwr, float time) {
    drive(pwr, pwr);

    float start = TimeNow();
    while(TimeNow() - start < time) {
        odometer->step();
        Sleep(10);
    }

    drive(0.0f, 0.0f);
}

}