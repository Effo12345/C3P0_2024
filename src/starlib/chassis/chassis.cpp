#include <starlib/chassis/chassis.hpp>
#include <FEHSD.h>

namespace starlib {

/**
 * Take in all necessary values to manage drivetrain movements.
 * 
 * @param leftMotor Lerft drivetrain motor
 * @param rightMotor Right drivetrain motor
 * @param motorVoltage Max voltage to drive both motors to
 * @param encoderL Digital input pins of the left quad encoder
 * @param diamL Left wheel diameter
 * @param encoderR Digital input pins of the right quad encoder
 * @param diamR Right wheel diameter
 * @param drive Wheel offsets to use while driving normally
 * @param turn Wheel offsets to use while point turning
 * @param settledVel Maximum wheel velocity (rpm) for the robot to be settled
 * @param settledTime Minimum time for the robot to be settled
 * @param interface Reference to gui to update its values during control loops
*/
Chassis::Chassis(FEHMotor::FEHMotorPort leftMotor, FEHMotor::FEHMotorPort rightMotor, float motorVoltage,
            encoderPair encoderL, float diamL, 
            encoderPair encoderR, float diamR,
            offsetPair drive, offsetPair turn,
            float settledVel, float settledTime,
            const std::shared_ptr<Interface> interface) {
    driveL = std::make_shared<FEHMotor>(leftMotor, motorVoltage);
    driveR = std::make_shared<FEHMotor>(rightMotor, motorVoltage);

    // Odometer encapsulates all tracking and encoding
    odometer->withSensors({encoderL.first, encoderL.second, diamL}, {encoderR.first, encoderR.second, diamR});
    odometer->withOffsets(drive.first, drive.second); // Drive offsets by default

    turnOffsets = turn;
    driveOffsets = drive;

    // Determine when the robot is settled during control loops
    settled = std::make_shared<SettledUtil>(settledVel, settledTime);

    gui = interface;
}

/**
 * Sets control constants to be used for PID-based point turns
 * 
 * @param pConst Proprtional multiplicative scale
 * @param iConst Integral multiplicative scale
 * @param dConst Derivative multiplicative scale
*/
void Chassis::setPIDConstants(float pConst, float iConst, float dConst) {
    kP = pConst;
    kI = iConst;
    kD = dConst;
}

/**
 * Sets control constants for Pure Pursuit-based arc movements
 * 
 * @param kV Feedforward velocity constant
 * @param kA Feedforward acceleration constant
 * @param kP Proportional feedback constant
*/
void Chassis::setPPConstants(float kV, float kA, float kP) {
    pather = std::make_shared<Wayfinder>(kV, kA, kP);
}

/**
 * Follows the path laid out by the parameters using the pure pursuit path
 * following algorithm
 * 
 * @param path The x-y coordinates of the path to be followed
 * @param vel The robot's target velocity at each point along the path
 * @param isReversed Whether the robot should path backwards or forwards
*/
void Chassis::followNewPath(std::vector<Point> path, std::vector<float> vel, bool isReversed) {
    Odom::Pose startPos = odometer->getPos();
    pather->setNewPath(path, vel, startPos, isReversed);

    settled->reset();
    
    Odom::Velocity velocity;
    Odom::Pose pos;

    // Loop until the robot is settled and it is more than 2 inches from its
    // starting position
    do {
        // Track position and feed live pos and vel data to path follower
        odometer->step();
        velocity = odometer->getVel();
        pos = odometer->getPos();
        Odom::Velocity pwr = pather->step(pos, velocity);

        // Spin wheels at speed requested by pather
        drive(pwr.leftVel, pwr.rightVel);

        updateGui(pos);

        Sleep(10);
    } while(!settled->isSettled(velocity) || pos.p.distanceTo(startPos.p) < 2.0f);

    drive(0.0f, 0.0f);

    // Full update at end of movement
    // gui->clear();
    gui->update();
}

/**
 * Turns to face the absolute angle given by the setpoint. A more positive angle
 * than current heading denotes clockwise rotation
 * 
 * @param setpoint Angle to turn towards
 * @param timeOut Maximum time limit to prevent infinite loop
*/
void Chassis::turn(float setpoint, float timeOut) {
    settled->reset();

    float error = 2.0f;
    float prevError;
    float integral;

    // Run until the robot is settled or the timeout is reached
    float start = TimeNow();
    while(!settled->isSettled(odometer->getVel()) && TimeNow() - start < timeOut) {
        const float Ki_active_t = 10;
        const float Ki_limit_t  = 100000;

        // Track position
        odometer->step();

        float adj_heading = odometer->getPos().a;

        error = setpoint - adj_heading; 

        // Only use integral if the error is below a threshold
        if(std::fabs(error) < Ki_active_t)
            integral = integral + error;
        else
            integral = 0;

        // Cap maximum value of integral to prevent windup
        if(integral > Ki_limit_t)
            integral = Ki_limit_t;
        else if(integral < -1 * Ki_limit_t)
            integral = -1 * Ki_limit_t;

        // Use proportional, integral, and derivative terms to control wheel speeds
        float derivative = error - prevError;
        float power = (error * kP) + (integral * kI) + (derivative * kD);
        prevError = error;


        drive(power, -power);

        updateGui();

        Sleep(10);
    }

    drive(0.0f, 0.0f);

    // gui->clear();
    gui->update();
}

/**
 * Drive with a specified power for a length of time
 * 
 * @param pwr Percentage to drive the motors
 * @param time Length of time to drive for
*/
void Chassis::driveFor(float pwr, float time) {
    settled->reset();

    drive(pwr, pwr);

    float start = TimeNow();
    while(TimeNow() - start < time) {
        odometer->step();
        updateGui();
        Sleep(10);
    }

    // Stop the motors, but keep tracking until the robot is settled
    drive(0.0f, 0.0f);

    awaitSettled();
}

/**
 * Drive the motors at a given percentage. Enforces +- 100% max power
 * 
 * @param leftPct Left wheel percent
 * @param rightPCt Right wheel percent
*/
void Chassis::drive(float leftPct, float rightPct) {
    driveL->SetPercent(clamp(leftPct, -100.0f, 100.0f));
    driveR->SetPercent(clamp(rightPct, -100.0f, 100.0f));
}


/**
 * Return the internal odom model
*/
std::shared_ptr<Odom> Chassis::getOdomModel() {
    return odometer;
}


/**
 * Continue tracking the robot's movements until settled condition is reached
 * 
 * @requires [settled->reset() was called at the start of the movement]
*/
void Chassis::awaitSettled() {
    while(!settled->isSettled(odometer->getVel())) {
        odometer->step();
        updateGui();
        Sleep(10);
    }
}

/**
 * Reset the internal settled util
*/
void Chassis::resetSettled() {
    settled->reset();
}

/**
 * Update the gui with position and encoder values
*/
void Chassis::updateGui(Odom::Pose position) {
    gui->setPos(position);
    gui->setEncoderVals(odometer->getRawEncVals());
    gui->update();
}

void Chassis::updateGui() {
    updateGui(odometer->getPos());
}

} // namespace starlib