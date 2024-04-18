#include <globals.hpp>
using namespace starlib;

std::shared_ptr<Interface> gui = std::make_shared<Interface>();

std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>(
    // Motors, motor voltage
    FEHMotor::Motor1, FEHMotor::Motor0, 9.0f,                          // V
    // Left encoder and wheel radius
    encoderPair{FEHIO::P0_0, FEHIO::P0_1}, 2.47f,                      // in
    // Right encoder and wheel radius
    encoderPair{FEHIO::P1_0, FEHIO::P1_1}, 2.47f,                      // in
    // Drive and turn wheel offsets
    offsetPair{-3.46875f, 3.46875f}, offsetPair{-4.0f, 4.0f},          // in
    // Settled threshold and min settled time
    1.0f, 0.1f,                                                        // rpm, s
    gui
);

// Red color threshold, starting light threshold, starting light timeout
std::shared_ptr<Cds> cds = std::make_shared<Cds>(FEHIO::P3_7, 0.65f, 0.55f, 30.0f);

FEHServo fuelArm(FEHServo::Servo0);
FEHServo passportArm(FEHServo::Servo1);
