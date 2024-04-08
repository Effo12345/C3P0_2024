#include <globals.hpp>
using namespace starlib;

std::shared_ptr<Interface> gui = std::make_shared<Interface>();

std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>(
    FEHMotor::Motor1, FEHMotor::Motor0, 9.0f,                          // V
    encoderPair{FEHIO::P0_0, FEHIO::P0_1}, 2.47f,                      // in
    encoderPair{FEHIO::P1_0, FEHIO::P1_1}, 2.47f,                      // in
    offsetPair{-3.46875f, 3.46875f}, offsetPair{-4.0f, 4.0f},          // in
    1.0f, 0.1f,                                                        // rpm, s
    gui
);

std::shared_ptr<Cds> cds = std::make_shared<Cds>(FEHIO::P3_7, 0.65f, 0.6f, 30.0f);

FEHServo fuelArm(FEHServo::Servo0);
