#include <globals.hpp>
using namespace starlib;

std::shared_ptr<Interface> gui = std::make_shared<Interface>();

std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>(
    FEHMotor::Motor1, FEHMotor::Motor0, 9.0f,
    encoderPair{FEHIO::P0_0, FEHIO::P0_1}, 2.47f,
    encoderPair{FEHIO::P1_0, FEHIO::P1_1}, 2.47f,
    offsetPair{-3.46875f, 3.46875f}, offsetPair{-4.0f, 4.0f},
    gui
);

std::shared_ptr<Cds> cds = std::make_shared<Cds>(FEHIO::P3_7, 0.65f, 0.6f, 30.0f);

FEHServo fuelArm(FEHServo::Servo0);
