#pragma once

#include <starlib/chassis/chassis.hpp>
#include <starlib/display/interface.hpp>
#include <FEHServo.h>

extern std::shared_ptr<starlib::Interface> gui;

extern std::shared_ptr<starlib::Chassis> chassis;

extern std::shared_ptr<starlib::Cds> cds;

extern FEHServo fuelArm;