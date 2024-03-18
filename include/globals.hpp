#pragma once

#include <starlib/chassis/chassis.hpp>
#include <starlib/display/interface.hpp>
#include <starlib/cds.hpp>

extern std::shared_ptr<starlib::Interface> gui;

extern std::shared_ptr<starlib::Chassis> chassis;

extern AnalogInputPin cds;