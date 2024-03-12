#pragma once

#include <FEHLCD.h>
#include <starlib/chassis/odometry.hpp>
#include <FEHBattery.h>
#include <FEHUtility.h>


namespace starlib {

class Interface {
    int imgRows = 240;
    int imgCols = 120;

    float canvasScaleX = 18.0f;
    float canvasScaleY = 36.0f;
    int robotRadius = 4;

    Odom::Pose pos {};
    float driveLPwr {};
    float driveRPwr {};
    float ambientLight {};
    float currentLight {};
    int lightColor = BLACK;

    float startCtrlTime {};
    float startRndrTime {};

    void drawImage();
    void drawRobot();

    void writePos();
    void writeDrivePower();
    void writeLightLevel();
    void writeTimingValues();
    void writeBatteryVoltage();

    float canvasX(float x);
    float canvasY(float y);
public:
    void init();
    void update(bool full = false);

    void setPos(const Odom::Pose& position);

    void pause();
};

}