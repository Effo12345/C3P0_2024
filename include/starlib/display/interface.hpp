#pragma once

#include <FEHLCD.h>
#include <starlib/chassis/odometry.hpp>
#include <starlib/cds.hpp>
#include <FEHBattery.h>
#include <FEHUtility.h>


namespace starlib {

class Interface {
    const int imgRows = 240;
    const int imgCols = 120;

    const float canvasScaleX = 18.0f;
    const float canvasScaleY = 36.0f;
    const int robotRadius = 4;

    bool isInitialized = false;

    Odom::Pose pos {};
    Odom::Velocity wheelVels {};
    float ambientLight {};
    float currentLight {};
    int lightColor = BLACK;

    float startCtrlTime {};
    float startRndrTime {};

    std::shared_ptr<Cds> cdsCell;

    void drawImage();
    void drawRobot();

    void writePos();
    void writeDrivePower();
    void writeLightLevel();
    void writeTimingValues();
    void writeBatteryVoltage();

    float canvasX(float x);
    float canvasY(float y);

    void fetchCdsValues();
public:
    void withCdsCell(std::shared_ptr<Cds> cds);

    void init();
    void update(bool full = false);

    void setPos(const Odom::Pose& position);
    void setMotorSpeeds(const Odom::Velocity& vels);
    void setColor(int color);
    void setLightLevel(float light);

    void pause();
    void clear();
};

}