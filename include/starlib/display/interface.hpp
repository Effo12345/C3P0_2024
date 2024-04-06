#pragma once

#include <FEHLCD.h>
#include <starlib/chassis/odometry.hpp>
#include <starlib/cds.hpp>
#include <FEHBattery.h>
#include <FEHUtility.h>
#include <FEHBuzzer.h>


namespace starlib {

class Interface {
    const int imgRows = 240;
    const int imgCols = 120;

    const float canvasScaleX = 18.0f;
    const float canvasScaleY = 36.0f;
    const int robotRadius = 4;

    bool isInitialized = false;

    Odom::Pose pos {};
    std::pair<float, float> encVals;
    int leverNum = -1;
    float ambientLight {};
    float currentLight {};
    int lightColor = BLACK;

    std::shared_ptr<Cds> cdsCell;

    void drawImage();
    void drawRobot();

    void writePos();
    void writeEncoderVals();
    void writeLeverNum();
    void writeLightLevel();
    void writeBatteryVoltage();

    float canvasX(float x);
    float canvasY(float y);

    void fetchCdsValues();
public:
    void withCdsCell(std::shared_ptr<Cds> cds);

    void init();
    void update(bool full = false);

    void setPos(const Odom::Pose& position);
    void setEncoderVals(std::pair<float, float> vals);
    void setLeverNum(int num);
    void setColor(int color);
    void setLightLevel(float light);

    void pause();
    void clear();
};

}