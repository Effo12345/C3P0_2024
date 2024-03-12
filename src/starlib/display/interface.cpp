#include <starlib/display/interface.hpp>

namespace starlib {

    void Interface::drawImage() {
        LCD.PrintFieldImage(0, 0);
    }

    float Interface::canvasX(float x) {
        return x / (canvasScaleX/(imgCols/2.0f));
    }

    float Interface::canvasY(float y) {
        return -y / (canvasScaleY/(imgRows/2.0f));
    }

    void Interface::drawRobot() {
        LCD.SetFontColor(BLUE);
        LCD.FillCircle(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f, robotRadius);
    }

    void Interface::writePos() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt('x', 140, 0);
        LCD.WriteAt('y', 210, 0);
        LCD.WriteAt("theta", 260, 0);

        LCD.WriteAt(pos.p.x, 120, 20);
        LCD.WriteAt(pos.p.y, 190, 20);
        LCD.WriteAt(pos.a, 260, 20);
    }

    void Interface::writeDrivePower() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt("driveL", 130, 60);
        LCD.WriteAt("driveR", 235, 60);

        LCD.WriteAt(driveLPwr, 135, 80);
        LCD.WriteAt(driveRPwr, 240, 80);
    }

    void Interface::writeLightLevel() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt("Amb", 135, 120);
        LCD.WriteAt("Curr", 198, 120);
        LCD.WriteAt("Smpl", 265, 120);

        LCD.WriteAt(ambientLight, 125, 140);
        LCD.WriteAt(currentLight, 190, 140);

        if(lightColor != BLACK) {
            LCD.SetFontColor(lightColor);
            LCD.FillRectangle(278, 140, 20, 20);
        }
        else {
            LCD.SetFontColor(GRAY);
            LCD.DrawRectangle(278, 140, 20, 20);
        }
    }

    void Interface::writeTimingValues() {
        LCD.WriteAt("Ctrl", 130, 170);
        LCD.WriteAt("Rndr", 240, 170);

        LCD.WriteAt(startRndrTime - startCtrlTime, 130, 190);
        LCD.WriteAt(TimeNow() - startRndrTime, 240, 190);
    }

    void Interface::writeBatteryVoltage() {
        LCD.SetFontColor(WHITE);

        std::string batteryOutput = "Battery: " + std::to_string(Battery.Voltage()).substr(0, 5);
        LCD.WriteAt(batteryOutput.c_str(), 125, 220);
    }

    void Interface::setPos(const Odom::Pose& position) {
        pos = position;
    }


    void Interface::pause() {
        int tmpX, tmpY;
        while(!LCD.Touch(&tmpX, &tmpY)) {
            update(true);
            Sleep(10);
        }
    }

    void Interface::update(bool full) {
        startRndrTime = TimeNow();

        if(full) {
            drawImage();
            drawRobot();        
        }

        writePos();
        writeDrivePower();
        writeLightLevel();
        writeBatteryVoltage();
        writeTimingValues();

        startCtrlTime = TimeNow();
    }

    void Interface::init() {
        LCD.Clear(BLACK);
        drawImage();
    }

}