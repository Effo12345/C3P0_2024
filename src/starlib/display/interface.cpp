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

        int vectorLength = 7;

        int xLength = vectorLength * cos(-pos.a * (M_PI / 180) + M_PI/2);
        int yLength = vectorLength * sin(-pos.a * (M_PI / 180) + M_PI/2);

        LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                     canvasX(pos.p.x + xLength) + imgCols / 2.0f, canvasY(pos.p.y + yLength) + imgRows / 2.0f
        );

        int orthogonalXLength = (vectorLength / 2) * cos(-pos.a * (M_PI / 180));
        int orthogonalYLength = (vectorLength / 2) * sin(-pos.a * (M_PI / 180));

        LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                     canvasX(pos.p.x + orthogonalXLength) + imgCols / 2.0f, canvasY(pos.p.y + orthogonalYLength) + imgRows / 2.0f
        );

        LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                     canvasX(pos.p.x - orthogonalXLength) + imgCols / 2.0f, canvasY(pos.p.y - orthogonalYLength) + imgRows / 2.0f
        );
    }

    void Interface::writePos() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt('x', 140, 0);
        LCD.WriteAt('y', 210, 0);
        LCD.WriteAt("theta", 260, 0);

        LCD.WriteAt(std::to_string(pos.p.x).substr(0, 5).c_str(), 118, 20);
        LCD.WriteAt(std::to_string(pos.p.y).substr(0, 5).c_str(), 182, 20);
        LCD.WriteAt(std::to_string(pos.a).substr(0, 6).c_str(), 248, 20);
    }

    void Interface::writeDrivePower() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt("driveL", 130, 60);
        LCD.WriteAt("driveR", 230, 60);

        LCD.WriteAt(wheelVels.leftVel, 135, 80);
        LCD.WriteAt(wheelVels.rightVel, 235, 80);
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

        std::string batteryOutput = "Battery: " + std::to_string(Battery.Voltage()).substr(0, 5) + " V";
        LCD.WriteAt(batteryOutput.c_str(), 125, 220);
    }


    void Interface::setPos(const Odom::Pose& position) {
        pos = position;
    }

    void Interface::setMotorSpeeds(const Odom::Velocity& vels) {
        wheelVels = vels;
    }

    void Interface::setColor(int color) {
        lightColor = color;
    }

    void Interface::setLightLevel(float light) {
        currentLight = light;
    }


    void Interface::pause() {
        if(!isInitialized)
            return;

        LCD.Clear(BLACK);
        int tmpX, tmpY;
        while(!LCD.Touch(&tmpX, &tmpY)) {
            update(true);
            Sleep(10);
        }

        Sleep(250);
        LCD.ClearBuffer();
    }

    void Interface::clear() {
        LCD.Clear(BLACK);
    }

    void Interface::update(bool full) {
        if(!isInitialized)
            return;

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
        isInitialized = true;
        // drawImage();
    }

}