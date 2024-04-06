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

    void Interface::writeEncoderVals() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt("encL", 138, 50);
        LCD.WriteAt("encR", 243, 50);

        LCD.WriteAt(encVals.first, 120, 70);
        LCD.WriteAt(encVals.second, 220, 70);
    }

    void Interface::writeLeverNum() {
        LCD.SetFontColor(WHITE);

        LCD.WriteAt("B", 160, 100);
        LCD.WriteAt("A1", 205, 100);
        LCD.WriteAt("A", 260, 100);

        LCD.SetFontColor(RED);

        int circleRad = 10;
        int circleCoords[3][2] = {
            {267, 130},
            {217, 130},
            {166, 130}
        };

        LCD.DrawCircle(circleCoords[0][0], circleCoords[0][1], circleRad);
        LCD.DrawCircle(circleCoords[1][0], circleCoords[1][1], circleRad);
        LCD.DrawCircle(circleCoords[2][0], circleCoords[2][1], circleRad);

        if(leverNum != -1)
            LCD.FillCircle(circleCoords[leverNum ][0], circleCoords[leverNum][1], circleRad);
    }

    void Interface::writeLightLevel() {
        fetchCdsValues();

        LCD.SetFontColor(WHITE);

        LCD.WriteAt("Amb", 135, 160);
        LCD.WriteAt("Curr", 198, 160);
        LCD.WriteAt("Smpl", 265, 160);

        LCD.WriteAt(ambientLight, 125, 180);
        LCD.WriteAt(currentLight, 190, 180);

        if(lightColor != BLACK) {
            LCD.SetFontColor(lightColor);
            LCD.FillRectangle(278, 180, 20, 20);
        }
        else {
            LCD.SetFontColor(GRAY);
            LCD.DrawRectangle(278, 180, 20, 20);
        }
    }

    void Interface::writeBatteryVoltage() {
        LCD.SetFontColor(WHITE);

        std::string batteryOutput = "Battery: " + std::to_string(Battery.Voltage()).substr(0, 5) + " V";
        LCD.WriteAt(batteryOutput.c_str(), 125, 220);
    }


    void Interface::setPos(const Odom::Pose& position) {
        pos = position;
    }

    void Interface::setEncoderVals(std::pair<float, float> vals) {
        encVals = vals;
    }

    void Interface::setLeverNum(int num) {
        leverNum = num;
    }

    void Interface::setColor(int color) {
        lightColor = color;
    }

    void Interface::setLightLevel(float light) {
        currentLight = light;
    }

    void Interface::fetchCdsValues() {
        if(cdsCell != nullptr) {
            ambientLight = cdsCell->getAmbientSample();
            currentLight = cdsCell->getOffsetValue();
            lightColor = cdsCell->getColor();
        }
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

        if(full) {
            drawImage();
            drawRobot();        
        }

        writePos();
        writeEncoderVals();
        writeLeverNum();
        writeLightLevel();
        writeBatteryVoltage();
    }

    void Interface::init() {
        LCD.Clear(BLACK);
        isInitialized = true;
        // drawImage();
    }

    void Interface::withCdsCell(std::shared_ptr<Cds> cds) {
        cdsCell = cds;
    }

}