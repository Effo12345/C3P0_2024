#include <starlib/display/interface.hpp>

namespace starlib {

/**
 * Draw a black and white image of the field to the display (requires 
 * modified firmware)
*/
void Interface::drawImage() {
    LCD.PrintFieldImage(0, 0);
}

/**
 * Scale between course coordinates and pixel coordinates for accurate object
 * placement
*/
float Interface::canvasX(float x) {
    return x / (canvasScaleX/(imgCols/2.0f));
}

float Interface::canvasY(float y) {
    return -y / (canvasScaleY/(imgRows/2.0f));
}

/**
 * Draw a dot with an arrow to represent the robot's current position and 
 * rotation relative to the field. Takes ~300 ms
*/
void Interface::drawRobot() {
    LCD.SetFontColor(BLUE);
    // Draw robot pos
    LCD.FillCircle(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f, robotRadius);

    int vectorLength = 7;

    // Draw long line in heading direction
    int xLength = vectorLength * cos(-pos.a * (M_PI / 180) + M_PI/2);
    int yLength = vectorLength * sin(-pos.a * (M_PI / 180) + M_PI/2);

    LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                    canvasX(pos.p.x + xLength) + imgCols / 2.0f, canvasY(pos.p.y + yLength) + imgRows / 2.0f
    );

    // Draw short line perpendicular to heading direction
    int orthogonalXLength = (vectorLength / 2) * cos(-pos.a * (M_PI / 180));
    int orthogonalYLength = (vectorLength / 2) * sin(-pos.a * (M_PI / 180));

    LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                    canvasX(pos.p.x + orthogonalXLength) + imgCols / 2.0f, canvasY(pos.p.y + orthogonalYLength) + imgRows / 2.0f
    );

    LCD.DrawLine(canvasX(pos.p.x) + imgCols / 2.0f, canvasY(pos.p.y) + imgRows / 2.0f,
                    canvasX(pos.p.x - orthogonalXLength) + imgCols / 2.0f, canvasY(pos.p.y - orthogonalYLength) + imgRows / 2.0f
    );
}

/**
 * Write the robot's current x, y, and theta coordinates to the display
*/
void Interface::writePos() {
    LCD.SetFontColor(WHITE);

    LCD.WriteAt('x', 140, 0);
    LCD.WriteAt('y', 210, 0);
    LCD.WriteAt("theta", 260, 0);

    // Take only the first few digits of each float to save space
    LCD.WriteAt(std::to_string(pos.p.x).substr(0, 5).c_str(), 118, 20);
    LCD.WriteAt(std::to_string(pos.p.y).substr(0, 5).c_str(), 182, 20);
    LCD.WriteAt(std::to_string(pos.a).substr(0, 6).c_str(), 248, 20);
}

/**
 * Write the current encoder measurements (left and right) to the display
*/
void Interface::writeEncoderVals() {
    LCD.SetFontColor(WHITE);

    LCD.WriteAt("encL", 138, 50);
    LCD.WriteAt("encR", 243, 50);

    LCD.WriteAt(encVals.first, 120, 70);
    LCD.WriteAt(encVals.second, 220, 70);
}

/**
 * Draw bubbles on the display to indicate the target lever; the chosen lever
 * will be a filled in circle
*/
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

    // Draw outlines for all 3 options
    LCD.DrawCircle(circleCoords[0][0], circleCoords[0][1], circleRad);
    LCD.DrawCircle(circleCoords[1][0], circleCoords[1][1], circleRad);
    LCD.DrawCircle(circleCoords[2][0], circleCoords[2][1], circleRad);

    // Fill circle of selected lever, if set
    if(leverNum != -1)
        LCD.FillCircle(circleCoords[leverNum][0], circleCoords[leverNum][1], circleRad);
}

/**
 * Write the current rcs value, the ambient light level, and the ticket kiosk
 * color, if set. Live fetches value from the cds
*/
void Interface::writeLightLevel() {
    // Updates cds data even when robot is paused
    fetchCdsValues();

    LCD.SetFontColor(WHITE);

    LCD.WriteAt("Amb", 135, 160);
    LCD.WriteAt("Curr", 198, 160);
    LCD.WriteAt("Smpl", 265, 160);

    LCD.WriteAt(ambientLight, 125, 180);
    LCD.WriteAt(currentLight, 190, 180);

    // Fill in a rectangle with ticket kiosk color, if set
    if(lightColor != BLACK) {
        LCD.SetFontColor(lightColor);
        LCD.FillRectangle(278, 180, 20, 20);
    }
    else {
        LCD.SetFontColor(GRAY);
        LCD.DrawRectangle(278, 180, 20, 20);
    }
}

/**
 * Get and display current battery voltage
*/
void Interface::writeBatteryVoltage() {
    LCD.SetFontColor(WHITE);

    // Shorten float to save space
    std::string batteryOutput = "Battery: " + std::to_string(Battery.Voltage()).substr(0, 5) + " V";
    LCD.WriteAt(batteryOutput.c_str(), 125, 220);
}

/**
 * Block control flow and display telemetry until the user touches the display
 * @requires this.isInitialized = true
*/
void Interface::pause() {
    if(!isInitialized)
        return;

    // Clear touch buffer from past interactions
    LCD.ClearBuffer();

    // Full clear before starting full updates
    clear();
    int tmpX, tmpY;
    // Run until user touches screen
    while(!LCD.Touch(&tmpX, &tmpY)) {
        update(true);
        Sleep(10);
    }

    // Clear any lingering touch inputs
    Sleep(250);
    LCD.ClearBuffer();
}

/**
 * Fill black over the screen to reset all pixels
*/
void Interface::clear() {
    LCD.Clear(BLACK);
}

/**
 * Redraw the interface to the display based on internal state variables
 * @param full Whether to update the course display
 * @requires this.initialized = true
*/
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

/**
 * Prepare interface to begin drawing to display
*/
void Interface::init() {
    LCD.Clear(BLACK);
    isInitialized = true;

    // Clear lingering touch input from RCS menu
    LCD.ClearBuffer();
}


/**
 * Fetch current cds values from internal reference, if set
*/
void Interface::fetchCdsValues() {
    if(cdsCell != nullptr) {
        ambientLight = cdsCell->getAmbientSample();
        currentLight = cdsCell->getOffsetValue();
        lightColor = cdsCell->getColor();
    }
}

/**
 * Set reference to cds class so values can be fetched automatically
*/
void Interface::withCdsCell(std::shared_ptr<Cds> cds) {
    cdsCell = cds;
}

/*
    * Start trivial setters
*/

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


} // namespace starlib