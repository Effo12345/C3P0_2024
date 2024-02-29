#include <FEHLCD.h>
#include <iostream>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <cmath>

#define turn_constant 0.5

enum direction {
    left,
    right
};

FEHMotor driveL(FEHMotor::Motor1, 9.0);
FEHMotor driveR(FEHMotor::Motor0, 9.0);

AnalogInputPin cds(FEHIO::P3_7);

void stop() {
    driveL.SetPercent(0);
    driveR.SetPercent(0);
}

void drive(int pct, float time) {
    driveL.SetPercent(pct);
    driveR.SetPercent(pct);
    Sleep(time);

    stop();
}

void turn(int pct, direction dir, float time) {
    if(dir == left) {
        driveL.SetPercent(-pct);
        driveR.SetPercent(pct);
    }
    else {
        driveL.SetPercent(pct);
        driveR.SetPercent(-pct);
    }

    Sleep(time);
    stop();
}

void waitForLight(float target, float tolerance, float timeout) {
    float start = TimeNow();
    while(cds.Value() > target && (TimeNow() - start) < timeout) {
        Sleep(50);
        LCD.WriteAt(cds.Value(), 0, 50);
    }
}


int main(void) {
    LCD.Clear(BLACK);

    waitForLight(1.7f, 0.2f, 10.0f);

    // Turn to face ramp and drive up it
    turn(25, right, 0.6);
    drive(25, 1.2);
    turn(25, left, 0.21);
    drive(50, 2.5);

    // Maneuver around passport
    float passportTurn = 0.5f;
    float passportDrive = 2.0f;
    turn(25, left, passportTurn);
    drive(25, passportDrive);

    // Face button and run into it
    float buttonTurn = 0.4f;
    float buttonDrive = 3.0f;
    turn(25, right, buttonTurn);
    drive(25, buttonDrive);

    // Undo path
    drive(-25, 2.5f);
    turn(25, left, 0.6f);
    drive(-25, 1.4f);
    turn(25, right, 0.4f);
    drive(-25, 4);
    
    // DriveTurn(1, 0.5); //realign
    // DriveStraight(1, 3.5); //drive up the ramp
    // DriveTurn(-1, turn_constant); //turn left
    // DriveStraight(1, 0.5); //forward a bit
    // DriveTurn(1, turn_constant); //turn right
    // DriveStraight(1, 0); //drive into buttons
    // DriveStraight(-1, 2); //drive backwards
    // DriveTurn(1, turn_constant); //turn
    // DriveStraight(1, .5); //forward a bit
    // DriveTurn(1, turn_constant); //turn
    // DriveStraight(1, 0); //drive down the ramp
}


