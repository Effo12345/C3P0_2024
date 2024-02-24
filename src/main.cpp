#include <FEHLCD.h>
#include <iostream>
#include <FEHIO.h>
#include <FEHMotor.h>

#define turn_constant 0.5

FEHMotor left_motor(FEHMotor::Motor0, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);
AnalogInputPin bump(FEHIO::P0_0);

void DriveStraight(int, float); //1 for forward, -1 for backward. float for seconds
void DriveTurn(int, float); //1 for right, -1 for left. float for seconds

void DriveStraight(int direction, float distance)
{
    left_motor.SetPercent((direction * 25));
    right_motor.SetPercent((direction * 25));

    if(distance != 0)
    {
       Sleep(distance); 
    }
    else
    {
        while(bump.Value() == 1)
        {}
    }

    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
    Sleep(1.0);
}

void DriveTurn(int direction, float angle)
{
    left_motor.SetPercent((direction * 15));
    right_motor.SetPercent((direction * -15));
    Sleep(angle);

    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
    Sleep(1.0);
}


int main(void) 
{
    DriveTurn(1, 0.5); //realign
    DriveStraight(1, 3.5); //drive up the ramp
    DriveTurn(-1, turn_constant); //turn left
    DriveStraight(1, 0.5); //forward a bit
    DriveTurn(1, turn_constant); //turn right
    DriveStraight(1, 0); //drive into buttons
    DriveStraight(-1, 2); //drive backwards
    DriveTurn(1, turn_constant); //turn
    DriveStraight(1, .5); //forward a bit
    DriveTurn(1, turn_constant); //turn
    DriveStraight(1, 0); //drive down the ramp
}


