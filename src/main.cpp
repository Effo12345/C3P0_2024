#include <FEHLCD.h>
#include <starlib/chassis/chassis.hpp>
#include <starlib/chassis/odometry.hpp>
#include <starlib/chassis/wayfinder.hpp>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHSD.h>
#include <FEHBuzzer.h>

FEHMotor driveL(FEHMotor::Motor1, 9.0);
FEHMotor driveR(FEHMotor::Motor0, 9.0);

/**
 * 
 * 
 * vmax = 200 rpm
 * 
 * 
*/

int main(void) {
    LCD.Clear(BLACK);
    //LCD.WriteLine("Hello, World!");


    /*
        Odom test
    */
//    starlib::Odom odometer;
//    odometer.withSensors({FEHIO::P0_0, FEHIO::P0_1, 2.5f}, {FEHIO::P1_0, FEHIO::P1_1, 2.5f});
//    odometer.withOffsets(-2.715f, 2.715f);

//     driveL.SetPercent(100);
//     driveR.SetPercent(100);

//     starlib::Odom::Pose pos = odometer.getPos();
//     odometer.tareWheelVelocity();
//    while(odometer.getPos().p.y < 96) {
//         odometer.step();
//         pos = odometer.getPos();
//         LCD.WriteAt(pos.p.x, 0, 0);
//         LCD.WriteAt(pos.p.y, 100, 0);
//         LCD.WriteAt(pos.a, 200, 0);

//         starlib::Odom::Velocity vel = odometer.getVel();

//         Sleep(10);
//    }

//     LCD.WriteAt(pos.p.x, 0, 0);
//     LCD.WriteAt(pos.p.y, 100, 0);
//     LCD.WriteAt(pos.a, 200, 0);

//     driveL.SetPercent(0);
//     driveR.SetPercent(0);


    /*  
        Pure pursuit test
    */
    // starlib::Odom odometer;
    // odometer.withSensors({FEHIO::P0_0, FEHIO::P0_1, 2.5f}, {FEHIO::P1_0, FEHIO::P1_1, 2.5f});
    // odometer.withOffsets(-2.715f, 2.715f);
    // odometer.setPos({{0, -35.961439836448605}, 0.0f});

    // starlib::Wayfinder pather;

    // FEHFile* targetVel = SD.FOpen("target.txt", "w");
    // FEHFile* measuredVel = SD.FOpen("measure.txt", "w");
    // LCD.Clear(BLACK);

    // starlib::Odom::Velocity vel = odometer.getVel();
    // starlib::Odom::Pose pos = odometer.getPos();

    // while(std::fabs(vel.leftVel) > 2.0f || pos.p.distanceTo({0, -35.961439836448605}) < 2.0f) {
    //     odometer.step();
    //     pos = odometer.getPos();
    //     LCD.WriteAt(pos.p.x, 0, 0);
    //     LCD.WriteAt(pos.p.y, 100, 0);
    //     LCD.WriteAt(pos.a, 200, 0);

    //     vel = odometer.getVel();
    //     LCD.WriteAt(vel.leftVel, 0, 20);
    //     LCD.WriteAt(vel.rightVel, 100, 20);

    //     starlib::Odom::Velocity motorPwr = pather.step(pos, vel);
    //     driveL.SetPercent(starlib::clamp(motorPwr.leftVel, -100.0f, 100.0f));
    //     driveR.SetPercent(starlib::clamp(motorPwr.rightVel, -100.0f, 100.0f));

    //     SD.FPrintf(targetVel, "%f\n", (motorPwr.leftVel / 100.0f) * 200.0f);
    //     SD.FPrintf(measuredVel, "%f\n", vel.leftVel);

    //     Sleep(10);
    // }

    // driveL.Stop();
    // driveR.Stop();

    // SD.FClose(targetVel);
    // SD.FClose(measuredVel);

    // Buzzer.Beep();

    starlib::Chassis chassis(FEHMotor::Motor1, FEHMotor::Motor0, 9.0f, 
                            {FEHIO::P0_0, FEHIO::P0_1}, 2.5f, -2.715f,
                            {FEHIO::P1_0, FEHIO::P1_1}, 2.5f, 2.715f);

    // chassis.setPIDConstants(1.35f, 0.005f, 0.3f);

    // chassis.turn(180.0f);

    chassis.setPPConstants(1.2f, 0.001f, 0.1f);

    /*
        Encoder test
    */
//    QuadEncoder encoder(FEHIO::P0_0, FEHIO::P0_1, 2.5f);

//    while(true) {
//         driveL.SetPercent(10);
//         float start = TimeNow();
//         while(TimeNow() - start < 1) {
//             int counts = encoder.ticks();
//             LCD.WriteAt(counts, 0, 0);
//             Sleep(50);
//             LCD.Clear(BLACK);
//         }

//         driveL.SetPercent(0);
//         Sleep(500);

//         driveL.SetPercent(-10);
//         start = TimeNow();
//         while(TimeNow() - start < 1) {
//             int counts = encoder.ticks();
//             LCD.WriteAt(counts, 0, 0);
//             Sleep(50);
//             LCD.Clear(BLACK);   
//         } 

//         driveL.SetPercent(0);
//         Sleep(500);    
//    }


    /*
        Quad encoder test
    */
    // QuadEncoder test(FEHIO::P0_0, FEHIO::P0_1, 2.5f);
    // QuadEncoder test2(FEHIO::P1_0, FEHIO::P1_1, 2.5f);

    // driveL.SetPercent(10);

    // while(test.degrees() < 360.0f) {
    //     Sleep(50);
    // }

    // driveL.SetPercent(0);
    // LCD.WriteAt(test.degrees(), 0, 0);
    // LCD.WriteAt(test.ticks(), 0, 20);
    // Sleep(500);

    // driveL.SetPercent(-10);
    // while(test.degrees() > 0.0f) {
    //     Sleep(500);
    // }

    // driveL.SetPercent(0);
    // LCD.WriteAt(test.degrees(), 0, 0);
    // LCD.WriteAt(test.ticks(), 0, 20);

    // driveL.SetPercent(25);
    // driveR.SetPercent(25);
    // QuadEncoder test(FEHIO::P0_0, FEHIO::P0_1, 2.5f);
    // QuadEncoder test2(FEHIO::P1_0, FEHIO::P1_1, 2.5f);

    // float start = TimeNow();
    // while(TimeNow() - start < 10.0f) {
    //     Sleep(50);
    //     LCD.WriteAt(TimeNow(), 0, 100);
    // }

    // LCD.WriteAt(test.degrees(), 0, 0);
    // LCD.WriteAt(test2.degrees(), 150, 0);

    // driveL.SetPercent(0);
    // driveR.SetPercent(0);


    // starlib::Chassis drive;

	return 0;
}
