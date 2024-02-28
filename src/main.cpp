#include <FEHLCD.h>
#include <starlib/chassis/chassis.hpp>
#include <starlib/chassis/odometry.hpp>
#include <FEHIO.h>
#include <FEHMotor.h>

FEHMotor driveL(FEHMotor::Motor0, 9.0);
FEHMotor driveR(FEHMotor::Motor1, 9.0);

int main(void) {
    LCD.Clear(BLACK);
    //LCD.WriteLine("Hello, World!");


    /*
        Odom test
    */
//    starlib::Odom odometer;
//    odometer.withSensors({FEHIO::P0_0, FEHIO::P0_1, 2.5f}, {FEHIO::P1_0, FEHIO::P1_1, 2.5f});
//    odometer.withOffsets(-2.75f, 2.75f);

//    while(true) {
//         starlib::Odom::Pose pos = odometer.getPos();

//         LCD.WriteAt(pos.p.x, 0, 70);
//         LCD.WriteAt(pos.p.y, 75, 70);
//         LCD.WriteAt(pos.a, 175, 70);

//         Sleep(10);
//    }

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
    QuadEncoder test(FEHIO::P0_0, FEHIO::P0_1, 2.5f);
    QuadEncoder test2(FEHIO::P1_0, FEHIO::P1_1, 2.5f);

    driveL.SetPercent(10);

    while(test.degrees() < 360.0f) {
        Sleep(50);
    }

    driveL.SetPercent(0);
    LCD.WriteAt(test.degrees(), 0, 0);
    LCD.WriteAt(test.ticks(), 0, 20);

    // starlib::Chassis drive;

	return 0;
}
