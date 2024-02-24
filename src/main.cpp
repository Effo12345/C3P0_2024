#include <FEHLCD.h>
#include <starlib/chassis/chassis.hpp>
#include <starlib/chassis/odometry.hpp>
#include <FEHIO.h>

int main(void) {
    LCD.Clear(BLACK);
    //LCD.WriteLine("Hello, World!");


    /*
        Odom test
    */
   starlib::Odom odometer;
   odometer.withSensors({FEHIO::P0_0, FEHIO::P0_1, 2.5f}, {FEHIO::P1_0, FEHIO::P1_1, 2.5f});
   odometer.withOffsets(-2.75f, 2.75f);

   while(true) {
        starlib::Odom::Pose pos = odometer.getPos();

        LCD.WriteAt(pos.p.x, 0, 70);
        LCD.WriteAt(pos.p.y, 75, 70);
        LCD.WriteAt(pos.a, 175, 70);

        Sleep(10);
   }


    /*
        Quad encoder test
    */
    // QuadEncoder test(FEHIO::P0_0, FEHIO::P0_1, 2.5f);
    // QuadEncoder test2(FEHIO::P1_0, FEHIO::P1_1, 2.5f);

    // while(true) {
    //     std::pair<int, int> pinout = test.pinTest();
    //     std::pair<int, int> pinout2 = test2.pinTest();

    //     LCD.WriteAt(pinout.first, 0, 0);
    //     LCD.WriteAt(pinout.second, 50, 0);
    //     LCD.WriteAt(test.distanceTraveled(), 175, 0);

    //     LCD.WriteAt(pinout2.first, 0, 20);
    //     LCD.WriteAt(pinout2.second, 50, 20);
    //     LCD.WriteAt(test2.distanceTraveled(), 175, 20);

    //     Sleep(50);
    // }

    // starlib::Chassis drive;

	return 0;
}
