#include <FEHLCD.h>
#include <starlib/chassis/chassis.hpp>
#include <FEHIO.h>

int main(void) {
    LCD.Clear(BLACK);
    //LCD.WriteLine("Hello, World!");
    QuadEncoder test(FEHIO::P0_0, FEHIO::P0_1, 2.5f);
    QuadEncoder test2(FEHIO::P1_0, FEHIO::P1_1, 2.5f);

    while(true) {
        std::pair<int, int> pinout = test.pinTest();
        std::pair<int, int> pinout2 = test2.pinTest();

        LCD.WriteAt(pinout.first, 0, 0);
        LCD.WriteAt(pinout.second, 50, 0);
        LCD.WriteAt(test.distanceTraveled(), 175, 0);

        LCD.WriteAt(pinout2.first, 0, 20);
        LCD.WriteAt(pinout2.second, 50, 20);
        LCD.WriteAt(test2.distanceTraveled(), 175, 20);

        Sleep(50);
    }

    starlib::Chassis drive;

	return 0;
}
