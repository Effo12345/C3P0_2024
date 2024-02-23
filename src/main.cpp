#include <FEHLCD.h>
#include <starlib/chassis/chassis.hpp>

int main(void) {
    LCD.Clear(BLACK);
    LCD.WriteLine("Hello, World!");

    starlib::Chassis drive;

	return 0;
}
