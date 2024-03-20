#include <autonomous.hpp>
#include <FEHRCS.h>

/**
 * 
 * 
 * vmax = 200 rpm
 * 
 * 
*/

void config() {
    chassis->setPIDConstants(1.35f, 0.005f, 0.3f);
    chassis->setPPConstants(1.2f, 0.001f, 0.1f);
    chassis->getOdomModel()->setPos({{11.25f, -29.22f}, 135.0f});

    fuelArm.SetMin(0);
    fuelArm.SetMax(255);

    // Uncomment this to call RCS init menu on startup
    // RCS.InitializeTouchMenu("C3Hp3wz3E");

    gui->init();
}

int main(void) {
    config();

    checkpoint3();

	return 0;
}
