#include <autonomous.hpp>

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
    chassis->getOdomModel()->setPos({{6.14f, -24.18f}, -45.0f});

    gui->init();
}

int main(void) {
    config();

    checkpoint2();

	return 0;
}
