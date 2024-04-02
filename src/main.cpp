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
    // chassis->getOdomModel()->setPos({{11.25f, -29.22f}, 135.0f}); // Backwards
    chassis->getOdomModel()->setPos({{6.14f, -24.18f}, -44.23f});  // Forwards

    fuelArm.SetMin(875);
    fuelArm.SetMax(2272);

    gui->withCdsCell(cds);

    // Uncomment this to call RCS init menu on startup
    // RCS.InitializeTouchMenu("C3Hp3wz3E");

    gui->init();
}

int main(void) {
    config();

    // checkpoint3();
    // checkpoint4();
    
    cds->sampleAmbient();
    gui->pause();
    
    cds->sampleLight(0.8f);
    gui->pause();


	return 0;
}
