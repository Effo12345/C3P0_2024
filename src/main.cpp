#include <autonomous.hpp>
#include <FEHRCS.h>

void startup() {
    // Set drive parameters
    chassis->setPIDConstants(1.6f, 0.005f, 0.3f);
    chassis->setPPConstants(1.2f, 0.001f, 0.1f);

    // Set servo parameters
    fuelArm.SetMin(875);
    fuelArm.SetMax(2272);

    highButton.SetMin(588);
    highButton.SetMax(1712);

    gui->withCdsCell(cds);

    // Connect to RCS
    RCS.InitializeTouchMenu("C3Hp3wz3E");

    gui->init();

    // Wait for permission to sample ambient light level
    gui->pause();
    cds->sampleAmbient();

    // Await final action
    gui->pause();

    // Set absolute pos, reset encoders
    chassis->getOdomModel()->setPos({{12.01f, -29.9f}, 135.0f}); // Backwards
    // chassis->getOdomModel()->setPos({{6.14f, -24.18f}, -44.23f});  // Forwards
    // chassis->getOdomModel()->setPos({{0.0f, 0.0f}, 0.0f});

    chassis->getOdomModel()->tarSensors();

    // Hold control until starting light crosses threshold
    cds->awaitStartingLight();
}

int main(void) {
    // Finish setting up internal state and await final action
    startup();

    // Run autonomous route
    showcase();


	return 0;
}
