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
    chassis->setPIDConstants(1.6f, 0.005f, 0.3f);
    chassis->setPPConstants(1.2f, 0.001f, 0.1f);

    fuelArm.SetMin(875);
    fuelArm.SetMax(2272);

    highButton.SetMin(588);
    highButton.SetMax(1712);

    gui->withCdsCell(cds);

    // Uncomment this to call RCS init menu on startup
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

    cds->awaitStartingLight();
}

int main(void) {
    config();

    // chassis->followNewPath({
    //     {0, -0.1261804906542056},
    //     {-0.08544044178095483, 1.6732145364692872},
    //     {-0.14910040756016713, 3.4434593449672164},
    //     {-0.1636008050278173, 5.150801394324874},
    //     {-0.09478540271469706, 6.751676504050944},
    //     {0.10436533598685638, 8.185818108767256},
    //     {0.5013272115460441, 9.367066230321322},
    //     {1.1942612797423025, 10.16983292339613},
    //     {2.2994566353149, 10.649611794933552},
    //     {3.678831105927242, 10.912102729384982},
    //     {5.237060431461636, 11.029116982566867},
    //     {6.9066872350772535, 11.049752901332687},
    //     {8.637693729545122, 11.008064274350982},
    //     {10.389628029593847, 10.928331380917163},
    //     {12.125314401175203, 10.828993491746603},
    //     {13.804065420560747, 10.725341705607478}

    // }, {
    //     59.199819292042925,
    //     54.11250731393604,
    //     48.59340430711135,
    //     42.602227435836085,
    //     36.08596221147724,
    //     28.963212907248707,
    //     20.978443113985126,
    //     17.809795418222194,
    //     41.317582376104305,
    //     56.957814018027655,
    //     52.38468509161091,
    //     47.00886988939082,
    //     40.69092223467984,
    //     33.0840123984697,
    //     23.199640562924653,
    //     0
    // });

    // gui->pause();

    // chassis->turn(0.0f);

    showcase();



	return 0;
}
