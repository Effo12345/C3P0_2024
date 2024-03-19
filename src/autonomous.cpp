#include <autonomous.hpp>
#include <FEHSD.h>

void checkpoint2() {
    FEHFile* posOut = SD.FOpen("moves.txt", "w");
    gui->clear();

    // chassis->turn(45.0f);
    // Sleep(100);

    // float startRun = TimeNow();
    // while(cds.Value() > 1.1f && (TimeNow() - startRun) < 30.0f) {
    //     // LCD.WriteAt(cds.Value(), 0 , 160);
    //     Sleep(100);
    // }

    starlib::Odom::Pose position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Start");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    chassis->followNewPath({
        {6.475981308411215, -24.260302464806035},
        {4.690354859923755, -23.48359715585645},
        {2.9164050743649668, -22.697147896103065},
        {1.1698442518903804, -21.889663874380744},
        {-0.5302869639879365, -21.046465996301738},
        {-2.157670655718212, -20.147411593159596},
        {-3.674818961158477, -19.163495968823632},
        {-5.127256641994576, -18.01436584613621},
        {-6.537056803962704, -16.756211582109117},
        {-7.919649510810176, -15.428399240806913},
        {-9.286157608749, -14.059479924362787},
        {-10.646219015981458, -12.67155214682717}
    }, {
        81.8058840978188,
        77.90434358568933,
        73.81150513693478,
        69.5154490543608,
        65.00092207311562,
        60.2509037889402,
        55.24061930385303,
        49.587006624310334,
        43.06051151614531,
        35.22481724393297,
        24.936538285691046,
        0,
    });
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "To ramp");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    chassis->turn(2.3f);
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Align ramp");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    chassis->drive(50.0f, 50.0f);
    while(chassis->getOdomModel()->getPos().p.y < 9.0f) {
        chassis->getOdomModel()->step();
        gui->setPos(chassis->getOdomModel()->getPos());
        gui->update();
        Sleep(10);
    }
    chassis->drive(0.0f, 0.0f);
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Ramp top");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    chassis->turn(35.0f);
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Light align");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    
    chassis->followNewPath({
        {-6.987242990654205, 10.565512955754718},
        {-6.631263564372162, 12.497932433532984},
        {-6.277256760786646, 14.425057195887943},
        {-5.925388383092541, 16.340540900935363},
        {-5.577151879467246, 18.23625358920891},
        {-5.234583213372861, 20.101111983315782},
        {-4.900588002802528, 21.919298623456367},
        {-4.579468448021306, 23.667393404199178},
        {-4.277453491355618, 25.311487516789903},
        {-4.004883177570093, 26.80073608659584}
    }, {
        81.25731009021703,
        76.26779638329391,
        70.94384242805704,
        65.22281140274217,
        59.01730431439476,
        52.19786885137709,
        44.55529024000087,
        35.69644417466868,
        24.608833778288325,
        0
    });
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Light path");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    bool isBlue = false;
    bool isRed = false;
    float lightValue = cds.Value();

    if(lightValue > 0.9f) { // Blue light
        // LCD.Clear(BLUE);
        gui->setColor(BLUE);
        isBlue = true;
    }
    else {
        // LCD.Clear(RED);
        gui->setColor(RED);
        isRed = true;
    }

    // LCD.WriteAt(lightValue, 0, 160);


    chassis->followNewPath({
        {-3.5788317757009342, 26.464254778184625},
        {-3.8820821596777453, 24.846135609468433},
        {-4.105000222108283, 23.259734731847185},
        {-4.152488740395803, 21.741864806020455},
        {-3.903346364524626, 20.339844821704045},
        {-3.2529056137939865, 18.979776465409575},
        {-2.33789713366057, 17.650813772461177},
        {-1.2540698422918704, 16.347061498476005},
        {-0.07063476044405659, 15.066390057943053},
        {1.1585649644660037, 13.810353843272768},
        {2.38588785046729, 12.584400806222007}
    }, {
        58.63154660579609,
        53.952251935671676,
        48.97152093588722,
        43.72932455830766,
        38.165151312173094,
        52.29094380367214,
        47.09588934474705,
        40.93272708451573,
        33.428887870517535,
        23.560837416297144,
        0
    }, true);
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, "Luggage path");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    chassis->turn(3.0f);
    Sleep(100);

    gui->pause();

    position = chassis->getOdomModel()->getPos();
    SD.FPrintf(posOut, " Button turn");
    SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);

    if(isBlue) {
        chassis->followNewPath({
            {2.4710981308411215, 12.500280479119205},
            {2.364182379103605, 14.418177693769296},
            {2.267094746353529, 16.32461750728169},
            {2.193388803007719, 18.20615240216336},
            {2.1591968917435636, 20.045331507408655},
            {2.1825621583440333, 21.867335768138776},
            {2.241163944231351, 23.645656299821976},
            {2.318146691966515, 25.346538379631706},
            {2.399556812709748, 26.92471117317729},
            {2.4710981308411215, 28.314901974446308}
        }, {
            71.1699375806957,
            66.71191812846374,
            61.96473758896679,
            56.895305134228614,
            51.462964705979175,
            45.44609370799173,
            38.677848519637855,
            30.8405009936795,
            21.10567996548882,
            0
        });

        position = chassis->getOdomModel()->getPos();
        SD.FPrintf(posOut, " Blue press");
        SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);
    }
    else if (isRed) {
        chassis->followNewPath({
            {2.4710981308411215, 12.500280479119205},
            {3.3808242380413502, 13.908478673184971},
            {4.201220234538762, 15.325473108767609},
            {4.831284232366762, 16.761559845564122},
            {5.271049508734557, 18.3506794921803},
            {5.582630211264703, 20.02874193865011},
            {5.808127116927789, 21.745591660350314},
            {5.976238855150036, 23.457066065023838},
            {6.106735169777372, 25.118310783631568},
            {6.21344679070188, 26.67639270067122},
            {6.305560747663551, 28.062540993137898}
        }, {
            72.23575725803994,
            68.42168769611432,
            64.47925701395472,
            60.46274044671931,
            55.92952802711366,
            50.81293072700767,
            45.031583091439224,
            38.43872956434318,
            30.729464616833017,
            21.084254245965255,
            0
        });

        position = chassis->getOdomModel()->getPos();
        SD.FPrintf(posOut, "Red press");
        SD.FPrintf(posOut, " %f %f %f\n", position.p.x, position.p.y, position.a);
    }
    Sleep(100);

    gui->pause();

    chassis->followNewPath({
        {4.516144859813084, 27.89430033893229},
        {4.036361579479535, 26.04973515421361},
        {3.543748403515439, 24.22295679847026},
        {3.025632586320412, 22.434521782935413},
        {2.4643112031896606, 20.711010229227732},
        {1.8353451366482842, 19.089131031218162},
        {1.103768569916942, 17.621416789393194},
        {0.21861540314188746, 16.384488685912025},
        {-0.9384517686418987, 15.281091040904206},
        {-2.2585992712990026, 14.25606012622874},
        {-3.6585044647879843, 13.266343763157256},
        {-5.067223453270739, 12.274434892362823},
        {-6.415201055591579, 11.2425046661457},
        {-7.623427053429274, 10.126453906467551},
        {-8.591069390052589, 8.86904867746786},
        {-9.224018703208424, 7.373106587560481},
        {-9.63100464178213, 5.717165758813131},
        {-9.885533670945978, 3.954760834531265},
        {-10.037268895608525, 2.122826334197863},
        {-10.11985112825937, 0.24743203104209147},
        {-10.156123319914679, -1.6521444467262427},
        {-10.161691157450855, -3.560387546939642},
        {-10.147375558811618, -5.463132021660943},
        {-10.12091088252755, -7.345346036804331},
        {-10.088108236425146, -9.188825882202059},
        {-10.05481308411215, -10.969290782563041}
    }, {
        85.6071642996064,
        81.96758702875368,
        78.18717880822521,
        74.27922683855952,
        70.266397971475,
        66.18689833172775,
        62.09610342507143,
        58.04482056253117,
        71.864535057951,
        68.04170035243426,
        63.88313646851508,
        59.41151606138693,
        54.64877769465519,
        49.59991638173192,
        44.186305757875445,
        73.51162401264607,
        73.13336722357417,
        69.12791525321421,
        64.73362495228167,
        59.91439282990217,
        54.60548678718489,
        48.694135128214434,
        41.97884385391046,
        34.05664858424201,
        23.871497227359356,
        0
    }, true);
    Sleep(100);

    gui->pause();

    SD.FClose(posOut);
}