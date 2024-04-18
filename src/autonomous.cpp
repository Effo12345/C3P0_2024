#include <autonomous.hpp>
#include <FEHRCS.h>

void fuelLever(int leverNum) {
     if(leverNum == 0) {
        // Route for first lever
        chassis->followNewPath({
            {11.58859813084112, -29.896364380693885},
        {10.261634262105131, -28.491574541881096},
        {8.933236580840848, -27.101390123192584},
        {7.603682962508183, -25.74266518802335},
        {6.272413776919699, -24.437214565806986},
        {4.938208958628012, -23.21515130206926},
        {3.5998225867650437, -22.119533258048374},
        {2.254983950728121, -21.21312642690189},
        {0.8231482115679349, -20.500736152316236},
        {-0.6130321137903445, -19.931565064274324},
        {-2.0520079185789664, -19.461414449085442},
        {-3.408411214953271, -19.044842184432202}
        },{
            77.68322997886798,
        73.59557132718669,
        69.28928024748056,
        64.75091155133218,
        59.96691053287555,
        54.927739827728004,
        49.634378972869186,
        44.09764817607752,
        37.85283714241815,
        30.634693364059213,
        21.30862738509119,
        0
        }, true);
    }
    else if(leverNum == 1) {
        // Route for second lever (primary points only)
        chassis->followNewPath({
            {11.673808411214953, -29.81224405359108},
        {10.327995764563068, -28.419625530710135},
        {8.980951941063609, -27.040547149530457},
        {7.633575453000745, -25.69077522518467},
        {6.285405025576117, -24.390614383870847},
        {4.9360716525774695, -23.168189219915117},
        {3.5849156161049596, -22.063519189326616},
        {2.231076106930523, -21.135165862265897},
        {0.873322215091092, -20.469201478473906},
        {-0.6541134950873094, -20.01238074573874},
        {-2.2775237762623974, -19.69339296229711},
        {-3.9381355098986717, -19.462495771183605},
        {-5.583026768976583, -19.283644138829818},
        {-7.1576635514018685, -19.128962511535004}
        },{
            84.90280760504325,
        81.17117711579773,
        77.27785172828105,
        73.22274993956546,
        69.00886675357788,
        64.64977238660857,
        60.17566907899617,
        55.64001928550644,
        51.10656248584465,
        45.844405242637315,
        39.65204393666063,
        32.18348435035616,
        22.501313586024068,
        0
        }, true);
    }
    else {
        // Route for third lever
        chassis->followNewPath({
            {11.247757009345793, -29.55988307228267},
        {9.964717650315574, -28.115501151352188},
        {8.679481300543516, -26.686060130264902},
        {7.391666998385875, -25.28874967702106},
        {6.099417643931064, -23.945788210385693},
        {4.800229080206282, -22.687970803364117},
        {3.490435926485245, -21.559314388827644},
        {2.1649553248165434, -20.624049731617134},
        {0.8159811501566608, -19.976496983809014},
        {-0.7249768235625066, -19.56622274730489},
        {-2.3865667119121925, -19.31503607970419},
        {-4.116338689753883, -19.16957054340905},
        {-5.8724209033719035, -19.09293464793447},
        {-7.617093990342953, -19.05896740167948},
        {-9.31073095828102, -19.048230431957734},
        {-10.906915887850467, -19.044842184432202}
        }, {
            88.93115875594637,
        85.38458371486506,
        81.70311942925358,
        77.89300602883557,
        73.96573426927631,
        69.9447618995018,
        65.87102052620243,
        61.80516860613887,
        57.80179635350825,
        57.156524554144184,
        52.240963366612284,
        46.622284329363154,
        40.139210711216414,
        32.44618696767433,
        22.600449735059794,
        0
        }, true);
    }

    if(leverNum == 0 || leverNum == 1) {
        // Align with lever
        chassis->turn(89.0f);

        float flipDownTime = TimeNow();
        // Actuate lever, then move arm up
        fuelArm.SetDegree(100);
        Sleep(1000);
        fuelArm.SetDegree(0);

        // Disengage from lever
        chassis->driveFor(-15.0f, 0.5f);

        // Put arm under lever
        fuelArm.SetDegree(150);
        Sleep(500);

        // Realign with lever
        chassis->driveFor(15.0f, 0.5f);

        // Wait until 5 seconds elapsed, then unflip lever
        Sleep(5.25f - (TimeNow() - flipDownTime));
        fuelArm.SetDegree(85);
        Sleep(300);

        // Get arm out of the way
        fuelArm.SetDegree(155);
        Sleep(100);

        // Line up with wall
        chassis->followNewPath({
            {-4.601355140186915, -18.203638913404163},
        {-6.542553139027724, -18.21816432774457},
        {-8.475465843640093, -18.23262029872955},
        {-10.388685199720005, -18.246928986045525},
        {-12.268982034528465, -18.260991450647396},
        {-14.097871260956413, -18.274669445276384},
        {-15.84911214953271, -18.28775924050697}
        }, {
            59.994857291017624,
        54.57272351347889,
        48.576051591192716,
        41.80174006511954,
        33.847801378972115,
        23.673038221726866,
        0
        }, true);
    }
    else { // Lever 3 (same as above)
        chassis->turn(89.0f);

        float flipDownTime = TimeNow();
        fuelArm.SetDegree(100);
        Sleep(1000);
        fuelArm.SetDegree(0);

        float startLeverFlip = TimeNow();

        chassis->driveFor(15.0f, 0.6f);

        fuelArm.SetDegree(150);
        Sleep(500);

        chassis->driveFor(-25.0f, 0.5f);

        Sleep(5.25f - (TimeNow() - flipDownTime));
        fuelArm.SetDegree(75);

        // Get arm out of the way of the levers
        Sleep(500);
        fuelArm.SetDegree(100);
        Sleep(50);
    }

    // Hard reset against wall
    chassis->getOdomModel()->setPos({{-10.65, FLT_MAX}, 90.0f});
}

void ramp() {
    // Drive to ramp
    chassis->followNewPath({
        {-15.678691588785046, -18.876601530226594},
        {-13.698875021554342, -18.91246736276308},
        {-11.723332954377499, -18.94674302094277},
        {-9.75483576731978, -18.977598455301877},
        {-7.798146692646822, -19.002639117755947},
        {-5.859893660084502, -19.018550259096155},
        {-3.949625325465347, -19.020607044902118},
        {-2.081347131721959, -19.00194331974281},
        {-0.27543279638341617, -18.952449431855445},
        {1.4377451762396969, -18.85722485961777},
        {3.0136764265968363, -18.69417877902899},
        {4.660870975296372, -18.437013059988615},
        {6.333978499053926, -18.123206567510323},
        {7.991881695821776, -17.78136826702066},
        {9.590901805421051, -17.435768419154662},
        {11.07733644859813, -17.110074661067713}
    }, {
        92.82264147666051,
        89.34426405747072,
        85.73289284981347,
        81.97641391135608,
        78.06369618999877,
        73.98431451610477,
        69.73085811183812,
        65.3032443318028,
        60.71576517337116,
        56.01197170927781,
        51.28694917149402,
        45.79152365297836,
        39.397125084066744,
        31.78749112782635,
        22.066792125559836,
        0
    });

    // Get servo out of the way, and face up the ramp
    fuelArm.SetDegree(0.0f);

    chassis->turn(0.0f);
    // gui->pause();

    // Facing exactly straight up the ramp
    chassis->getOdomModel()->setPos({{FLT_MAX, FLT_MAX}, 0.0f});

    passportArm.SetDegree(60.0f);

    // Drive to the top of the ramp
    chassis->resetSettled();
    chassis->drive(80.0f, 80.0f);
    while(chassis->getOdomModel()->getPos().p.y < 6.0f) {
        chassis->getOdomModel()->step();
        gui->setPos(chassis->getOdomModel()->getPos());
        Sleep(10);
    }
    chassis->drive(0.0f, 0.0f);
    chassis->awaitSettled();

    // gui->pause();
    chassis->getOdomModel()->setPos({{FLT_MAX, 7.45f}, 0.0f});

    chassis->followNewPath({
        {11.58859813084112, 8.210143796876213},
        {9.908909095269006, 8.328215052310563},
        {8.276037247860241, 8.426572588816413},
        {6.6575734911592575, 8.493105880490905},
        {5.1126168224299064, 8.546625105287427}
 
    }, {
        45.55569549404768,
        39.198145285872094,
        31.828108896697696,
        22.241463096664337,
        0
    });

    chassis->turn(-90.0f);

    chassis->followNewPath({
        {5.623878504672897, 11.49083655388556},
        {7.321734497071307, 11.452314846268939},
        {8.970933437001593, 11.413286566152252},
        {10.518668896577896, 11.373143379496334},
        {12.085253858296184, 11.327769185853722},
        {13.603161703200582, 11.281494073011615},
        {14.997009345794392, 11.238475572577148}
    }, {
        77.46617692247663,
        70.10064970184126,
        62.11536247314622,
        53.5484307760901,
        43.178693876686836,
        29.874525092666733,
        0
    }, true);

    

    // Hard reset at top of ramp
    chassis->getOdomModel()->setPos({{10.75f, FLT_MAX}, -90.0f});
}

void light() {
    // Drive to light
    chassis->followNewPath({
        {15.082219626168223, 10.565512955754718},
        {13.189359112769221, 10.63224639500929},
        {11.311705145567851, 10.725052116954085},
        {9.468831066560712, 10.874260694521448},
        {7.685027952064008, 11.118719622630827},
        {5.9935574834547545, 11.512424721235734},
        {4.442114002986909, 12.132629661205382},
        {3.051926745547541, 13.089303188065214},
        {1.771459189578323, 14.267097555014626},
        {0.5663731564045801, 15.585266980166635},
        {-0.5858608741825532, 16.984882469899294},
        {-1.6995405119022775, 18.41969315007208},
        {-2.7829646633763003, 19.849004053789088},
        {-3.839719745452008, 21.231223726318728},
        {-4.869214819859706, 22.517482192349785},
        {-5.967290700057634, 23.80478339420836},
        {-7.085657535166577, 25.07226780914759},
        {-8.18018691588785, 26.296014123979017}

    }, {
        79.24432290543687,
        75.32311087406752,
        71.21789298439758,
        66.9353331361619,
        62.48348991233368,
        57.86576877042477,
        53.04512798390488,
        74.53339853890364,
        70.70008038409894,
        66.5355599731656,
        62.0230299044117,
        57.14576645311728,
        51.88169351161282,
        46.20542343202363,
        40.09656931886647,
        32.65409692972953,
        22.921172067483063,
        0
    });

    // Sample light color using cds cell
    cds->sampleLight();
}

void luggage() {
    // Drive to luggage
    chassis->followNewPath({
        {-8.265397196261683, 26.127773469773413},
        {-7.032410619575566, 25.089793012483334},
        {-5.88725322364639, 24.029674676012025},
        {-4.929264221011782, 22.921805706223097},
        {-4.287920131703484, 21.73309409460841},
        {-3.890342329086113, 20.291792874274382},
        {-3.6560542989407168, 18.68452691006081},
        {-3.530097803848733, 16.972061726825007},
        {-3.4743811372002638, 15.198691774101086},
        {-3.4618082133422496, 13.399263545257641},
        {-3.4719135376119334, 11.604517310799636},
        {-3.492098375579581, 9.774994991602082},
        {-3.5183912917882085, 7.9372220630965105},
        {-3.5478126066516724, 6.116491631309103},
        {-3.5788317757009342, 4.340608750147241}
    }, {
        63.83370569284735,
        58.01364261647306,
        51.75859350171615,
        45.10563683635887,
        44.11238208408953,
        83.83795508522063,
        79.46151147321613,
        74.55609764733894,
        69.1226727955311,
        63.13617146381743,
        56.53737955081649,
        48.90231563383388,
        39.78358496108396,
        27.95545843912436,
        0
    }, true);
}

void button() {
    if(cds->getColor() == RED) {
        // Press red button (primary points only)
        chassis->followNewPath({
            {-3.749252336448598, 2.7423225351939693},
        {-2.8229960994136265, 4.298501594601106},
        {-1.9072169125559553, 5.826023675999866},
        {-1.0119029869053173, 7.291603112843903},
        {-0.1512513981934448, 8.652624945905446},
        {0.6551138307947585, 9.849705149899387},
        {1.523689134507458, 11.053291325698307},
        {2.32926425609209, 12.279637610661487},
        {2.9368143825832, 13.548431366994386},
        {3.404133902608845, 15.033328420197975},
        {3.7550400127612007, 16.657804586757806},
        {3.9952706382917853, 18.36718383404862},
        {4.114159324521835, 20.125729938933524},
        {4.153493018978708, 21.89653804184577},
        {4.14273253193273, 23.644797945544173},
        {4.10343939043185, 25.33206993022894},
        {4.052711240096367, 26.910410706213657},
        {4.004883177570093, 28.314901974446308}
        }, {
            94.0135143676026,
        90.8791928723005,
        87.6875499651871,
        84.49576325169805,
        81.38940029562819,
        78.5007399627603,
        75.41486165518022,
        72.23486331512983,
        69.04863816855713,
        65.34195639262495,
        61.13713421437876,
        56.43910377816638,
        51.19915164486063,
        45.32719672406643,
        38.66653587927561,
        30.903520389128236,
        21.20607756258975,
        0
        });
    }
    else {
        // Press blue button (default)
        chassis->followNewPath({
            {-3.0675700934579435, 0.891675338932287},
        {-3.091694027589364, 2.8015868503728854},
        {-3.1054396347834574, 4.697859792135125},
        {-3.0946971768349143, 6.564600592805611},
        {-3.0425205202817778, 8.381377021836236},
        {-2.9254743099693425, 10.12006866144757},
        {-2.7100232296402407, 11.740191882866965},
        {-2.3591289743495376, 13.425527586040582},
        {-1.9468121307413822, 15.1546999119488},
        {-1.5374893784399923, 16.913226146769478},
        {-1.1960091592351123, 18.69124334208409},
        {-0.9881595254150563, 20.503368343614685},
        {-0.8688470246274554, 22.320510615404196},
        {-0.8068131249255397, 24.114380259546834},
        {-0.7797465702544012, 25.853054738170695},
        {-0.7707848062601355, 27.496061414081172},
        {-0.7668925233644859, 28.98786459126874}
        }, {
            95.1573018238276,
        91.88956227716034,
        88.52608865289316,
        85.08525973972787,
        81.595914482738,
        78.10411422955175,
        74.68098190942273,
        70.89694214197749,
        66.76472251116088,
        62.287693056743834,
        57.449045683979485,
        52.122076676480994,
        46.19494842372606,
        39.49166804142144,
        31.672521195700412,
        21.84899635136888,
        0
        });
    }
}

void passport() {
    // Try to press the top button (NOT FUNCTIONAL)
    // highButton.SetDegree(70);
    // Sleep(1000);
    // highButton.SetDegree(0);

    // Go to passport lever
    // fuelArm.SetDegree(170);
    passportArm.SetDegree(180);

    chassis->followNewPath({
        {1.7042056074766354, 26.54837510528743},
        {1.7138533186656226, 24.75072013987888},
        {1.7493848384486137, 22.98352299149517},
        {1.8427746419084852, 21.2821941030158},
        {2.034001051127976, 19.692090436742173},
        {2.3782248595118887, 18.276441300312822},
        {2.9338867460299283, 16.869539725617276},
        {3.5924885774555477, 15.495203387834},
        {4.260514018691588, 14.18268702117528} 
    }, {
        64.092366924167,
        59.435457638085126,
        54.46977367697463,
        49.210886971526406,
        43.69452962610751,
        37.986898788787755,
        30.966984622872655,
        21.7088958380032,
        0
    }, true);

    chassis->turn(0.0f);

    chassis->followNewPath({
        {4.856985981308411, 14.603288656689298},
        {4.904900411160484, 16.455861922583583},
        {4.951236860963787, 18.285611711992114},
        {4.996329192098817, 20.066233312233454},
        {5.039308534097578, 21.763416465128785},
        {5.078970389130134, 23.329597542907365},
        {5.1126168224299064, 24.697727909025748}
    }, {
        56.84414103612772,
        51.363748684841035,
        45.30482346396909,
        38.50383669095963,
        30.647556411474053,
        20.926874870257436,
        0
    });

    // chassis->turn(-3.0f);

    // passportArm.SetDegree(90);
    // Sleep(1000);
    // passportArm.SetDegree(0);
    // Sleep(500);
    // passportArm.SetDegree(180);
    // chassis->driveFor(15.0f, 0.2f);
    // passportArm.SetDegree(90);
    // Sleep(100);

    chassis->turn(0.0f);
    passportArm.SetDegree(55);
    Sleep(500);
    passportArm.SetDegree(180);

    chassis->driveFor(25.0f, 0.15f);
    fuelArm.SetDegree(40.0f);

    chassis->driveFor(-25.0f, 0.5f);
    fuelArm.SetDegree(0.0f);
    passportArm.SetDegree(60.0f);

    // chassis->turn(0.0f);

    // Move passport lever up
    // chassis->driveFor(25.0f, 0.8f);
    // fuelArm.SetDegree(30.0f);
    // Sleep(1000);

    // fuelArm.SetDegree(170);

    // chassis->driveFor(15.0f, 1.7f);

    // fuelArm.SetDegree(20.0f);
    // chassis->driveFor(25.0f, 0.5f);

    // fuelArm.SetDegree(20.0f);

    // Move passport lever down
    // chassis->driveFor(-15.0f, 0.75f);
    // fuelArm.SetDegree(0.0f);
    // Sleep(500);

    // chassis->driveFor(15.0f, 1.25f);
    
    // fuelArm.SetDegree(20.0f);
    // Sleep(500);

    // chassis->driveFor(-15.0f, 1.0f);
    
    // fuelArm.SetDegree(0.0f);

    // gui->pause();
}

void finalButton() {
    // Path to final button
    chassis->followNewPath({
        {4.77177570093458, 19.902869264165933},
        {4.848821252133011, 18.068360958721772},
        {4.949357975345889, 16.25944102732709},
        {5.10271141843916, 14.505662119714376},
        {5.345482733859882, 12.845205755425832},
        {5.728241255320996, 11.330780985069175},
        {6.323319460676121, 10.037918191227345},
        {7.226870006850864, 8.846504262270706},
        {8.295384785120406, 7.698990944842439},
        {9.411130259592914, 6.544704181099443},
        {10.46373797386768, 5.331892405086219},
        {11.333011106501512, 3.999684499924848},
        {11.896264864906078, 2.456731809867296},
        {12.251033608306521, 0.7742029258031378},
        {12.462335458778378, -0.9985025134459854},
        {12.57280810730008, -2.8260582770181126},
        {12.609355495206678, -4.681701454954301},
        {12.587343517694835, -6.543050297843913},
        {12.512991608569042, -8.388612602690475},
        {12.384341427916638, -10.194428384941926},
        {12.190949374167218, -11.930354597083934},
        {11.912285141525713, -13.555305690879768},
        {11.514488638637044, -15.010927837655466},
        {10.945153031385315, -16.21238731568607},
        {10.115657546922362, -17.339387333202723},
        {9.177881175497282, -18.43260387426818},
        {8.266799149743973, -19.52745459090485},
        {7.521555675746558, -20.65957757890552},
        {6.992526562511182, -21.925548543312267},
        {6.728762551868992, -23.23991647677771},
        {6.8206754207509075, -24.524773413182608},
        {7.391784585130038, -25.74587257827295},
        {8.27176467811867, -26.920277088437977},
        {9.338478779904836, -28.05775701933768},
        {10.498937632049273, -29.162326084137575},
        {11.673808411214953, -30.232845689105098}
    }, {
        79.07912474388175,
        74.2903613025482,
        69.24141202506331,
        63.95455249949406,
        58.47170919521697,
        52.85946021644345,
        47.16806662943345,
        70.75917960270193,
        66.17912024764992,
        61.13527088752014,
        55.63419598870485,
        49.5870225030883,
        82.4820593389124,
        96.74358815267595,
        92.97967581151153,
        88.95427729758347,
        84.67858188589493,
        80.16152838445001,
        75.41251138169942,
        70.44778078248666,
        65.30102315441455,
        60.03961001407822,
        54.782800061562895,
        49.6924893718404,
        62.71304960725357,
        57.93785539879034,
        52.79257241567335,
        47.3803597060878,
        41.183414908178385,
        34.05649247792154,
        25.388749023457052,
        48.13793583259562,
        43.59335785308978,
        35.72981124235239,
        25.21463402506015,
        0
    }, true);

    // In case the we miss, turn then back up into final button
    chassis->turn(-45.0f);
    chassis->driveFor(-50.0f, 2.0f);
}

/**
 * Final route for the in-class performance test
*/
void showcase() {
    // Fetch lever from RCS and give to GUI
    // int leverNum = RCS.GetCorrectLever();
    int leverNum = 1;
    gui->setLeverNum(leverNum);

    // Set initial states for servos and press starting button
    fuelArm.SetDegree(0.0f);
    passportArm.SetDegree(148);

    chassis->driveFor(25.0f, 0.2f);

    
    fuelLever(leverNum);
    ramp();
    light();
    luggage();
    button();
    passport();
    finalButton();
}