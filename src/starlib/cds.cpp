#include <starlib/cds.hpp>

namespace starlib {

Cds::Cds(const FEHIO::FEHIOPin& pin, float redThreshold, float startThreshold,
            float startTimeout) 
: AnalogInputPin(pin), redMin{redThreshold}, startMin{startThreshold},
    timeout{startTimeout} {}

/**
 * Sample the light value numSamples times with 10 ms between each sample to
 * ensure an accurate reading
*/
float Cds::sample(const int numSamples) {
    // Average over numSamples
    float runningSum {};
    for(int i = 0; i < numSamples; i++) {
        runningSum += Value();
        Sleep(10);
    }

    return runningSum / static_cast<float>(numSamples);
}

/**
 * Sample the ambient light level at the start of the run
*/
void Cds::sampleAmbient() {
    const int numSamples = 10;
    ambient = sample(numSamples);
}

/**
 * Sample the ticket kiosk light. Use thresholds (offset from abient) to
 * determine light color
*/
void Cds::sampleLight() {
    const int numSamples = 10;
    float lightLevel = ambient - sample(numSamples);

    if(lightLevel > redMin) {
        measuredColor = RED;
    }
    else {
        measuredColor = BLUE;
    }
}

/**
 * Hold control until the cds measures above the start threshold or times out
*/
void Cds::awaitStartingLight() {
    float startTime = TimeNow();
    while(getOffsetValue() < startMin && TimeNow() - startTime < timeout) {
        Sleep(50);
    }
}

/*
    * Start trivial getters
*/

int Cds::getColor() {
    return measuredColor;
}

float Cds::getOffsetValue() {
    return ambient - Value();
}

float Cds::getAmbientSample() {
    return ambient;
}

} // namespace starlib