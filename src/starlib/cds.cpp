#include <starlib/cds.hpp>

namespace starlib {

    Cds::Cds(const FEHIO::FEHIOPin& pin, float redThreshold, float startThreshold,
             float startTimeout) 
    : AnalogInputPin(pin), redMin{redThreshold}, startMin{startThreshold},
      timeout{startTimeout} {}

    float Cds::sample(const int numSamples) {
        float runningSum {};
        for(int i = 0; i < numSamples; i++) {
            runningSum += Value();
            Sleep(10);
        }

        return runningSum / static_cast<float>(numSamples);
    }

    void Cds::sampleAmbient() {
        const int numSamples = 10;
        ambient = sample(numSamples);
    }

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

    void Cds::awaitStartingLight() {
        float startTime = TimeNow();
        while(getOffsetValue() < startMin && TimeNow() - startTime < timeout) {
            Sleep(50);
        }
    }


    int Cds::getColor() {
        return measuredColor;
    }

    float Cds::getOffsetValue() {
        return ambient - Value();
    }

    float Cds::getAmbientSample() {
        return ambient;
    }

}