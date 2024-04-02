#include <starlib/cds.hpp>

namespace starlib {

    Cds::Cds(const FEHIO::FEHIOPin& pin) {
        cell = std::make_shared<AnalogInputPin>(pin);
    }

    float Cds::sample(const int numSamples) {
        float runningSum {};
        for(int i = 0; i < numSamples; i++) {
            runningSum += cell->Value();
            Sleep(10);
        }

        return runningSum / static_cast<float>(numSamples);
    }

    void Cds::sampleAmbient() {
        const int numSamples = 10;
        ambient = sample(numSamples);
    }

    void Cds::sampleLight(float redThreshold) {
        const int numSamples = 10;
        float lightLevel = ambient - sample(numSamples);

        if(lightLevel > redThreshold) {
            measuredColor = RED;
        }
        else {
            measuredColor = BLUE;
        }
    }

    void Cds::awaitStartingLight(float threshold, float timeout) {
        float startTime = TimeNow();
        while(getOffsetValue() < threshold && TimeNow() - startTime < timeout) {
            Sleep(50);
        }
    }


    int Cds::getColor() {
        return measuredColor;
    }

    float Cds::getOffsetValue() {
        return ambient - cell->Value();
    }

    float Cds::getAmbientSample() {
        return ambient;
    }

}