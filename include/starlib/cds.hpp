#pragma once

#include <FEHIO.h>
#include <memory>

namespace starlib {

    class Cds {
        std::shared_ptr<AnalogInputPin> cell;

        float ambient {};
        int measuredColor = BLACK;

        float sample(const int numSamples = 10);
    public:
        Cds(const FEHIO::FEHIOPin& pin);

        void sampleAmbient();
        void sampleLight(float redThreshold);
        void awaitStartingLight(float threshold, float timeout = 30.0f);


        int getColor();
        float getOffsetValue();
        float getAmbientSample();
    };

}