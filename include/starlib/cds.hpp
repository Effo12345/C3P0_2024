#pragma once

#include <FEHIO.h>
#include <starlib/display/interface.hpp>

namespace starlib {

    class Cds {
        std::shared_ptr<AnalogInputPin> cds;
        std::shared_ptr<Interface> gui;

        float ambient;
        int measuredColor;

        float sample(const int numSamples = 10);
    public:
        Cds(const FEHIO::FEHIOPin& pin, const std::shared_ptr<Interface> interface);

        void sampleAmbient();
        void sampleLight(float redThreshold);
        void awaitStartingLight(float threshold);


        int getColor();
        float getOffsetValue();
    };

}