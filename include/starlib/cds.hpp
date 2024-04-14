#pragma once

#include <FEHIO.h>
#include <memory>
#include <FEHUtility.h>
#include <FEHLCD.h>

namespace starlib {

    class Cds : AnalogInputPin {

        float ambient {};
        int measuredColor = BLACK;

        float redMin {};
        float startMin {};
        float timeout {};

        float sample(const int numSamples = 10);
    public:
        Cds(const FEHIO::FEHIOPin& pin, float redThreshold, float startThreshold,
            float startTimeout = 30.0f);

        void sampleAmbient();
        void sampleLight();
        void awaitStartingLight();


        int getColor();
        float getOffsetValue();
        float getAmbientSample();
    };

} // namespace starlib