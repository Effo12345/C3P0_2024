#include <starlib/cds.hpp>

namespace starlib {

    Cds::Cds(const FEHIO::FEHIOPin& pin, const std::shared_ptr<Interface> interface) {
        cds = std::make_shared<AnalogInputPin>(pin);
        gui = interface;
    }

    float Cds::sample(const int numSamples) {
        float runningSum {};
        for(int i = 0; i < numSamples; i++) {
            runningSum += cds->Value();
            Sleep(10);
        }
    }

    void Cds::sampleAmbient() {
        const int numSamples = 10;
        ambient = sample(numSamples) / static_cast<float>(numSamples);
    }

    void Cds::sampleLight(float redThreshold) {
        
    }

    void Cds::awaitStartingLight(float threshold) {

    }


    int Cds::getColor() {

    }

}