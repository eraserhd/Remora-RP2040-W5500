#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>
#include <string>

#include "../extern.h"
#include "../module.h"
#include "../../drivers/pin/pin.h"

class Stepgen : public Module
{
private:
    int jointNumber;                // LinuxCNC joint number
    int mask;

    volatile int32_t stepperPosition;
    volatile int32_t DDSaddValue;
    int32_t DDSaccumulator;         // Direct Digital Synthesis (DDS) accumulator

    Pin *stepPin, *directionPin;        // class object members - Pin objects

public:
    Stepgen(int32_t, int, std::string, std::string);

    static Stepgen* load(JsonObject module);

    void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand);
    inline int32_t jointFeedback() const { return stepperPosition; }

    virtual void update(void);           // Module default interface
    virtual void updatePost(void);
};


#endif
