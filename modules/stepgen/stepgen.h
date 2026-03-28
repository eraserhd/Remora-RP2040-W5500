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

    int32_t rawCount;               // current position raw count - not currently used - mirrors original stepgen.c
    int32_t DDSaccumulator;         // Direct Digital Synthesis (DDS) accumulator
    float   frequencyScale;           // frequency scale
    Pin *stepPin, *directionPin;        // class object members - Pin objects

public:
    Stepgen(int32_t, int, std::string, std::string);

    static Stepgen* load(JsonObject module);

    void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand);

    virtual void update(void);           // Module default interface
    virtual void updatePost(void);
};


#endif
