#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>
#include <string>

#include "../extern.h"
#include "../module.h"
#include "../../drivers/pin/pin.h"


void createStepgen(void);

class Stepgen : public Module
{
private:
    int jointNumber;               // LinuxCNC joint number

    int32_t rawCount;              // current position raw count - not currently used - mirrors original stepgen.c
    int32_t DDSaccumulator;        // Direct Digital Synthesis (DDS) accumulator
    float   frequencyScale;        // frequency scale

    float steplen;
    float stepspace;
    float dirsetup;
    float dirhold;
    float dirdelay;

    Pin *stepPin;
    Pin *directionPin;

public:
    Stepgen(
        int32_t threadFreq,
        int jointNumber,
        std::string step,
        std::string direction,
        float steplen,
        float stepspace,
        float dirsetup,
        float dirhold,
        float dirdelay
    );

    virtual void update(void);     // Module default interface
    virtual void updatePost(void);
    virtual void slowUpdate(void);
    void makePulses();
    void stopPulses();
};


#endif
