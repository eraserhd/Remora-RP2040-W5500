#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>
#include <string>

#include "../extern.h"
#include "../module.h"
#include "../../drivers/pin/pin.h"


void createStepgen(void);

template<class Pin>
class BasicStepgen : public Module
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
    BasicStepgen(
        int32_t threadFreq,
        int jointNumber,
        std::string step,
        std::string direction,
        float steplen,
        float stepspace,
        float dirsetup,
        float dirhold,
        float dirdelay
    ) : jointNumber(jointNumber)
      , rawCount(0)
      , DDSaccumulator(0)
      , steplen(steplen)
      , stepspace(stepspace)
      , dirsetup(dirsetup)
      , dirhold(dirhold)
      , dirdelay(dirdelay)
      , stepPin(new Pin(step, OUTPUT))
      , directionPin(new Pin(direction, OUTPUT))
    {
        this->frequencyScale = (float)(1 << STEPBIT) / (float)threadFreq;
    }

    virtual void update(void);     // Module default interface
    virtual void updatePost(void);
    virtual void slowUpdate(void);
    void makePulses();
    void stopPulses();
};

typedef BasicStepgen<Pin> Stepgen;

#endif
