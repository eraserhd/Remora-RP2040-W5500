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

template<class Pin>
void BasicStepgen<Pin>::update()
{
    // Use the standard Module interface to run makePulses()
    this->makePulses();
}

template<class Pin>
void BasicStepgen<Pin>::updatePost()
{
    this->stopPulses();
}

template<class Pin>
void BasicStepgen<Pin>::slowUpdate()
{
    return;
}

template<class Pin>
void BasicStepgen<Pin>::makePulses()
{
    rxData_t *rxData = getCurrentRxBuffer(&rxPingPongBuffer);
    bool isEnabled = (rxData->jointEnable & (1 << jointNumber)) != 0;
    if (!isEnabled)
        return;

    int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];  // Get the latest frequency command via pointer to the data source
    int32_t DDSaddValue = frequencyCommand * this->frequencyScale;       // Scale the frequency command to get the DDS add value
    int32_t stepNow = this->DDSaccumulator;                              // Save the current DDS accumulator value
    this->DDSaccumulator += DDSaddValue;                                 // Update the DDS accumulator with the new add value
    stepNow ^= this->DDSaccumulator;                                     // Test for changes in the low half of the DDS accumulator
    stepNow &= (1L << STEPBIT);                                          // Check for the step bit

    if (!stepNow)
        return;

    bool isForward = DDSaddValue > 0;
    this->directionPin->set(isForward);                                  // Set direction pin
    this->stepPin->set(true);                                            // Raise step pin
    if (isForward)
    {
        ++this->rawCount;
    }
    else
    {
        --this->rawCount;
    }
    txData_t *txData = getCurrentTxBuffer(&txPingPongBuffer);
    txData->jointFeedback[this->jointNumber] = this->rawCount;
}

template<class Pin>
void BasicStepgen<Pin>::stopPulses()
{
    this->stepPin->set(false);  // Reset step pin
}

typedef BasicStepgen<Pin> Stepgen;

#endif
