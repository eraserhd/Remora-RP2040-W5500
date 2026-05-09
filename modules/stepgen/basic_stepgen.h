#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
#include <cmath>
#include <string>

#include "../../remora.h"
#include "../module.h"

template<class PinType>
class BasicStepgen : public Module
{
private:
    static constexpr int StepBit = 22;

    int jointNumber;
    int32_t rawCount;
    int32_t DDSaddValue;
    int32_t DDSaccumulator;
    int32_t threadFreq;
    int32_t steplenInCycles;
    int32_t steplenCyclesRemaining;
    int32_t maximumFrequency;
    int32_t dirsetupInCycles;
    int32_t dirsetupCyclesRemaining;
    int32_t dirholdInCycles;
    int32_t dirholdCyclesRemaining;
    float dirdelay;
    PinType* stepPin;
    PinType* directionPin;
    RxPingPongBuffer* rxBuffer;
    TxPingPongBuffer* txBuffer;

public:
    BasicStepgen(
        RxPingPongBuffer* rxBuffer,
        TxPingPongBuffer* txBuffer,
        int32_t threadFreq,
        int jointNumber,
        std::string step,
        std::string direction,
        int32_t steplen,
        int32_t stepspace,
        int32_t dirsetup,
        float dirhold,
        float dirdelay
    ) : jointNumber(jointNumber)
      , rawCount(0)
      , DDSaddValue(0)
      , DDSaccumulator(0)
      , threadFreq(threadFreq)
      , steplenCyclesRemaining(0)
      , dirsetupCyclesRemaining(0)
      , dirholdCyclesRemaining(0)
      , dirdelay(dirdelay)
      , stepPin(new PinType(step, OUTPUT))
      , directionPin(new PinType(direction, OUTPUT))
      , rxBuffer(rxBuffer)
      , txBuffer(txBuffer)
    {
        float nsPerCycle = 1.0 / float(threadFreq) * 1000000000.0;
        steplenInCycles = steplen ? ceil(steplen / nsPerCycle) : 1;
        int32_t stepspaceInCycles = stepspace ? ceil(stepspace / nsPerCycle) : 1;

        maximumFrequency = threadFreq / (steplenInCycles + stepspaceInCycles);
        dirsetupInCycles = dirsetup ? ceil(dirsetup / nsPerCycle) : 1;
        dirholdInCycles = dirhold ? ceil(dirhold / nsPerCycle) : 1;
    }

    virtual void update() override { this->makePulses(); }
    virtual void updatePost() override {}
    virtual void slowUpdate() override {}

    void setFrequency(int32_t frequency, bool enabled)
    {
        if (!enabled)
        {
            DDSaddValue = 0;
            return;
        }
        if (frequency > 0)
            frequency = std::min(frequency, maximumFrequency);
        else
            frequency = std::max(frequency, -maximumFrequency);
        DDSaddValue = frequency * ((float)(1 << StepBit) / (float)threadFreq);
    }

    void makePulses()
    {
        if (dirholdCyclesRemaining)
            --dirholdCyclesRemaining;
        if (steplenCyclesRemaining)
        {
            if (0 == --steplenCyclesRemaining)
            {
                this->stepPin->set(false);
                dirholdCyclesRemaining = dirholdInCycles;
            }
        }
        if (dirsetupCyclesRemaining)
            --dirsetupCyclesRemaining;

        rxData_t* rxData = getCurrentRxBuffer(this->rxBuffer);
        bool isEnabled = (rxData->jointEnable & (1 << jointNumber)) != 0;
        int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];
        setFrequency(frequencyCommand, isEnabled);

        int32_t toAdd = DDSaddValue;
        if (0 == toAdd)
            return;

        bool isForward = toAdd > 0;
        if (this->directionPin->get() != isForward && 0 == dirholdCyclesRemaining)
        {
            this->directionPin->set(isForward);
            dirsetupCyclesRemaining = dirsetupInCycles;
        }

        int32_t next = DDSaccumulator + toAdd;
        bool timeToStep = (next ^ DDSaccumulator) & (1 << StepBit);
        if (!timeToStep)
        {
            DDSaccumulator = next;
            return;
        }

        // Hold off on stepping if we're still in dirsetup, but don't update
        // the accumulator so we step immediately after dirstep
        if (dirsetupCyclesRemaining > 0)
            return;
        if (dirholdCyclesRemaining > 0)
            return;

        DDSaccumulator = next;
        this->stepPin->set(true);
        steplenCyclesRemaining = steplenInCycles;
        if (isForward)
            ++this->rawCount;
        else
            --this->rawCount;

        txData_t* txData = getCurrentTxBuffer(this->txBuffer);
        txData->jointFeedback[this->jointNumber] = this->rawCount;
    }
};

#endif
