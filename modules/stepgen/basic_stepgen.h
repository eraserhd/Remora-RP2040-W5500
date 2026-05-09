#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
#include <cmath>
#include <string>

#include "../../remora.h"
#include "../module.h"

struct CycleCounter
{
    int32_t remaining;
    int32_t cycles;

public:
    inline CycleCounter(int32_t threadFreq, int32_t ns)
        : remaining(0)
    {
        float nsPerCycle = 1.0 / float(threadFreq) * 1000000000.0;
        cycles = ns ? ceil(ns / nsPerCycle) : 1;
    }
};

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
    CycleCounter steplen;
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
        int32_t steplenNs,
        int32_t stepspaceNs,
        int32_t dirsetupNs,
        int32_t dirholdNs,
        float dirdelay
    ) : jointNumber(jointNumber)
      , rawCount(0)
      , DDSaddValue(0)
      , DDSaccumulator(0)
      , threadFreq(threadFreq)
      , steplen(threadFreq, steplenNs)
      , dirsetupCyclesRemaining(0)
      , dirholdCyclesRemaining(0)
      , dirdelay(dirdelay)
      , stepPin(new PinType(step, OUTPUT))
      , directionPin(new PinType(direction, OUTPUT))
      , rxBuffer(rxBuffer)
      , txBuffer(txBuffer)
    {
        float nsPerCycle = 1.0 / float(threadFreq) * 1000000000.0;
        int32_t stepspaceInCycles = stepspaceNs ? ceil(stepspaceNs / nsPerCycle) : 1;

        maximumFrequency = threadFreq / (steplen.cycles + stepspaceInCycles);
        dirsetupInCycles = dirsetupNs ? ceil(dirsetupNs / nsPerCycle) : 1;
        dirholdInCycles = dirholdNs ? ceil(dirholdNs / nsPerCycle) : 1;
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
        if (steplen.remaining)
        {
            if (0 == --steplen.remaining)
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
        bool needToSwitchDirections = this->directionPin->get() != isForward;
        if (needToSwitchDirections && 0 == dirholdCyclesRemaining)
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
        // the accumulator so we step immediately after dirstep.
        if (dirsetupCyclesRemaining > 0)
            return;

        // If we still need to switch directions, we're in dirhold so hold off.
        if (needToSwitchDirections)
            return;

        DDSaccumulator = next;
        this->stepPin->set(true);
        steplen.remaining = steplen.cycles;
        if (isForward)
            ++this->rawCount;
        else
            --this->rawCount;

        txData_t* txData = getCurrentTxBuffer(this->txBuffer);
        txData->jointFeedback[this->jointNumber] = this->rawCount;
    }
};

#endif
