#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
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
    float frequencyScale;
    float steplen;
    float stepspace;
    float dirsetup;
    float dirhold;
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
        float steplen,
        float stepspace,
        float dirsetup,
        float dirhold,
        float dirdelay
    ) : jointNumber(jointNumber)
      , rawCount(0)
      , DDSaddValue(0)
      , DDSaccumulator(0)
      , steplen(steplen)
      , stepspace(stepspace)
      , dirsetup(dirsetup)
      , dirhold(dirhold)
      , dirdelay(dirdelay)
      , stepPin(new PinType(step, OUTPUT))
      , directionPin(new PinType(direction, OUTPUT))
      , rxBuffer(rxBuffer)
      , txBuffer(txBuffer)
    {
        this->frequencyScale = (float)(1 << StepBit) / (float)threadFreq;
    }

    virtual void update()
    {
        this->makePulses();
    }

    virtual void updatePost()
    {
        this->stopPulses();
    }

    virtual void slowUpdate() {}

    void setFrequency(int32_t frequency, bool enabled)
    {
        if (!enabled)
        {
            DDSaddValue = 0;
            return;
        }
        DDSaddValue = frequency * frequencyScale;
    }

    void makePulses()
    {
        rxData_t* rxData = getCurrentRxBuffer(this->rxBuffer);
        bool isEnabled = (rxData->jointEnable & (1 << jointNumber)) != 0;
        int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];
        setFrequency(frequencyCommand, isEnabled);

        int32_t toAdd = DDSaddValue;
        int32_t stepNow = DDSaccumulator;
        DDSaccumulator += toAdd;
        stepNow ^= DDSaccumulator;
        stepNow &= (1L << StepBit);
        if (!stepNow)
            return;

        bool isForward = toAdd > 0;
        this->directionPin->set(isForward);
        this->stepPin->set(true);
        if (isForward)
            ++this->rawCount;
        else
            --this->rawCount;

        txData_t* txData = getCurrentTxBuffer(this->txBuffer);
        txData->jointFeedback[this->jointNumber] = this->rawCount;
    }

    void stopPulses()
    {
        this->stepPin->set(false);
    }
};

#endif
