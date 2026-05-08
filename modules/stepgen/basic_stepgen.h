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

    void makePulses()
    {
        rxData_t* rxData = getCurrentRxBuffer(this->rxBuffer);
        bool isEnabled = (rxData->jointEnable & (1 << jointNumber)) != 0;
        if (!isEnabled)
            return;

        int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];
        int32_t DDSaddValue = frequencyCommand * this->frequencyScale;
        int32_t stepNow = this->DDSaccumulator;
        this->DDSaccumulator += DDSaddValue;
        stepNow ^= this->DDSaccumulator;
        stepNow &= (1L << StepBit);

        if (!stepNow)
            return;

        bool isForward = DDSaddValue > 0;
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
