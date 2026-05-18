#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
#include <cmath>
#include <string>

#include "../../remora.h"
#include "../module.h"

enum class PinType
{
    StepPin,
    DirectionPin,
    NoPin
};

template<class IOType>
class BasicStepgen
  : public Module
  , protected IOType
{
    struct CycleCounter
    {
        int32_t remaining;
        int32_t cycles;

        inline CycleCounter(int32_t threadFreq, int32_t ns)
            : remaining(0)
        {
            float nsPerCycle = 1.0 / float(threadFreq) * 1000000000.0;
            cycles = ns ? ceil(ns / nsPerCycle) : 1;
        }

        inline void start()
        {
            remaining = cycles;
        }

        inline bool active() const
        {
            return remaining > 0;
        }

        inline void tick()
        {
            if (remaining)
                --remaining;
        }

        inline bool tickAndExpired()
        {
            if (0 == remaining) return false;
            return 0 == --remaining;
        }
    };

private:
    static constexpr int StepBit = 22;

    int jointNumber;
    volatile int32_t rawCount;
    volatile int32_t DDSaddValue;
    int32_t DDSaccumulator;
    int32_t threadFreq;
    uint32_t cyclesPerTick;
    CycleCounter steplen;
    int32_t maximumFrequency;
    CycleCounter dirsetup;
    CycleCounter dirhold;
    CycleCounter dirdelay;
    bool lastPulseWasForward;
    bool currentDirection;

public:
    BasicStepgen(
        int32_t cpuFreq,
        int32_t threadFreq,
        int jointNumber,
        std::string step,
        std::string direction,
        int32_t steplenNs,
        int32_t stepspaceNs,
        int32_t dirsetupNs,
        int32_t dirholdNs,
        int32_t dirdelayNs
    ) : IOType(step, direction)
      , jointNumber(jointNumber)
      , rawCount(0)
      , DDSaddValue(0)
      , DDSaccumulator(0)
      , threadFreq(threadFreq)
      , cyclesPerTick(cpuFreq / threadFreq)
      , steplen(threadFreq, steplenNs)
      , dirsetup(threadFreq, dirsetupNs)
      , dirhold(threadFreq, dirholdNs)
      , dirdelay(threadFreq, dirdelayNs)
      , lastPulseWasForward(false)
      , currentDirection(false)
    {
        float nsPerCycle = 1.0 / float(threadFreq) * 1000000000.0;
        int32_t stepspaceInCycles = stepspaceNs ? ceil(stepspaceNs / nsPerCycle) : 1;
        maximumFrequency = threadFreq / (steplen.cycles + stepspaceInCycles);

        IOType::schedule(0, PinType::DirectionPin, currentDirection);
        IOType::schedule(cyclesPerTick, PinType::NoPin, false);
    }

    // Callable from core0, owing to DDSaddValue volatility and it being the only
    // data member updated so it can't be inconsistent.
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

    // Callable from core0, owing to rawCount volatility and it being the only
    // data member read so it can't be inconsistent.  Lagging a little bit is OK.
    int32_t getRawCount() const
    {
        return rawCount;
    }

    virtual void update() override
    {
        dirhold.tick();
        if (steplen.tickAndExpired())
        {
            IOType::schedule(0, PinType::StepPin, false);
            IOType::schedule(cyclesPerTick, PinType::NoPin, false);
            dirhold.start();
        }
        dirsetup.tick();
        dirdelay.tick();

        int32_t toAdd = DDSaddValue;
        if (0 == toAdd)
            return;

        bool isForward = toAdd > 0;
        bool needToSwitchDirections = currentDirection != isForward;
        if (needToSwitchDirections && !dirhold.active())
        {
            IOType::schedule(0, PinType::DirectionPin, isForward);
            IOType::schedule(cyclesPerTick, PinType::NoPin, false);
            currentDirection = isForward;
            dirsetup.start();
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
        if (dirsetup.active())
            return;

        // If we still need to switch directions, we're in dirhold so hold off.
        if (needToSwitchDirections)
            return;

        // Hold opposite direction pulse if we are in dirdelay
        if (dirdelay.active() && isForward != lastPulseWasForward)
            return;

        DDSaccumulator = next;
        IOType::schedule(0, PinType::StepPin, true);
        IOType::schedule(cyclesPerTick, PinType::NoPin, false);
        if (isForward)
            ++this->rawCount;
        else
            --this->rawCount;
        steplen.start();
        lastPulseWasForward = isForward;
        dirdelay.start();
    }
};

#endif
