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

template<
    int32_t CpuFreq
  , int32_t ThreadFreq
  , class IOType
>
class BasicStepgen
  : public Module
  , protected IOType
{
    static_assert(CpuFreq % ThreadFreq == 0, "CpuFreq must be an integer multiple of ThreadFreq");
    static constexpr uint32_t cyclesPerTick = CpuFreq / ThreadFreq;

    struct CycleCounter
    {
        int32_t remaining;
        int32_t cycles;

        inline CycleCounter(int32_t ns)
            : remaining(0)
        {
            float nsPerCycle = 1.0 / float(ThreadFreq) * 1000000000.0;
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
    CycleCounter steplen;
    int32_t maximumFrequency;
    CycleCounter dirsetup;
    CycleCounter dirhold;
    CycleCounter dirdelay;
    bool lastPulseWasForward;
    bool currentDirection;

public:
    BasicStepgen(
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
      , steplen(steplenNs)
      , dirsetup(dirsetupNs)
      , dirhold(dirholdNs)
      , dirdelay(dirdelayNs)
      , lastPulseWasForward(false)
      , currentDirection(false)
    {
        float nsPerCycle = 1.0 / float(ThreadFreq) * 1000000000.0;
        int32_t stepspaceInCycles = stepspaceNs ? ceil(stepspaceNs / nsPerCycle) : 1;
        maximumFrequency = ThreadFreq / (steplen.cycles + stepspaceInCycles);

        IOType::schedule(0, PinType::DirectionPin, currentDirection);
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
        DDSaddValue = frequency * ((float)(1 << StepBit) / (float)ThreadFreq);
    }

    // Callable from core0, owing to rawCount volatility and it being the only
    // data member read so it can't be inconsistent.  Lagging a little bit is OK.
    int32_t getRawCount() const
    {
        return rawCount;
    }

    virtual void update() override
    {
        changePins();
        IOType::schedule(cyclesPerTick, PinType::NoPin, false);
    }

private:
    void changePins()
    {
        dirhold.tick();
        if (steplen.tickAndExpired())
        {
            IOType::schedule(0, PinType::StepPin, false);
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
