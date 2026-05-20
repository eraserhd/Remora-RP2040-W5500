#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
#include <cmath>
#include <string>

#include "../../remora.h"
#include "../module.h"

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

    static constexpr uint32_t nsToCycles(int32_t ns)
    {
        return ns > 0
            ? uint32_t((uint64_t(ns) * uint64_t(CpuFreq) + 999999999ULL) / 1000000000ULL)
            : cyclesPerTick;
    }

    struct CycleCounter
    {
        uint32_t durationCycles;
        uint32_t expiryCycle;
        bool armed;

        inline CycleCounter(int32_t ns)
            : durationCycles(nsToCycles(ns))
            , expiryCycle(0)
            , armed(false)
        {
        }

        inline void start(uint32_t now)
        {
            expiryCycle = now + durationCycles;
            armed = true;
        }

        // Disarm if expired, so we don't spuriously show armed on next cycle.
        inline void update(uint32_t now)
        {
            if (armed && int32_t(now - expiryCycle) >= 0)
                armed = false;
        }

        inline bool active(uint32_t now) const
        {
            return armed && int32_t(now - expiryCycle) < 0;
        }

        inline bool expired(uint32_t now)
        {
            if (armed && int32_t(now - expiryCycle) >= 0)
            {
                armed = false;
                return true;
            }
            return false;
        }
    };

private:
    static constexpr int StepBit = 22;

    int jointNumber;
    volatile int32_t rawCount;
    volatile int32_t DDSaddValue;
    int32_t DDSaccumulator;
    uint32_t nowCycles;
    CycleCounter steplen;
    int32_t maximumFrequency;
    CycleCounter dirsetup;
    CycleCounter dirhold;
    CycleCounter dirdelay;
    bool lastPulseWasForward;
    bool currentDirection;
    bool currentStep;

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
      , nowCycles(0)
      , steplen(steplenNs)
      , maximumFrequency(CpuFreq / (steplen.durationCycles + nsToCycles(stepspaceNs)))
      , dirsetup(dirsetupNs)
      , dirhold(dirholdNs)
      , dirdelay(dirdelayNs)
      , lastPulseWasForward(false)
      , currentDirection(false)
      , currentStep(false)
    {
        IOType::schedule(0, currentStep, currentDirection);
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
        if (std::abs(frequency) > maximumFrequency)
        {
            printf("frequency %d exceeds maximum %d\n", frequency, maximumFrequency);
        }
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
        uint32_t nextCycles = nowCycles + cyclesPerTick;
        changePins();
        int32_t toWait = int32_t(nextCycles - nowCycles);
        nowCycles += IOType::schedule(toWait, currentStep, currentDirection);
    }

private:
    void changePins()
    {
        if (steplen.expired(nowCycles))
        {
            currentStep = false;
            nowCycles += IOType::schedule(0, currentStep, currentDirection);
            dirhold.start(nowCycles);
        }
        else
            dirhold.update(nowCycles);
        dirsetup.update(nowCycles);
        dirdelay.update(nowCycles);

        int32_t toAdd = DDSaddValue;
        if (0 == toAdd)
            return;

        bool isForward = toAdd > 0;
        bool needToSwitchDirections = currentDirection != isForward;
        if (needToSwitchDirections && !dirhold.active(nowCycles))
        {
            currentDirection = isForward;
            nowCycles += IOType::schedule(0, currentStep, currentDirection);
            dirsetup.start(nowCycles);
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
        if (dirsetup.active(nowCycles))
            return;

        // If we still need to switch directions, we're in dirhold so hold off.
        if (needToSwitchDirections)
            return;

        // Hold opposite direction pulse if we are in dirdelay
        if (dirdelay.active(nowCycles) && isForward != lastPulseWasForward)
            return;

        DDSaccumulator = next;
        currentStep = true;
        nowCycles += IOType::schedule(0, currentStep, currentDirection);
        if (isForward)
            ++this->rawCount;
        else
            --this->rawCount;
        steplen.start(nowCycles);
        lastPulseWasForward = isForward;
        dirdelay.start(nowCycles);
    }
};

#endif
