#ifndef BASIC_STEPGEN_H
#define BASIC_STEPGEN_H

#include <cstdint>
#include <cmath>
#include <string>

#include "../../remora.h"
#include "../module.h"

template<int32_t Dx>
struct BasicDDSAccumulator
{
    int32_t value;

    inline BasicDDSAccumulator() : value(0) {}

    static constexpr int32_t low  = -Dx + 1;
    static constexpr int32_t high = Dx;

    inline bool triggered() const                     { return value < low || value > high; }
    inline void advance(int32_t freq, int32_t cycles) { if (!triggered()) value += 2*freq*cycles; }

    // Acknowledge that the triggered step has been emitted.
    inline void reset()
    {
        assert(triggered());
        if (value < low) value += 2*Dx;
        else if (value > high) value -= 2*Dx;
        assert(!triggered());
    }

    inline int32_t cyclesUntilNextStep(int32_t freq) const
    {
        if (triggered()) return 0;
        if (freq == 0) return INT32_MAX;
        if (freq > 0)
            return ((high + 1 - value) + (2*freq - 1)) / (2*freq);
        else
            return ((value - (low - 1)) + (-2*freq - 1)) / (-2*freq);
    }
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

        inline int32_t remaining(uint32_t now) const
        {
            if (!armed) return 0;
            return int32_t(now - expiryCycle);
        }

        // Disarm if expired, so we don't spuriously show armed on next cycle.
        inline void update(uint32_t now)
        {
            if (armed && remaining(now) >= 0)
                armed = false;
        }

        inline bool active(uint32_t now) const
        {
            return armed && remaining(now) < 0;
        }
    };

    using DDSAccumulator = BasicDDSAccumulator<CpuFreq>;

private:
    int jointNumber;
    volatile int32_t rawCount;
    volatile int32_t frequency;
    int32_t localFrequency;
    DDSAccumulator dds;
    uint32_t tickStartCycle;
    uint32_t plannedCycles;
    uint32_t steplenCycles;
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
      , frequency(0)
      , localFrequency(0)
      , tickStartCycle(0)
      , plannedCycles(0)
      , steplenCycles(nsToCycles(steplenNs))
      , maximumFrequency(CpuFreq / nsToCycles(steplenNs + stepspaceNs))
      , dirsetup(dirsetupNs)
      , dirhold(dirholdNs)
      , dirdelay(dirdelayNs)
      , lastPulseWasForward(false)
      , currentDirection(false)
    {
        IOType::schedule(0, false, currentDirection);
    }

    // Callable from core0, owing to frequency volatility and it being the only
    // data member updated so it can't be inconsistent.
    void setFrequency(int32_t frequency, bool enabled)
    {
        if (!enabled)
        {
            this->frequency = 0;
            return;
        }
        this->frequency = frequency;
        if (std::abs(frequency) > maximumFrequency)
        {
            printf("frequency %d exceeds maximum %d\n", frequency, maximumFrequency);
        }
    }

    // Callable from core0, owing to rawCount volatility and it being the only
    // data member read so it can't be inconsistent.  Lagging a little bit is OK.
    int32_t getRawCount() const
    {
        return rawCount;
    }

    virtual void update() override
    {
        dirhold.update(plannedCycles);
        dirsetup.update(plannedCycles);
        dirdelay.update(plannedCycles);
        changePins();
        tickStartCycle += cyclesPerTick;
    }

private:
    inline void plan(uint32_t cycles, bool step, bool dir)
    {
        plannedCycles += IOType::schedule(cycles, step, dir);
    }

    inline int32_t tickCyclesRemaining() const
    {
        uint32_t nextCycles = tickStartCycle + cyclesPerTick;
        return int32_t(nextCycles - plannedCycles);
    }

    inline void planWaitUntilEndOfTick()
    {
        int32_t toWait = tickCyclesRemaining();
        if (toWait > 0)
            plan(toWait, false, currentDirection);
    }

    // Wait for cycles, but not past the end of the tick in case we
    // get new orders in.
    inline void planEvitableWait(uint32_t cycles)
    {
        int32_t remaining = tickCyclesRemaining();
        if (remaining <= 0) return;
        plan(std::min(cycles, uint32_t(remaining)), false, currentDirection);
    }

    void changePins()
    {
        localFrequency = frequency;
        dds.advance(localFrequency, cyclesPerTick);
        if (0 == localFrequency)
        {
            planWaitUntilEndOfTick();
            return;
        }

        bool isForward = localFrequency > 0;
        if (currentDirection != isForward)
        {
            uint32_t wait = dirhold.remaining(plannedCycles);
            if (wait > tickCyclesRemaining())
            {
                // LinuxCNC might change its mind about direction before
                // we get there.
                planWaitUntilEndOfTick();
                return;
            }

            currentDirection = isForward;
            plan(wait, false, currentDirection);
            dirsetup.start(plannedCycles);
        }

        if (dirsetup.active(plannedCycles))
            planEvitableWait(dirsetup.durationCycles);

        if (dirdelay.active(plannedCycles) && isForward != lastPulseWasForward)
            planEvitableWait(dirdelay.durationCycles);

        while (tickCyclesRemaining() > 0)
        {
            if (!dds.triggered())
            {
                planWaitUntilEndOfTick();
                continue;
            }

            dds.reset();
            plan(0, true, currentDirection);
            if (isForward)
                ++this->rawCount;
            else
                --this->rawCount;
            lastPulseWasForward = isForward;
            dirdelay.start(plannedCycles);
            plan(steplenCycles, false, currentDirection);
            dirhold.start(plannedCycles);
            planWaitUntilEndOfTick(); //FIXME:
        }
    }
};

#endif
