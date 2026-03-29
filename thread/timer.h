#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "../configuration.h"

class pruThread; // forward declaration

struct InterruptRunContext
{
    static void run(pruThread* thread);
};

struct NormalRunContext
{
    static void run(pruThread* thread);
};

template<int32_t irq, int32_t bit, int32_t freq, typename RunContextType>
struct ThreadRunner
{
    typedef RunContextType RunContext;
    static constexpr int32_t IRQ = irq;
    static constexpr int32_t BIT = bit;
    static constexpr int32_t PERIOD = 1000000 / freq;
    static pruThread *thread;

    static void handleAlarmInterrupt()
    {
        hw_clear_bits(&timer_hw->intr, 1u << BIT);
        timer_hw->alarm[BIT] += PERIOD;
        RunContext::run(thread);
    }
};

template<int32_t irq, int32_t bit, int32_t freq, typename RunContextType>
pruThread *ThreadRunner<irq,bit,freq,RunContextType>::thread = NULL;

typedef ThreadRunner<TIMER_IRQ_0, 0, PRU_BASEFREQ, InterruptRunContext> BaseThreadRunner;
typedef ThreadRunner<TIMER_IRQ_1, 1, PRU_SERVOFREQ, NormalRunContext> ServoThreadRunner;

class pruTimer
{
public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);
};

#endif
