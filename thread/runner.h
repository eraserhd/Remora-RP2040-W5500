#ifndef RUNNER_H
#define RUNNER_H

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

template<int32_t Irq, int32_t Bit, int32_t Freq, typename RunContext>
class ThreadRunner
{
private:
    static constexpr int32_t Period = 1000000 / Freq;

    static pruThread *thread;
    static void handleAlarmInterrupt()
    {
        hw_clear_bits(&timer_hw->intr, 1u << Bit);
        timer_hw->alarm[Bit] += Period;
        RunContext::run(thread);
    }

public:
    static void start(pruThread *_thread)
    {
        printf("    setting up timer Slice %d\n", Bit);
        printf("    actual period = %d\n", Period);

        thread = _thread;
        hw_set_bits(&timer_hw->inte, 1u << Bit);
        irq_set_exclusive_handler(Irq, handleAlarmInterrupt);
        irq_set_enabled(Irq, true);
        timer_hw->alarm[Bit] = timer_hw->timerawl + Period;

        printf("    timer started\n");
    }
};

template<int32_t irq, int32_t bit, int32_t freq, typename RunContextType>
pruThread *ThreadRunner<irq,bit,freq,RunContextType>::thread = NULL;

typedef ThreadRunner<TIMER_IRQ_0, 0, PRU_BASEFREQ, InterruptRunContext> BaseThreadRunner;
typedef ThreadRunner<TIMER_IRQ_1, 1, PRU_SERVOFREQ, NormalRunContext> ServoThreadRunner;

#endif
