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

template<int32_t Bit, int32_t Freq, typename RunContext>
class ThreadRunner
{
private:
    static constexpr int32_t Irq = TIMER_IRQ_0 + Bit;
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

template<int32_t Bit, int32_t Freq, typename RunContext>
pruThread *ThreadRunner<Bit,Freq,RunContext>::thread = NULL;

typedef ThreadRunner<0, PRU_BASEFREQ, InterruptRunContext> BaseThreadRunner;
typedef ThreadRunner<1, PRU_SERVOFREQ, NormalRunContext> ServoThreadRunner;

#endif
