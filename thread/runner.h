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

template<int TimerNumber, int32_t Freq, typename RunContext>
class ThreadRunner
{
private:
    static_assert(TimerNumber >= 0 && TimerNumber <= 3, "RP2040 only has timers 0-3.");

    static constexpr int32_t Irq = TIMER_IRQ_0 + TimerNumber;
    static constexpr int32_t Period = 1000000 / Freq;

    static pruThread *thread;
    static void handleAlarmInterrupt()
    {
        hw_clear_bits(&timer_hw->intr, 1u << TimerNumber);
        timer_hw->alarm[TimerNumber] += Period;
        RunContext::run(thread);
    }

public:
    static void start(pruThread *_thread)
    {
        printf("    setting up timer Slice %d\n", TimerNumber);
        printf("    actual period = %d\n", Period);

        thread = _thread;
        hw_set_bits(&timer_hw->inte, 1u << TimerNumber);
        irq_set_exclusive_handler(Irq, handleAlarmInterrupt);
        irq_set_enabled(Irq, true);
        timer_hw->alarm[TimerNumber] = timer_hw->timerawl + Period;

        printf("    timer started\n");
    }
};

template<int TimerNumber, int32_t Freq, typename RunContext>
pruThread *ThreadRunner<TimerNumber,Freq,RunContext>::thread = NULL;

typedef ThreadRunner<0, PRU_BASEFREQ, InterruptRunContext> BaseThreadRunner;
typedef ThreadRunner<1, PRU_SERVOFREQ, NormalRunContext> ServoThreadRunner;

#endif
