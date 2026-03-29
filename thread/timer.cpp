#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "timer.h"
#include "pruThread.h"

pruThread *BaseThreadTimer::thread = NULL;
pruThread *ServoThreadTimer::thread = NULL;

void InterruptRunContext::run(pruThread* thread)
{
    thread->execute = true;
    thread->run();
}

void NormalRunContext::run(pruThread* thread)
{
    thread->execute = true;
}

template<typename Traits>
void handleAlarmInterrupt()
{
    hw_clear_bits(&timer_hw->intr, 1u << Traits::BIT);
    timer_hw->alarm[Traits::BIT] += Traits::PERIOD;
    Traits::RunContext::run(Traits::thread);
}

template<typename Traits>
void runThread(pruThread *thread)
{
    printf("    setting up timer Slice %d\n", Traits::BIT);
    printf("    actual period = %d\n", Traits::PERIOD);

    Traits::thread = thread;
    hw_set_bits(&timer_hw->inte, 1u << Traits::BIT);
    irq_set_exclusive_handler(Traits::IRQ, handleAlarmInterrupt<Traits>);
    irq_set_enabled(Traits::IRQ, true);
    timer_hw->alarm[Traits::BIT] = timer_hw->timerawl + Traits::PERIOD;

    printf("    timer started\n");
}

pruTimer::pruTimer(uint8_t slice, pruThread* ownerPtr)
{
    if (slice == 0) 
        ::runThread<BaseThreadTimer>(ownerPtr);
    else if (slice == 1)
        ::runThread<ServoThreadTimer>(ownerPtr);
    else
        printf("    Invalid Slice\n");
}
