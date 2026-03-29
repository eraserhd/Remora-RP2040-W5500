#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "timer.h"
#include "pruThread.h"

struct BaseThreadTimer
{
    static constexpr int32_t IRQ = TIMER_IRQ_0;
    static constexpr int32_t BIT = 0;
    static constexpr int32_t PERIOD = 1000000 / PRU_BASEFREQ;
    static pruThread *thread;
};

struct ServoThreadTimer
{
    static constexpr int32_t IRQ = TIMER_IRQ_1;
    static constexpr int32_t BIT = 1;
    static constexpr int32_t PERIOD = 1000000 / PRU_SERVOFREQ;
    static pruThread *thread;
};

pruThread *BaseThreadTimer::thread = NULL;
pruThread *ServoThreadTimer::thread = NULL;

void PWM_Wrap_Handler0()
{
    typedef BaseThreadTimer Traits;

    hw_clear_bits(&timer_hw->intr, 1u << Traits::BIT);
    timer_hw->alarm[Traits::BIT] += Traits::PERIOD;
    // base thread runs in interrupt context
    Traits::thread->execute = true;
    Traits::thread->run();
}

void PWM_Wrap_Handler1()
{
    typedef ServoThreadTimer Traits;

    hw_clear_bits(&timer_hw->intr, 1u << Traits::BIT);
    timer_hw->alarm[Traits::BIT] += Traits::PERIOD;
    // servo thread will run next poll
    Traits::thread->execute = true;
}


template<typename Traits>
void startTimer(pruThread *thread)
{
    printf("    setting up timer Slice %d\n", Traits::BIT);
    printf("    actual period = %d\n", Traits::PERIOD);

	Traits::thread = thread;
    hw_set_bits(&timer_hw->inte, 1u << Traits::BIT);
    irq_set_exclusive_handler(Traits::IRQ, PWM_Wrap_Handler0);
    irq_set_enabled(Traits::IRQ, true);
    timer_hw->alarm[Traits::BIT] = timer_hw->timerawl + Traits::PERIOD;

    printf("    timer started\n");
}

pruTimer::pruTimer(uint8_t slice, pruThread* ownerPtr)
{
    if (slice == 0) 
        ::startTimer<BaseThreadTimer>(ownerPtr);
    else if (slice == 1)
        ::startTimer<ServoThreadTimer>(ownerPtr);
    else
        printf("    Invalid Slice\n");
}
