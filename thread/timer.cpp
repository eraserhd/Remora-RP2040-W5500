#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "timer.h"
#include "pruThread.h"


#define BASE_PERIOD 1000000 / PRU_BASEFREQ
#define SERVO_PERIOD 1000000 / PRU_SERVOFREQ

struct BaseThreadTimer
{
    static constexpr int32_t IRQ = TIMER_IRQ_0;
    static constexpr int32_t BIT = 0;
    static constexpr int32_t PERIOD = BASE_PERIOD;
};

struct ServoThreadTimer
{
    static constexpr int32_t IRQ = TIMER_IRQ_1;
    static constexpr int32_t BIT = 1;
    static constexpr int32_t PERIOD = SERVO_PERIOD;
};

// Base class for all interrupt derived classes

#define PERIPH_COUNT_IRQn   8               // Total number of device interrupt sources - 8 PWM Slices (for the moment)


static pruThread* ISRVectorTable[PERIPH_COUNT_IRQn] = {};

void PWM_Wrap_Handler0()
{
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    timer_hw->alarm[0] += BASE_PERIOD;
    // base thread runs in interrupt context
    ISRVectorTable[0]->execute = true;
    ISRVectorTable[0]->run();
}

void PWM_Wrap_Handler1()
{
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    timer_hw->alarm[1] += SERVO_PERIOD;
    // servo thread will run next poll
    ISRVectorTable[1]->execute = true;
}

pruTimer::pruTimer(uint8_t slice, pruThread* ownerPtr)
    : slice(slice)
{
    ISRVectorTable[slice] = ownerPtr;
    this->startTimer();
}

template<typename Traits>
void startTimer()
{
    printf("    setting up timer Slice %d\n", Traits::BIT);
    printf("    actual period = %d\n", Traits::PERIOD);

    hw_set_bits(&timer_hw->inte, 1u << Traits::BIT);
    irq_set_exclusive_handler(Traits::IRQ, PWM_Wrap_Handler0);
    irq_set_enabled(Traits::IRQ, true);
    timer_hw->alarm[Traits::BIT] = timer_hw->timerawl + Traits::PERIOD;

    printf("    timer started\n");
}


void pruTimer::startTimer(void)
{
    if (this->slice == 0) 
        ::startTimer<BaseThreadTimer>();
    else if (this->slice == 1)
        ::startTimer<ServoThreadTimer>();
    else
        printf("    Invalid Slice\n");
}