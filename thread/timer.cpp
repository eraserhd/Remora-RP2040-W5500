#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "../interrupt/interrupt.h"
#include "timerInterrupt.h"
#include "timer.h"
#include "pruThread.h"

#define BASE_PERIOD 1000000 / PRU_BASEFREQ
#define SERVO_PERIOD 1000000 / PRU_SERVOFREQ

void PWM_Wrap_Handler0()
{
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    timer_hw->alarm[0] = timer_hw->timerawl + BASE_PERIOD;
    Interrupt::SLICE0_Wrapper();
}

void PWM_Wrap_Handler1()
{
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    timer_hw->alarm[1] = timer_hw->timerawl + SERVO_PERIOD;
    Interrupt::SLICE1_Wrapper();
}

pruTimer::pruTimer(uint8_t slice, pruThread* ownerPtr)
    : slice(slice)
    , timerOwnerPtr(ownerPtr)
{
    new TimerInterrupt(this->slice, this);   // Instantiate a new Timer Interrupt object and pass "this" pointer
    this->startTimer();
}


void pruTimer::timerTick(void)
{
    //base thread is run from interrupt context.  Servo thread is not and can get interrupted.
    this->timerOwnerPtr->execute = true;
    if (this->slice == 0)
        this->timerOwnerPtr->run();
}

void pruTimer::startTimer(void)
{
    uint32_t period;
    
    printf("    setting up timer Slice %d\n", this->slice);
   
    if (this->slice == 0)
        period = BASE_PERIOD;
    else if (this->slice == 1)
        period = SERVO_PERIOD;
    else
        period = 0;
    printf("    actual period = %d\n", period);

    if (this->slice == 0){
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 0
        irq_set_exclusive_handler(TIMER_IRQ_0, PWM_Wrap_Handler0);
        irq_set_enabled(TIMER_IRQ_0, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + BASE_PERIOD;
    }

    else if (this->slice == 1){
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 1
        irq_set_exclusive_handler(TIMER_IRQ_1, PWM_Wrap_Handler1);
        irq_set_enabled(TIMER_IRQ_1, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + SERVO_PERIOD;
    } else{
        printf("    Invalid Slice\n");
    }

    printf("    timer started\n");
}