#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include <cstdio>

#include "pico/stdlib.h"

#include "pruThread.h"
#include "../modules/module.h"


using namespace std;

template<int ISR>
pruThread *pruTimer<ISR>::thread = nullptr;

// Timer constructor
template<int ISR>
pruTimer<ISR>::pruTimer(uint8_t slice, pruThread* ownerPtr):
    slice(slice),
    timerOwnerPtr(ownerPtr)
{
    Register(ownerPtr);
    this->startTimer();
}

template<int ISR>
void pruTimer<ISR>::ISR_Handler(void)
{
    //base thread is run from interrupt context.  Servo thread is not and can get interrupted.
    thread->execute = true;
    if (ISR == 0)
        thread->run();
}

template<int ISR>
void pruTimer<ISR>::startTimer(void)
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
        gpio_init(6);
        gpio_set_dir(6, 1);
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 0
        irq_set_exclusive_handler(TIMER_IRQ_0, pruTimer::SLICE0_Wrapper);
        irq_set_enabled(TIMER_IRQ_0, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + BASE_PERIOD;
    }

    else if (this->slice == 1){
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 1
        irq_set_exclusive_handler(TIMER_IRQ_1, pruTimer::SLICE1_Wrapper);
        irq_set_enabled(TIMER_IRQ_1, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + SERVO_PERIOD;
    } else{
        printf("    Invalid Slice\n");
    }

    printf("    timer started\n");
}

// Thread constructor
pruThread::pruThread(uint8_t slice) :
    slice(slice)
{
    printf("Creating thread %d\n", this->slice);

    if (this->slice == 1){
        gpio_init(27);
        gpio_set_dir(27, 1);
    }

    this->execute = false;
}

void pruThread::startThread(void)
{
    if (this->slice == 0)
        new pruTimer<TIMER_IRQ_0>(this->slice, this);
    else
        new pruTimer<TIMER_IRQ_1>(this->slice, this);
}

void pruThread::registerModule(Module* module)
{
    this->vThread.push_back(module);
}


void pruThread::run(void)
{
    if(!this->execute)
        return;

    if (this->slice == 1){
        gpio_put(27, 1);
    }

    for (auto& m : vThread) m->runModule();

    if (this->slice == 1){
        gpio_put(27, 0);
    }

    this->execute = false;
}

template<int ISR>
void pruTimer<ISR>::Register(pruThread* intThisPtr)
{
       printf("Registering interrupt for interrupt number = %d\n", ISR);
       thread = intThisPtr;
}

template<int ISR>
void pruTimer<ISR>::SLICE0_Wrapper(void)
{
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    timer_hw->alarm[0] += BASE_PERIOD;
    gpio_put(6, 1);
    ISR_Handler();
    gpio_put(6, 0);
}

template<int ISR>
void pruTimer<ISR>::SLICE1_Wrapper(void)
{
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    timer_hw->alarm[1] += SERVO_PERIOD;
    ISR_Handler();
}
