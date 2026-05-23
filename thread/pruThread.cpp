#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include <cstdio>

#include "pico/stdlib.h"

#include "pruThread.h"
#include "../modules/module.h"


using namespace std;

// Timer constructor
pruTimer::pruTimer(uint8_t slice, pruThread* ownerPtr):
    slice(slice),
    timerOwnerPtr(ownerPtr)
{
    Interrupt::Register(this->slice, this);
    this->startTimer();
}


void pruTimer::ISR_Handler(void)
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
        gpio_init(6);
        gpio_set_dir(6, 1);
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 0
        irq_set_exclusive_handler(TIMER_IRQ_0, Interrupt::SLICE0_Wrapper);
        irq_set_enabled(TIMER_IRQ_0, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + BASE_PERIOD;
    }

    else if (this->slice == 1){
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 1
        irq_set_exclusive_handler(TIMER_IRQ_1, Interrupt::SLICE1_Wrapper);
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
    TimerPtr = new pruTimer(this->slice, this);
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

// Define the vector table, it is only declared in the class declaration
Interrupt* Interrupt::ISRVectorTable[] = {0};

void Interrupt::Register(int interruptNumber, Interrupt* intThisPtr)
{
       printf("Registering interrupt for interrupt number = %d\n", interruptNumber);
       ISRVectorTable[interruptNumber] = intThisPtr;
}

void Interrupt::SLICE0_Wrapper(void)
{
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    timer_hw->alarm[0] += BASE_PERIOD;
    gpio_put(6, 1);
    ISRVectorTable[0]->ISR_Handler();
    gpio_put(6, 0);
}

void Interrupt::SLICE1_Wrapper(void)
{
    hw_clear_bits(&timer_hw->intr, 1u << 1);
    timer_hw->alarm[1] += SERVO_PERIOD;
    ISRVectorTable[1]->ISR_Handler();
}
