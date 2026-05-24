#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "../configuration.h"
#include "../modules/module.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <vector>
#include <cstdint>

using namespace std;

struct BaseThreadTraits
{
    static constexpr int irq = TIMER_IRQ_0;
    static constexpr uint32_t period = 1000000 / PRU_BASEFREQ;
    static constexpr int debugPin = 6;
    static constexpr bool runInISR = true;
};

struct ServoThreadTraits
{
    static constexpr int irq = TIMER_IRQ_1;
    static constexpr uint32_t period = 1000000 / PRU_SERVOFREQ;
    static constexpr int debugPin = 27;
    static constexpr bool runInISR = false;
};

template<class Traits>
class pruThread
{
private:
    uint8_t                         slice;

    vector<Module*> vThread;                // vector containing pointers to Thread modules

public:
    bool                            execute;

    pruThread(uint8_t slice);

    void registerModule(Module *module)
    {
        vThread.push_back(module);
    }

    void startThread(void);

    void run(void)
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

};

template<class Traits>
class pruTimer
{
protected:
    static pruThread<Traits>* thread;

    static void Register(pruThread<Traits>* intThisPtr)
    {
        printf("Registering interrupt for interrupt number = %d\n", Traits::irq);
        thread = intThisPtr;
    }

    static void SLICE0_Wrapper()
    {
        hw_clear_bits(&timer_hw->intr, 1u << 0);
        timer_hw->alarm[0] += Traits::period;
        gpio_put(Traits::debugPin, 1);
        ISR_Handler();
        gpio_put(Traits::debugPin, 0);
    }

    static void SLICE1_Wrapper()
    {
        hw_clear_bits(&timer_hw->intr, 1u << 1);
        timer_hw->alarm[1] += Traits::period;
        ISR_Handler();
    }

private:

    uint8_t             slice;
    pruThread<Traits>*  timerOwnerPtr;

    void startTimer(void);

    static void ISR_Handler(void)
    {
        //base thread is run from interrupt context.  Servo thread is not and can get interrupted.
        thread->execute = true;
        if (Traits::runInISR)
            thread->run();
    }

public:
    pruTimer(uint8_t slice, pruThread<Traits>* ownerPtr)
        : slice(slice)
        , timerOwnerPtr(ownerPtr)
    {
        Register(ownerPtr);
        this->startTimer();
    }
};

template<class Traits>
void pruThread<Traits>::startThread(void)
{
    new pruTimer<Traits>(this->slice, this);
}

template<class Traits>
pruThread<Traits> *pruTimer<Traits>::thread = nullptr;

template<class Traits>
void pruTimer<Traits>::startTimer(void)
{
    printf("    setting up timer Slice %d\n", this->slice);
    printf("    actual period = %d\n", Traits::period);

    if (this->slice == 0){
        gpio_init(Traits::debugPin);
        gpio_set_dir(Traits::debugPin, 1);
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 0
        irq_set_exclusive_handler(TIMER_IRQ_0, pruTimer::SLICE0_Wrapper);
        irq_set_enabled(TIMER_IRQ_0, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + Traits::period;
    }

    else if (this->slice == 1){
        hw_set_bits(&timer_hw->inte, 1u << slice);//use alarm 1
        irq_set_exclusive_handler(TIMER_IRQ_1, pruTimer::SLICE1_Wrapper);
        irq_set_enabled(TIMER_IRQ_1, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + Traits::period;
    } else{
        printf("    Invalid Slice\n");
    }

    printf("    timer started\n");
}

template<class Traits>
pruThread<Traits>::pruThread(uint8_t slice) :
    slice(slice)
{
    printf("Creating thread %d\n", this->slice);

    if (this->slice == 1){
        gpio_init(27);
        gpio_set_dir(27, 1);
    }

    this->execute = false;
}

using BaseThread = pruThread<BaseThreadTraits>;
using ServoThread = pruThread<ServoThreadTraits>;

#endif
