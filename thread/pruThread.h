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
    static constexpr int slice = 0;
    static constexpr int irq = TIMER_IRQ_0;
    static constexpr uint32_t period = 1000000 / PRU_BASEFREQ;
    static constexpr int debugPin = 6;
    static constexpr bool runInISR = true;
};

struct ServoThreadTraits
{
    static constexpr int slice = 1;
    static constexpr int irq = TIMER_IRQ_1;
    static constexpr uint32_t period = 1000000 / PRU_SERVOFREQ;
    static constexpr int debugPin = 27;
    static constexpr bool runInISR = false;
};

template<class Traits>
class pruThread
{
protected:
    static pruThread<Traits>* instance;

    static void Register(pruThread<Traits>* intThisPtr)
    {
        instance = intThisPtr;
    }

private:
    void startTimer(void)
    {
        printf("    setting up timer Slice %d\n", Traits::slice);
        printf("    actual period = %d\n", Traits::period);

        hw_set_bits(&timer_hw->inte, 1u << Traits::slice);
        irq_set_exclusive_handler(Traits::irq, ISR_Handler);
        irq_set_enabled(Traits::irq, true);
        timer_hw->alarm[Traits::slice] = timer_hw->timerawl + Traits::period;

        printf("    timer started\n");
    }

    static void ISR_Handler(void)
    {
        hw_clear_bits(&timer_hw->intr, 1u << Traits::slice);
        timer_hw->alarm[Traits::slice] += Traits::period;
        //base thread is run from interrupt context.  Servo thread is not and can get interrupted.
        instance->execute = true;
        if (Traits::runInISR)
            instance->run();
    }

private:
    vector<Module*> vThread;                // vector containing pointers to Thread modules

public:
    bool                            execute;

    pruThread()
    {
        Register(this);
        printf("Creating thread %d\n", Traits::slice);

        gpio_init(Traits::debugPin);
        gpio_set_dir(Traits::debugPin, 1);

        this->execute = false;
    }

    void registerModule(Module *module)
    {
        vThread.push_back(module);
    }

    void startThread(void)
    {
        startTimer();
    }

    void run(void)
    {
        if(!this->execute) return;

        gpio_put(Traits::debugPin, 1);
        for (auto& m : vThread) m->runModule();
        gpio_put(Traits::debugPin, 0);

        this->execute = false;
    }
};

template<class Traits>
pruThread<Traits> *pruThread<Traits>::instance = nullptr;


using BaseThread = pruThread<BaseThreadTraits>;
using ServoThread = pruThread<ServoThreadTraits>;

#endif
