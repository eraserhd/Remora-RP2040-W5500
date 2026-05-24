#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "../configuration.h"
#include "../modules/module.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <vector>
#include <cstdint>


template<int Pin>
struct DebugPin
{
    inline static void init(void)
    {
        gpio_init(Pin);
        gpio_set_dir(Pin, 1);
    }
    inline static void set(void)   { gpio_put(Pin, 1); }
    inline static void clear(void) { gpio_put(Pin, 0); }
};

struct BaseThreadTraits
{
    static constexpr int slice = 0;
    static constexpr int irq = TIMER_IRQ_0;
    static constexpr uint32_t period = 1000000 / PRU_BASEFREQ;
    static constexpr bool runInISR = true;
};

struct ServoThreadTraits
{
    static constexpr int slice = 1;
    static constexpr int irq = TIMER_IRQ_1;
    static constexpr uint32_t period = 1000000 / PRU_SERVOFREQ;
    static constexpr bool runInISR = false;
};

template<class Traits, class DebugPinPolicy>
class pruThread
{
private:
    static std::vector<Module*> modules;
    static bool execute;

    static void startTimer(void)
    {
        printf("    setting up timer Slice %d\n", Traits::slice);
        printf("    actual period = %d\n", Traits::period);

        DebugPinPolicy::init();

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
        execute = true;
        if (Traits::runInISR)
            run();
    }

public:
    static void registerModule(Module *module)
    {
        modules.push_back(module);
    }

    static void start(void)
    {
        startTimer();
    }

    static void run(void)
    {
        if(!execute) return;

        DebugPinPolicy::set();
        for (auto& m : modules) m->runModule();
        DebugPinPolicy::clear();

        execute = false;
    }
};

template<class Traits, class DebugPinPolicy>
std::vector<Module*> pruThread<Traits, DebugPinPolicy>::modules;

template<class Traits, class DebugPinPolicy>
bool pruThread<Traits, DebugPinPolicy>::execute = false;

using BaseThread = pruThread<BaseThreadTraits, DebugPin<6>>;
using ServoThread = pruThread<ServoThreadTraits, DebugPin<27>>;

#endif
