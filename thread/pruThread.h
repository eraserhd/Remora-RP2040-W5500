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

template<irq_num_t Irq, uint32_t Period>
struct IRQThreadRunner
{
    static constexpr int slice = TIMER_ALARM_NUM_FROM_IRQ(Irq);
    static constexpr irq_num_t irq = Irq;
    static constexpr uint32_t period = Period;
    static constexpr bool runInISR = true;
};

template<class RunPolicy, class DebugPinPolicy>
class pruThread
{
private:
    static std::vector<Module*> modules;
    static bool execute;

    static void startTimer(void)
    {
        printf("    setting up timer Slice %d\n", RunPolicy::slice);
        printf("    actual period = %d\n", RunPolicy::period);

        DebugPinPolicy::init();

        hw_set_bits(&timer_hw->inte, 1u << RunPolicy::slice);
        irq_set_exclusive_handler(RunPolicy::irq, ISR_Handler);
        irq_set_enabled(RunPolicy::irq, true);
        timer_hw->alarm[RunPolicy::slice] = timer_hw->timerawl + RunPolicy::period;

        printf("    timer started\n");
    }

    static void ISR_Handler(void)
    {
        hw_clear_bits(&timer_hw->intr, 1u << RunPolicy::slice);
        timer_hw->alarm[RunPolicy::slice] += RunPolicy::period;
        //base thread is run from interrupt context.  Servo thread is not and can get interrupted.
        execute = true;
        if (RunPolicy::runInISR)
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

template<class RunPolicy, class DebugPinPolicy>
std::vector<Module*> pruThread<RunPolicy, DebugPinPolicy>::modules;

template<class RunPolicy, class DebugPinPolicy>
bool pruThread<RunPolicy, DebugPinPolicy>::execute = false;

using BaseThread = pruThread<IRQThreadRunner<TIMER_IRQ_0, 1000000 / PRU_BASEFREQ>, DebugPin<6>>;
using ServoThread = pruThread<IRQThreadRunner<TIMER_IRQ_1, 1000000 / PRU_SERVOFREQ>, DebugPin<27>>;

#endif
