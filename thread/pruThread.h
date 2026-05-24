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

template<irq_num_t Irq, uint32_t Period, class Thread>
struct IRQThreadRunner
{
    static constexpr int slice = TIMER_ALARM_NUM_FROM_IRQ(Irq);
    static constexpr irq_num_t irq = Irq;
    static constexpr uint32_t period = Period;
    static constexpr bool runInISR = true;

    static void init(void)
    {
        hw_set_bits(&timer_hw->inte, 1u << slice);
        irq_set_exclusive_handler(Irq, handleInterrupt);
        irq_set_enabled(Irq, true);
        timer_hw->alarm[slice] = timer_hw->timerawl + Period;
    }

private:
    static void handleInterrupt(void)
    {
        hw_clear_bits(&timer_hw->intr, 1u << slice);
        timer_hw->alarm[slice] += Period;

        Thread::execute = true;
        if (runInISR)
            Thread::run();
    }
};

template<class RunPolicy, class DebugPinPolicy>
class pruThread
{
private:
    static std::vector<Module*> modules;

    static void runModules(void)
    {
        DebugPinPolicy::set();
        for (auto& m : modules) m->runModule();
        DebugPinPolicy::clear();
    }

public:
    static bool execute;
    static void registerModule(Module *module)
    {
        modules.push_back(module);
    }

    static void start(void)
    {
        printf("    setting up timer Slice %d\n", RunPolicy::slice);
        printf("    actual period = %d\n", RunPolicy::period);

        DebugPinPolicy::init();
        RunPolicy::init();

        printf("    timer started\n");
    }

    static void run(void)
    {
        if(!execute) return;
        runModules();
        execute = false;
    }
};

template<class RunPolicy, class DebugPinPolicy>
std::vector<Module*> pruThread<RunPolicy, DebugPinPolicy>::modules;

template<class RunPolicy, class DebugPinPolicy>
bool pruThread<RunPolicy, DebugPinPolicy>::execute = false;

struct BaseThread : public pruThread<IRQThreadRunner<TIMER_IRQ_0, 1000000 / PRU_BASEFREQ, BaseThread>, DebugPin<6>> {};
struct ServoThread : public pruThread<IRQThreadRunner<TIMER_IRQ_1, 1000000 / PRU_SERVOFREQ, ServoThread>, DebugPin<27>> {};

#endif
