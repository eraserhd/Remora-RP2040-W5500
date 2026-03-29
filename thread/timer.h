#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "hardware/irq.h"
#include "../configuration.h"

class pruThread; // forward declaration

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

class pruTimer
{
public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);
};

#endif
