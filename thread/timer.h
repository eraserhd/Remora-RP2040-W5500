#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

class pruThread; // forward declaration

class pruTimer
{
private:
    uint8_t             slice;

    void startTimer(void);

    static void PWM_Wrap_Handler0(void);
    static void PWM_Wrap_Handler1(void);

public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);
};

#endif
