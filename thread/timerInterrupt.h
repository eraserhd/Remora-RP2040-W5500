#ifndef TIMERINTERRUPT_H
#define TIMERINTERRUPT_H

// Derived class for timer interrupts

class pruTimer; // forward declaration

#include "../configuration.h"

// Base class for all interrupt derived classes

#define PERIPH_COUNT_IRQn   8               // Total number of device interrupt sources - 8 PWM Slices (for the moment)


class Interrupt
{
protected:
    static Interrupt* ISRVectorTable[PERIPH_COUNT_IRQn];

public:
    Interrupt(void);

    static void Register(int interruptNumber, Interrupt* intThisPtr);

    // wrapper functions to ISR_Handler()
    static void SLICE0_Wrapper();
    static void SLICE1_Wrapper();

    virtual void ISR_Handler(void) = 0;
};

class TimerInterrupt : public Interrupt
{
private:
    pruTimer* InterruptOwnerPtr;

public:
    TimerInterrupt(int interruptNumber, pruTimer* ownerptr);

    void ISR_Handler(void);
};

#endif
