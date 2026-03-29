#include "timerInterrupt.h"
#include "timer.h"
#include <cstdio>

// Define the vector table, it is only declared in the class declaration
Interrupt* Interrupt::ISRVectorTable[] = {0};

// Constructor
Interrupt::Interrupt(void){}

// Methods

void Interrupt::Register(int interruptNumber, Interrupt* intThisPtr)
{
    printf("Registering interrupt for interrupt number = %d\n", interruptNumber);
    ISRVectorTable[interruptNumber] = intThisPtr;
}

void Interrupt::SLICE0_Wrapper(void)
{
    ISRVectorTable[0]->ISR_Handler();
}

void Interrupt::SLICE1_Wrapper(void)
{
    ISRVectorTable[1]->ISR_Handler();
}


TimerInterrupt::TimerInterrupt(int interruptNumber, pruTimer* owner)
{
    // Allows interrupt to access owner's data
    InterruptOwnerPtr = owner;

    // When a device interrupt object is instantiated, the Register function must be called to let the
    // Interrupt base class know that there is an appropriate ISR function for the given interrupt.
    Interrupt::Register(interruptNumber, this);
}


void TimerInterrupt::ISR_Handler(void)
{
    this->InterruptOwnerPtr->timerTick();
}
