#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "../configuration.h"

// Standard Template Library (STL) includes
#include <vector>
#include <stdint.h>

using namespace std;

class Module;

class TimerInterrupt; // forward declaration
class pruThread; // forward declaration

class Interrupt
{
protected:
    static Interrupt* ISRVectorTable[8];

public:
    static void Register(int interruptNumber, Interrupt* intThisPtr);

    // wrapper functions to ISR_Handler()
    static void SLICE0_Wrapper();
    static void SLICE1_Wrapper();

    virtual void ISR_Handler(void) = 0;

};

class pruTimer
{
friend class TimerInterrupt;

private:

    TimerInterrupt*     interruptPtr;
    uint8_t             slice;
    pruThread*          timerOwnerPtr;

    void startTimer(void);
    void timerTick();           // Private timer tiggered method

public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);

};

class pruThread
{
private:
    pruTimer *TimerPtr;
    uint8_t                         slice;

    vector<Module*> vThread;                // vector containing pointers to Thread modules

public:
    bool                            execute;

    pruThread(uint8_t slice);

    void registerModule(Module *module);
    void startThread(void);

    void run(void);
};

#define BASE_PERIOD 1000000 / PRU_BASEFREQ
#define SERVO_PERIOD 1000000 / PRU_SERVOFREQ

// Base class for all interrupt derived classes

class TimerInterrupt : public Interrupt
{
private:
    
    pruTimer* InterruptOwnerPtr;

public:

    TimerInterrupt(int interruptNumber, pruTimer* ownerptr);

    void ISR_Handler(void);
};


#endif
