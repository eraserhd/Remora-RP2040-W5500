#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "../configuration.h"

// Standard Template Library (STL) includes
#include <vector>
#include <stdint.h>

using namespace std;

class Module;

class pruThread
{
private:
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

class Interrupt
{
protected:
    static Interrupt* ISRVectorTable[8];

public:
    static void Register(int interruptNumber, Interrupt* intThisPtr);

    static void SLICE0_Wrapper();
    static void SLICE1_Wrapper();

    virtual void ISR_Handler(void) = 0;

};

class pruTimer
    : public Interrupt
{
private:

    uint8_t             slice;
    pruThread*          timerOwnerPtr;

    void startTimer(void);

    virtual void ISR_Handler(void) override;

public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);

};

#endif
