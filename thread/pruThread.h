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

template<int ISR>
class pruTimer
{
protected:
    static pruThread* thread;

    static void Register(pruThread* intThisPtr);

    static void SLICE0_Wrapper();
    static void SLICE1_Wrapper();

private:

    uint8_t             slice;
    pruThread*          timerOwnerPtr;

    void startTimer(void);

    static void ISR_Handler(void);

public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);

};

#endif
