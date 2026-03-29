#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

class pruThread; // forward declaration

class pruTimer
{
private:
    uint8_t             slice;

    void startTimer(void);
public:
    pruTimer(uint8_t slice, pruThread* ownerPtr);
};

#endif
