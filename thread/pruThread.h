#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include "timer.h"
#include <vector>

using namespace std;

class Module;

class pruThread
{
private:
    uint8_t             slice;

    vector<Module*> vThread;        // vector containing pointers to Thread modules

public:
    bool                execute;

    pruThread(uint8_t slice);

    void registerModule(Module *module);
    void startThread(void);
    void run(void);
};

#endif
