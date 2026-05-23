#include <cstdio>

#include "pico/stdlib.h"

#include "pruThread.h"
#include "../modules/module.h"


using namespace std;

// Thread constructor
pruThread::pruThread(uint8_t slice) :
    slice(slice)
{
    printf("Creating thread %d\n", this->slice);

    if (this->slice == 1){
        gpio_init(27);
        gpio_set_dir(27, 1);
    }

    this->execute = false;
}

void pruThread::startThread(void)
{
    TimerPtr = new pruTimer(this->slice, this);
}

void pruThread::registerModule(Module* module)
{
    this->vThread.push_back(module);
}


void pruThread::run(void)
{
    if(!this->execute)
        return;

    if (this->slice == 1){
        gpio_put(27, 1);
    }

    for (auto& m : vThread) m->runModule();

    if (this->slice == 1){
        gpio_put(27, 0);
    }

    this->execute = false;
}
