#include <cstdio>

#include "pico/stdlib.h"

#include "pruThread.h"
#include "../modules/module.h"


using namespace std;

// Thread constructor
pruThread::pruThread(uint8_t slice) 
    : slice(slice)
{
    printf("Creating thread\n");
    
    if (this->slice == 0){
        gpio_init(6);
        gpio_set_dir(6, 1);
    }

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

void pruThread::stopThread(void)
{
    this->TimerPtr->stopTimer();
}


void pruThread::registerModule(Module* module)
{
    this->vThread.push_back(module);
}


void pruThread::run(void)
{
    if(!this->execute)
        return; 
    
    if (this->slice == 0){
        gpio_put(6, 1);
    }

    if (this->slice == 1){
        gpio_put(27, 1);
    }

    for (auto iter = vThread.begin(); iter != vThread.end(); ++iter) (*iter)->runModule();
    for (auto iter = vThread.begin(); iter != vThread.end(); ++iter) (*iter)->runModulePost();

    if (this->slice == 0){
        gpio_put(6, 0);
    }

    if (this->slice == 1){
        gpio_put(27, 0);
    }

    this->execute = false;
}
