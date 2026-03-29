#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "timer.h"
#include "pruThread.h"

void InterruptRunContext::run(pruThread* thread)
{
    thread->execute = true;
    thread->run();
}

void NormalRunContext::run(pruThread* thread)
{
    thread->execute = true;
}
