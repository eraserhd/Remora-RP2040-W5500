#include "hardware/irq.h"
#include "hardware/timer.h"

#include <stdio.h>

#include "../configuration.h"
#include "runner.h"
#include "pruThread.h"

void NormalRunContext::run(pruThread* thread)
{
    thread->execute = true;
}
