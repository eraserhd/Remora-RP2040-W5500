#include "../extern.h"

void createThreads(void)
{
    baseThread = new pruThread(BASE_SLICE);

    servoThread = new pruThread(SERVO_SLICE);
}
