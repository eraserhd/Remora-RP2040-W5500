#include "../extern.h"

void createThreads(void)
{
    servoThread = new pruThread(SERVO_SLICE);
}
