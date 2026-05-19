#include "../extern.h"

void createThreads(void)
{
    baseThread = new pruThread(BASE_SLICE, PRU_BASEFREQ);

    servoThread = new pruThread(SERVO_SLICE , servo_freq);
}
