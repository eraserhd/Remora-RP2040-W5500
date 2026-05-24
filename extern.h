#ifndef EXTERN_H
#define EXTERN_H

#include "configuration.h"
#include "remora.h"

#include "lib/ArduinoJson6/ArduinoJson.h"
#include "thread/pruThread.h"


extern JsonObject module;

// pointers to objects with global scope
extern BaseThread baseThread;
extern ServoThread servoThread;

// unions for RX and TX data pointers that are used by the PRU threads

extern RxPingPongBuffer rxPingPongBuffer;
extern TxPingPongBuffer txPingPongBuffer;

#endif
