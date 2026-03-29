#ifndef EXTERN_H
#define EXTERN_H

#include "configuration.h"
#include "remora.h"

#include "lib/ArduinoJson6/ArduinoJson.h"
#include "thread/pruThread.h"

extern JsonObject module;

// pointers to objects with global scope
extern pruThread* baseThread;
extern pruThread* servoThread;

#endif
