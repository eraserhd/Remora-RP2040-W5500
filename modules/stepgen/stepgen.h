#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>
#include <string>

#include "../extern.h"
#include "../module.h"
#include "../../drivers/pin/pin.h"

class Stepgen
{
public:
    virtual void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand) = 0;
    virtual int32_t jointFeedback() = 0;
    virtual ~Stepgen();

    static Stepgen* load(JsonObject module);
};

#endif
