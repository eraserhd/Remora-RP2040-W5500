#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>

#include "../extern.h"
#include "../../configuration.h"
#include "../../drivers/pin/pin.h"

#include "basic_stepgen.h"

class PinIO
{
private:
    Pin *stepPin;
    Pin *dirPin;

public:
    PinIO(std::string const& step, std::string const& direction)
        : stepPin(new Pin(step, OUTPUT))
        , dirPin(new Pin(direction, OUTPUT))
    {
    }

    inline void schedule(uint32_t cycles, PinType pin, bool value)
    {
        switch (pin)
        {
        case PinType::StepPin:
            stepPin->set(value);
            break;
        case PinType::DirectionPin:
            dirPin->set(value);
            break;
        }
    }
};

using Stepgen = BasicStepgen<PLL_SYS_KHZ * 1000, PRU_BASEFREQ, PinIO>;

Stepgen *createStepgen(void);

#endif
