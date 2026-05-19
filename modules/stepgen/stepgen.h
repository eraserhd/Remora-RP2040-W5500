#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>

#include "hardware/pio.h"

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

    inline void schedule(uint32_t cycles, bool step, bool dir)
    {
        stepPin->set(step);
        dirPin->set(dir);
    }
};

class PIOIO
{
private:
    int stepPin;
    int dirPin;
    PIO pio;
    uint sm;
    uint offset;

    // Reuse a PIO block we already loaded the program into when possible,
    // so multiple stepgens can share program memory.
    static PIO lastPio;
    static uint lastOffset;

    bool findStateMachine();

public:
    PIOIO(std::string const& step, std::string const& direction);

    inline void schedule(uint32_t cycles, bool step, bool dir)
    {
        // TODO: feed PIO TX FIFO once the PIO program does real work.
    }
};

using Stepgen = BasicStepgen<PLL_SYS_KHZ * 1000, PRU_BASEFREQ, PinIO>;

Stepgen *createStepgen(void);

#endif
