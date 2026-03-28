#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <cstdint>

#include "../extern.h"
#include "../module.h"
#include "../../drivers/pin/pin.h"


class DigitalPin : public Module
{
private:
    int bitNumber;              // location in the data source
    bool invert;
    int mask;

    int mode;
    int modifier;
    std::string portAndPin;

    Pin *pin;

public:
    DigitalPin(int, std::string, int, bool, int);
    static DigitalPin *load(JsonObject module);

    bool read() const;
    void write(bool pinState) const;

    virtual void update(void);
};

#endif
