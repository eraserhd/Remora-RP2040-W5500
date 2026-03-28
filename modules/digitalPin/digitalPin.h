#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <cstdint>

#include "../../drivers/pin/pin.h"
#include "../lib/ArduinoJson6/ArduinoJson.h"


class DigitalPin
{
private:
    bool invert;
    int mode;
    int modifier;
    std::string portAndPin;

    Pin *pin;

public:
    DigitalPin(int, std::string, bool, int);
    static DigitalPin *load(JsonObject module);

    bool read() const;
    void write(bool pinState) const;

    virtual void update(void);
};

#endif
