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
    virtual int32_t jointFeedback() const = 0;
    virtual ~Stepgen();

    static Stepgen* load(JsonObject module);
};

class ThreadStepgen : public Module, public Stepgen
{
private:
    volatile int32_t stepperPosition;
    volatile int32_t DDSaddValue;
    int32_t DDSaccumulator;         // Direct Digital Synthesis (DDS) accumulator

    Pin *stepPin, *directionPin;        // class object members - Pin objects

public:
    ThreadStepgen(std::string, std::string);
    ~ThreadStepgen() override;


    void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand) override;
    int32_t jointFeedback() const override;

    void update(void) override;           // Module default interface
    void updatePost(void) override;
};


#endif
