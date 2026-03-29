
#include "stepgen.h"
#include "../remora.h"

Stepgen *Stepgen::load(JsonObject module)
{
    const char* comment = module["Comment"];
    printf("\n%s\n",comment);

    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // create the step generator, register it in the thread
    Stepgen* stepgen = new Stepgen(step, dir);
    baseThread->registerModule(stepgen);
    return stepgen;
}

Stepgen::Stepgen(std::string step, std::string direction)
    : stepperPosition(0)
    , DDSaddValue(0)
    , DDSaccumulator(0)
    , stepPin(new Pin(step, OUTPUT))
    , directionPin(new Pin(direction, OUTPUT))
{
}

void Stepgen::frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand)
{
    if (!enable)
    {
        DDSaddValue = 0;
        return;
    }
    DDSaddValue = frequencyCommand * (float)(1 << STEPBIT) / (float)threadFrequency;
}

void Stepgen::update()
{
    int32_t toAdd = this->DDSaddValue;
    if (0 == toAdd)
        return;

    int32_t stepNow = DDSaccumulator; // Save the current DDS accumulator value
    DDSaccumulator += toAdd;          // Update the DDS accumulator with the new add value
    stepNow ^= DDSaccumulator;        // Test for changes in the low half of the DDS accumulator
    stepNow &= (1L << STEPBIT);       // Check for the step bit
    if (!stepNow)
        return;

    bool isForward = (toAdd > 0);
    directionPin->set(isForward);
    stepPin->set(true);

    if (isForward)
        ++stepperPosition;
    else
        --stepperPosition;
}

void Stepgen::updatePost()
{
    stepPin->set(false);  // Reset step pin
}
