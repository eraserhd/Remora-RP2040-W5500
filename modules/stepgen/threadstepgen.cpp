#include "threadstepgen.h"

ThreadStepgen::ThreadStepgen(std::string step, std::string direction)
    : stepperPosition(0)
    , DDSaddValue(0)
    , DDSaccumulator(0)
    , stepPin(new Pin(step, OUTPUT))
    , directionPin(new Pin(direction, OUTPUT))
{
}

ThreadStepgen::~ThreadStepgen()
{
}

void ThreadStepgen::frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand)
{
    if (!enable)
    {
        DDSaddValue = 0;
        return;
    }
    DDSaddValue = frequencyCommand * (float)(1 << STEPBIT) / (float)threadFrequency;
}

int32_t ThreadStepgen::jointFeedback() const
{
    return stepperPosition;
}

void ThreadStepgen::update()
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

void ThreadStepgen::updatePost()
{
    stepPin->set(false);  // Reset step pin
}
