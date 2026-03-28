
#include "stepgen.h"
#include "../remora.h"

Stepgen *Stepgen::load(JsonObject module)
{
    const char* comment = module["Comment"];
    printf("\n%s\n",comment);

    int joint = module["Joint Number"];
    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // create the step generator, register it in the thread
    Stepgen* stepgen = new Stepgen(base_freq, joint, step, dir);
    baseThread->registerModule(stepgen);
    baseThread->registerModulePost(stepgen);
    return stepgen;
}

Stepgen::Stepgen(int32_t threadFreq, int jointNumber, std::string step, std::string direction)
    : jointNumber(jointNumber)
{
    this->stepPin = new Pin(step, OUTPUT);
    this->directionPin = new Pin(direction, OUTPUT);
    this->DDSaccumulator = 0;
    this->rawCount = 0;
    this->frequencyScale = (float)(1 << STEPBIT) / (float)threadFreq;
    this->mask = 1 << this->jointNumber;
}

void Stepgen::frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand)
{
}

void Stepgen::update()
{
    rxData_t *rxData = getCurrentRxBuffer(&rxPingPongBuffer);
    bool isEnabled = ((rxData->jointEnable & this->mask) != 0);
    int32_t frequencyCmd = rxData->jointFreqCmd[this->jointNumber];
    frequencyCommand(base_freq, isEnabled, frequencyCmd);

    if (!isEnabled)
        return;

    int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber]; // Get the latest frequency command via pointer to the data source
    int32_t DDSaddValue = frequencyCommand * this->frequencyScale;      // Scale the frequency command to get the DDS add value
    int32_t stepNow = this->DDSaccumulator;                             // Save the current DDS accumulator value
    this->DDSaccumulator += DDSaddValue;                                // Update the DDS accumulator with the new add value
    stepNow ^= this->DDSaccumulator;                                    // Test for changes in the low half of the DDS accumulator
    stepNow &= (1L << STEPBIT);                                         // Check for the step bit
    if (!stepNow)
        return;

    bool isForward = (DDSaddValue > 0);
    this->directionPin->set(isForward);                                 // Set direction pin
    this->stepPin->set(true);                                           // Raise step pin

    if (isForward)
    {
        ++this->rawCount;
    }
    else
    {
        --this->rawCount;
    }

    txData_t *txData = getCurrentTxBuffer(&txPingPongBuffer);
    txData->jointFeedback[this->jointNumber] = this->rawCount;
}

void Stepgen::updatePost()
{
    this->stepPin->set(false);  // Reset step pin
}
