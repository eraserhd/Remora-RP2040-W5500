
#include "stepgen.h"
#include "../remora.h"


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createStepgen()
{
    const char* comment = module["Comment"];
    printf("\n%s\n",comment);

    int joint = module["Joint Number"];
    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // create the step generator, register it in the thread
    Module* stepgen = new Stepgen(base_freq, joint, step, dir, STEPBIT);
    baseThread->registerModule(stepgen);
    baseThread->registerModulePost(stepgen);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Stepgen::Stepgen(int32_t threadFreq, int jointNumber, std::string step, std::string direction, int stepBit) :
    jointNumber(jointNumber),
    stepBit(stepBit)
{
    this->stepPin = new Pin(step, OUTPUT);
    this->directionPin = new Pin(direction, OUTPUT);
    this->DDSaccumulator = 0;
    this->rawCount = 0;
    this->frequencyScale = (float)(1 << this->stepBit) / (float)threadFreq;
    this->mask = 1 << this->jointNumber;
}


void Stepgen::update()
{
    // Use the standard Module interface to run makePulses()
    this->makePulses();
}

void Stepgen::updatePost()
{
    this->stopPulses();
}

void Stepgen::makePulses()
{
    int32_t stepNow = 0;

    rxData_t *rxData = getCurrentRxBuffer(&rxPingPongBuffer);
    txData_t *txData = getCurrentTxBuffer(&txPingPongBuffer);

    bool isEnabled = ((rxData->jointEnable & this->mask) != 0);
    if (!isEnabled)
        return;

    int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];             // Get the latest frequency command via pointer to the data source
    this->DDSaddValue = frequencyCommand * this->frequencyScale;      // Scale the frequency command to get the DDS add value
    stepNow = this->DDSaccumulator;                                         // Save the current DDS accumulator value
    this->DDSaccumulator += this->DDSaddValue;                              // Update the DDS accumulator with the new add value
    stepNow ^= this->DDSaccumulator;                                        // Test for changes in the low half of the DDS accumulator
    stepNow &= (1L << this->stepBit);                                       // Check for the step bit
    if (!stepNow)
        return;

    bool isForward = (this->DDSaddValue > 0);
    this->directionPin->set(isForward);                           // Set direction pin
    this->stepPin->set(true);                                           // Raise step pin

    if (isForward)
    {
        ++this->rawCount;
    }
    else
    {
        --this->rawCount;
    }
    txData->jointFeedback[this->jointNumber] = this->rawCount;
}


void Stepgen::stopPulses()
{
    this->stepPin->set(false);  // Reset step pin
}
