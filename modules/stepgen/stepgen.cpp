
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

    float steplen = module["steplen"];
    float stepspace = module["stepspace"];
    float dirsetup = module["dirsetup"];
    float dirhold = module["dirhold"];
    float dirdelay = module["dirdelay"];

    // configure pointers to data source and feedback location
    //ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    //ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    //ptrJointEnable = &rxData.jointEnable;

    // create the step generator, register it in the thread
    Module* stepgen = new Stepgen(
        base_freq, joint, step, dir, steplen, stepspace,
        dirsetup, dirhold, dirdelay, STEPBIT
    );
    baseThread->registerModule(stepgen);
    baseThread->registerModulePost(stepgen);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Stepgen::Stepgen(
    int32_t threadFreq,
    int jointNumber,
    std::string step,
    std::string direction,
    float steplen,
    float stepspace,
    float dirsetup,
    float dirhold,
    float dirdelay,
    int stepBit
) : jointNumber(jointNumber)
  , step(step)
  , direction(direction)
  , steplen(steplen)
  , stepspace(stepspace)
  , dirsetup(dirsetup)
  , dirhold(dirhold)
  , dirdelay(dirdelay)
  , stepBit(stepBit)
{
    this->stepPin = new Pin(this->step, OUTPUT);
    this->directionPin = new Pin(this->direction, OUTPUT);
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

void Stepgen::slowUpdate()
{
    return;
}

void Stepgen::makePulses()
{
    this->rxData = getCurrentRxBuffer(&rxPingPongBuffer);
    this->txData = getCurrentTxBuffer(&txPingPongBuffer);

    bool isEnabled = ((rxData->jointEnable & this->mask) != 0);
    if (!isEnabled)                                                      // this Step generator is enables so make the pulses
        return;

    int32_t frequencyCommand = rxData->jointFreqCmd[this->jointNumber];  // Get the latest frequency command via pointer to the data source
    this->DDSaddValue = frequencyCommand * this->frequencyScale;         // Scale the frequency command to get the DDS add value
    int32_t stepNow = this->DDSaccumulator;                              // Save the current DDS accumulator value
    this->DDSaccumulator += this->DDSaddValue;                           // Update the DDS accumulator with the new add value
    stepNow ^= this->DDSaccumulator;                                     // Test for changes in the low half of the DDS accumulator
    stepNow &= (1L << this->stepBit);                                    // Check for the step bit

    if (!stepNow)
        return;

    bool isForward = this->DDSaddValue > 0;
    this->directionPin->set(isForward);                                  // Set direction pin
    this->stepPin->set(true);                                            // Raise step pin
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
