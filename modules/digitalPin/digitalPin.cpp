#include "digitalPin.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

DigitalPin *DigitalPin::load(JsonObject module)
{
    const char* comment = module["Comment"];
    printf("\n%s\n",comment);

    const char* pin = module["Pin"];
    const char* mode = module["Mode"];
    const char* invert = module["Invert"];
    const char* modifier = module["Modifier"];
    int dataBit = module["Data Bit"];

    int mod;
    bool inv;

    if (!strcmp(modifier,"Pull Up"))
    {
        mod = PULLUP;
    }
    else if (!strcmp(modifier,"Pull Down"))
    {
        mod = PULLDOWN;
    }
    else if (!strcmp(modifier,"Pull None"))
    {
        mod = PULLNONE;
    }
    else
    {
        printf("Invalid modifier '%s' at pin %s\n", modifier, pin);
        mod = PULLNONE;
    }

    if (!strcmp(invert,"True"))
    {
        inv = true;
    }
    else inv = false;

    printf("Make Digital %s at pin %s\n", mode, pin);

    if (!strcmp(mode,"Output"))
    {
        DigitalPin* digitalPin = new DigitalPin(1, pin, dataBit, inv, mod);
        servoThread->registerModule(digitalPin);
        return digitalPin;
    }

    if (!strcmp(mode,"Input"))
    {
        DigitalPin* digitalPin = new DigitalPin(0, pin, dataBit, inv, mod);
        servoThread->registerModule(digitalPin);
        return digitalPin;
    }

    printf("Error - incorrectly defined Digital Pin\n");
    return NULL;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

DigitalPin::DigitalPin(int mode, std::string portAndPin, int bitNumber, bool invert, int modifier) :
    mode(mode),
    portAndPin(portAndPin),
    bitNumber(bitNumber),
    invert(invert),
    modifier(modifier)
{
    this->pin = new Pin(this->portAndPin, this->mode, this->modifier);      // Input 0x0, Output 0x1
    this->mask = 1 << this->bitNumber;
}

bool DigitalPin::read() const
{
    bool pinState = pin->get();
    if(invert)
    {
        pinState = !pinState;
    }
    return pinState;
}

void DigitalPin::write(bool pinState) const
{
    if(invert)
    {
        pinState = !pinState;
    }
	pin->set(pinState);
}

void DigitalPin::update()
{
    if (this->mode == 0x0)                                  // the pin is configured as an input
    {
        txData_t* currentTxPacket = getCurrentTxBuffer(&txPingPongBuffer);
        if (read())
            currentTxPacket->inputs |= this->mask;
        else
            currentTxPacket->inputs &= ~this->mask;
    }
    else                                                // the pin is configured as an output
    {
        rxData_t* currentRxPacket = getCurrentRxBuffer(&rxPingPongBuffer);
        bool pinState = currentRxPacket->outputs & this->mask;       // get the value of the bit in the data source
        write(pinState);
    }
}
