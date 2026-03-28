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
        return new DigitalPin(1, pin, inv, mod);

    if (!strcmp(mode,"Input"))
        return new DigitalPin(0, pin, inv, mod);

    printf("Error - incorrectly defined Digital Pin\n");
    return NULL;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

// Input mode 0x0, Output 0x1
DigitalPin::DigitalPin(int mode, std::string portAndPin, bool invert, int modifier)
    : mode(mode)
    , portAndPin(portAndPin)
    , invert(invert)
    , modifier(modifier)
    , pin(new Pin(this->portAndPin, this->mode, this->modifier))
{
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
