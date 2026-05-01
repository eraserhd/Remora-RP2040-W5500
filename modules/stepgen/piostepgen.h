#ifndef PIOSTEPGEN_H
#define PIOSTEPGEN_H

#include "stepgen.h"
#include "hardware/pio.h"

class PioStepgen : public Stepgen
{
private:
    int stepPin;
    int dirPin;
    PIO pio;
    uint sm;
    uint offset;

    bool lastDir;
    int32_t position;

    uint32_t steplen;
    uint32_t stepspace;
    uint32_t dirhold;
    uint32_t dirsetup;

    bool find_sm(void);

public:
    PioStepgen(std::string, std::string);
    ~PioStepgen() override;

    void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand) override;
    int32_t jointFeedback() override;
};

#endif
