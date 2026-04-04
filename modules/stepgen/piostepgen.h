#ifndef PIOSTEPGEN_H
#define PIOSTEPGEN_H

#include "stepgen.h"

class PioStepgen : public Stepgen
{
public:
    PioStepgen(std::string, std::string);
    ~PioStepgen() override;

    void frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand) override;
    int32_t jointFeedback() const override;
};

#endif
