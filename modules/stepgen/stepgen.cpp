
#include "stepgen.h"
#include "threadstepgen.h"
#include "../remora.h"

Stepgen::~Stepgen()
{
}

Stepgen *Stepgen::load(JsonObject module)
{
    const char* comment = module["Comment"];
    printf("\n%s\n",comment);

    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // create the step generator, register it in the thread
    ThreadStepgen* stepgen = new ThreadStepgen(step, dir);
    baseThread->registerModule(stepgen);
    return stepgen;
}
