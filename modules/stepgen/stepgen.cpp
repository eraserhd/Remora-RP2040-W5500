
#include "stepgen.h"
#include "piostepgen.h"
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
    return new PioStepgen(step, dir);
}
