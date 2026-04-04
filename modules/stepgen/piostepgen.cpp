#include "piostepgen.h"
#include "piostepgen.pio.h"

// Typical required stepper timings, in nanoseconds:
//
//                   TB6600  Min Typ Max Typ
// ---------------- ------- -------- -------
// step time          5,000      125  30,000
// step space         5,000      125 100,000
// direction hold    20,000      125  24,000
// direction setup   20,000      125 200,000
//
// There are 1,000,000,000 nanoseconds in a second.
// The Pico runs at 133Mhz.
// This works out to be 7.518796992481203ns per clock cycle.
// 
// (/ 100 7.518796992481203) ;=> 13.3
// So we'll set the PIO divider to 13 and 77/256 (~ 13.30078125)
// With error, that's (/ 1000000000.0 (/ 133000000.0 (+ 13 77/256))) ;=> 100.00587406015038
//
// stepspace (which is just a minimum gap value) is handled before passing to
// the PIO.
/*
    (let [cycle-ns      100.00587406015038
          max-steplen   30000
          max-dirhold   24000
          max-dirsetup  200000
          bits          (fn [max-value]
                         (long (Math/ceil (/ (Math/log (/ max-value cycle-ns)) (Math/log 2)))))
          steplen-bits  (bits max-steplen)
          gap-bits      (- 32 1 1 steplen-bits)
          max-gap-value (* cycle-ns (Math/pow 2 gap-bits))]
      {:steplen-bits    steplen-bits,
       :gap-bits        gap-bits,
       :lowest-freq     (/ 1000000000 max-gap-value)}) ;=> {:steplen-bits 9, :gap-bits 21, :lowest-freq 4.768091501468429}
 */

PioStepgen::PioStepgen(std::string, std::string)
{
}

PioStepgen::~PioStepgen()
{
}

void PioStepgen::frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand)
{
}

int32_t PioStepgen::jointFeedback() const
{
    return 0; //FIXME:
}
