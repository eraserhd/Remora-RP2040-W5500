#include "piostepgen.h"
#include "piostepgen.pio.h"
#include <cmath>

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
static int parse_pin(std::string const& name)
{
    if (name.length() != 4)
        return -1;
    if (name[0] != 'G' || name[1] != 'P')
        return -1;
    return 10*(name[2]-'0') + (name[3]-'0');
}

PioStepgen::PioStepgen(std::string step, std::string dir)
    : stepPin(parse_pin(step))
    , dirPin(parse_pin(dir))
    , lastDir(true)
    , position(0)
{
    //FIXME: configure
    const uint32_t steplen_ns = 5000;
    const uint32_t stepspace_ns = 5000;
    const uint32_t dirhold_ns = 20000;
    const uint32_t dirsetup_ns = 20000;

    steplen = (uint32_t)ceil(double(steplen_ns)/100.00587406015038 - 3);
    stepspace = (uint32_t)ceil(double(stepspace_ns)/100.00587406015038 - 6);
    dirhold = (uint32_t)ceil(double(dirhold_ns)/100.00587406015038 - 3);
    dirsetup = (uint32_t)ceil(double(dirsetup_ns)/100.00587406015038);

    printf("steplen = %u, stepspace = %u, dirhold = %u, dirsetup = %u\n", steplen, stepspace, dirhold, dirsetup);

    if (!pio_claim_free_sm_and_add_program(&stepgen_program, &pio, &sm, &offset))
    {
        printf("Could not claim state machine!\n");
        return;
    }

    pio_gpio_init(pio, stepPin);
    if (PICO_OK != pio_sm_set_consecutive_pindirs(pio, sm, stepPin, 1, true))
    {
        printf("Could not set step pin %d direction!\n", stepPin);
        return;
    }
    //FIXME: pull up/down?

    pio_gpio_init(pio, dirPin);
    if (PICO_OK != pio_sm_set_consecutive_pindirs(pio, sm, dirPin, 1, true))
    {
        printf("Could not set dir pin %d direction!\n", dirPin);
        return;
    }
    //FIXME: pull up/down?

    auto config = stepgen_program_get_default_config(offset);
    sm_config_set_sideset_pins(&config, stepPin);
    sm_config_set_out_pins(&config, dirPin, 1);
    sm_config_set_in_pins(&config, dirPin);
    sm_config_set_jmp_pin(&config, dirPin);
    if (PICO_OK != pio_sm_init(pio, sm, offset, &config))
    {
        printf("Could not configure PIO state machine!\n");
        return;
    }

    pio_sm_set_enabled(pio, sm, true);

    printf("Stepgen(%d,%d) finished initializing.\n", stepPin, dirPin);
}

PioStepgen::~PioStepgen()
{
}

static volatile bool logging = false;

void PioStepgen::frequencyCommand(int32_t threadFrequency, bool enable, int32_t frequencyCommand)
{
    logging = 0 != frequencyCommand;
    if (logging)
        printf("fq = %d %d\n", enable, frequencyCommand);
    if (!enable || 0 == frequencyCommand)
    {
        // Zero frequency command
        pio_sm_put_blocking(pio, sm, (1u << 22));
        return;
    }
    uint32_t gap = ceil(frequencyCommand / (float)threadFrequency / 100.00587406015038 - 6.0);
    bool dir = frequencyCommand > 0;
    //if (dir != lastDir)
    //{
        //gap=min(dirhold?,gap)
        // DIR COMMAND
        //pio_sm_put_blocking(pio, sm, (1u << 31) | (dirhold << 10) | ((uint32_t)lastDir << 9) | steplen);
        //pio_sm_put_blocking(pio, sm, max(dirsetup, gap) << 1| (uint32_t)lastDir); 
        //lastDir = dir;
        //return;
    //}

    gap = min(steplen, gap);
    pio_sm_put_blocking(pio, sm, (steplen << 23) | gap);
}

int32_t PioStepgen::jointFeedback()
{
    int n = 0;
    uint32_t rx;
    if (logging)
        printf("jointFeedback enter.\n");
    // Read the steps recorded in the RX queue
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
    {
        rx = pio_sm_get_blocking(pio, sm);
        position = position + (rx ? 1 : -1);
        ++n;
    }
    if (logging)
        printf("%d pos = %d (%d)\n", stepPin, position, n);
    return position;
}
