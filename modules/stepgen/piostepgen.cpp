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
// The Pico runs at 125Mhz.
// (/ 1000000000.0 125000000000) ;=> 0.008
// This works out to be 8ns per clock cycle.
// We'll set the divider to (/ 50 8.0) ;=> 6.25
//
// stepspace (which is just a minimum gap value) is handled before passing to
// the PIO.
/*
    (let [cycle-ns      50
          max-steplen   30000
          max-dirhold   24000
          max-dirsetup  200000
          bits          (fn [max-value]
                         (long (Math/ceil (/ (Math/log (/ max-value cycle-ns)) (Math/log 2)))))
          steplen-bits  (bits max-steplen)
          gap-bits      (- 32 1 steplen-bits)
          max-gap-value (* cycle-ns (Math/pow 2 gap-bits))]
      {:steplen-bits    steplen-bits,
       :gap-bits        gap-bits,
       :lowest-freq     (/ 1000000000 max-gap-value)}) ;=> {:steplen-bits 10, :gap-bits 21, :lowest-freq 9.5367431640625}
 */
static int parse_pin(std::string const& name)
{
    if (name.length() != 4)
        return -1;
    if (name[0] != 'G' || name[1] != 'P')
        return -1;
    return 10*(name[2]-'0') + (name[3]-'0');
}

std::vector<PioStepgen*> stepgens;

void pio_rx_irq_handler(void)
{
    for (auto* stepgen : stepgens)
    {
        while (!pio_sm_is_rx_fifo_empty(stepgen->pio, stepgen->sm))
        {
            auto rx = pio_sm_get(stepgen->pio, stepgen->sm);
            stepgen->position += (rx ? 1 : -1);
        }
    }
}

bool PioStepgen::find_sm(void)
{
    static PIO last_pio = NULL;
    static uint last_offset = 0;

    // Try to claim a state machine on the last PIO block first -- we might
    // be able to reuse the last program.
    if (NULL != last_pio)
    {
        int ret = pio_claim_unused_sm(last_pio, false);
        if (ret >= 0)
        {
            pio = last_pio;
            sm = ret;
            offset = last_offset;

            printf("Reusing program in last PIO block.\n");
            return true;
        }
    }

    if (!pio_claim_free_sm_and_add_program(&stepgen_program, &pio, &sm, &offset))
    {
        printf("Could not claim state machine!\n");
        return false;
    }

    if (pio == pio0)
    {
        irq_add_shared_handler(PIO0_IRQ_0, pio_rx_irq_handler, 0);
        irq_set_enabled(PIO0_IRQ_0, true);
    }
    else if (pio == pio1)
    {
        irq_add_shared_handler(PIO1_IRQ_0, pio_rx_irq_handler, 0);
        irq_set_enabled(PIO1_IRQ_0, true);
    }
    //FIXME: RP2350

    last_pio = pio;
    last_offset = offset;
    return true;
}

void PioStepgen::send_pio_command(uint32_t cmd)
{
    if (pio_sm_is_tx_fifo_full(pio, sm))
    {
        printf("tx is full!\n");
        return;
    }
    //printf("put %x\n", cmd);
    pio_sm_put(pio, sm, cmd);
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

    steplen = (uint32_t)ceil(double(steplen_ns)/100 - 3);
    stepspace = (uint32_t)ceil(double(stepspace_ns)/100 - 6);
    dirhold = (uint32_t)ceil(double(dirhold_ns)/100 - 3);
    dirsetup = (uint32_t)ceil(double(dirsetup_ns)/100);

    printf("steplen = %u, stepspace = %u, dirhold = %u, dirsetup = %u\n", steplen, stepspace, dirhold, dirsetup);

    if (!find_sm())
        return;

    printf("Claimed PIO %p, sm = %u, offset = %u\n", pio, sm, offset);

    pio_set_irq0_source_enabled(pio, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty << sm), true);

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

    stepgens.push_back(this);
    pio_sm_put_blocking(pio, sm, (uint32_t(lastDir) << 16) | 1);
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
        printf("fq = %d %d (%d)\n", enable, frequencyCommand, was_fired);
    if (!enable || 0 == frequencyCommand)
    {
        // Zero frequency command
        pio_sm_put_blocking(pio, sm, (uint32_t(lastDir) << 16) | 1);
        return;
    }
    bool dir = frequencyCommand > 0;
    if (dir != lastDir)
    {
        // DIR COMMAND
        pio_sm_put_blocking(pio, sm, (dirsetup << 17) | (uint32_t(dir) << 16) | (dirhold << 1) | 1);
        lastDir = dir;
    }

    uint32_t gap = ceil(frequencyCommand / (float)threadFrequency / 100.00587406015038 - 6.0);
    gap = min(steplen, gap);
    pio_sm_put_blocking(pio, sm, (gap << 11) | (steplen << 1));
}

int32_t PioStepgen::jointFeedback()
{
    return position;
}
