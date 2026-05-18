#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "../../modules/stepgen/basic_stepgen.h"

// mini test harness

int failures = 0;

#define TEST(name) \
    void name##_impl_(void); \
    void name(void) \
    { \
        printf(#name "... "); \
        fflush(stdout); \
        int failures_on_entry = failures; \
        name##_impl_(); \
        if (failures == failures_on_entry) \
           printf("ok.\n"); \
        else \
           printf("\n\n"); \
    } \
    void name##_impl_(void)

// --- IO stub ---

struct TestIO
{
    struct Command
    {
        uint32_t cycles;
        PinType  pin;
        bool     value;
    };

    std::vector<Command> commands;
    uint32_t cycleCounter = 0;

    TestIO(std::string, std::string) {}

    void schedule(uint32_t cycles, PinType pin, bool value)
    {
        cycleCounter += cycles;
        commands.push_back({cycleCounter, pin, value});
    }
};

struct ExpectedCommand
{
    uint32_t cycles;
    PinType pin;
    bool value;
};

class TestStepgen : public BasicStepgen<TestIO>
{
public:
    using BasicStepgen::BasicStepgen;
    TestIO& io() { return *this; }
};

static const int32_t THREAD_FREQ = 40000;
static const char* STEP_PIN = "GP02";
static const char* DIR_PIN  = "GP03";

static const int32_t CPU_FREQ = 125000000;
static const uint32_t CYCLES_PER_TICK = CPU_FREQ / THREAD_FREQ;

class Scenario
{
private:
    int32_t cpuFreq;
    int32_t threadFreq;
    int32_t steplen;
    int32_t stepspace;
    int32_t dirsetup;
    int32_t dirhold;
    int32_t dirdelay;
    bool reportedScheduleFailure;
    std::optional<TestStepgen> stepgen;

    void start()
    {
        if (!stepgen.has_value())
            stepgen.emplace(cpuFreq, threadFreq, 0, STEP_PIN, DIR_PIN, steplen, stepspace, dirsetup, dirhold, dirdelay);
    }

    void callUpdate()
    {
        auto& testIO = stepgen->io();
        size_t before = testIO.commands.size();
        stepgen->update();
        if (testIO.commands.size() == before && !reportedScheduleFailure)
        {
            reportedScheduleFailure = true;
            fail("update() emitted no commands (always-schedule invariant violated)");
        }
    }

    int countPulses(std::optional<bool> forward = {})
    {
        int n = 0;
        bool dir = false;
        for (auto const& c : stepgen->io().commands)
        {
            if (c.pin == PinType::DirectionPin)
                dir = c.value;
            else if (c.pin == PinType::StepPin && c.value)
                if (!forward.has_value() || *forward == dir)
                    ++n;
        }
        return n;
    }

    void fail(const char *msg, ...)
    {
        va_list args;
        ++failures;
        printf("\n\n\e[31mFAILED\e[0m: ");
        va_start(args, msg);
        vprintf(msg, args);
        va_end(args);

        dumpCalls();
    }

public:
    Scenario()
      : cpuFreq(CPU_FREQ)
      , threadFreq(THREAD_FREQ)
      , steplen(0)
      , stepspace(0)
      , dirsetup(0)
      , dirhold(0)
      , dirdelay(0)
      , reportedScheduleFailure(false)
    {
    }

    Scenario& dumpCalls(int n = 40)
    {
        auto const& calls = stepgen->io().commands;
        int count = std::min(n, int(calls.size()));
        printf("\n  Calls (%d of %d):\n", count, int(calls.size()));
        for (int i = 0; i < count; ++i)
        {
            auto const& c = calls[i];
            const char* pinName =
                (c.pin == PinType::StepPin) ? "Step" :
                (c.pin == PinType::DirectionPin) ? "Dir" : "NoPin";
            printf("    cycles=%u %s=%s\n",
                c.cycles, pinName, c.value ? "true" : "false");
        }
        return *this;
    }

    Scenario& withThreadFrequency(int32_t value) { threadFreq = value; return *this; }
    Scenario& withSteplen(int32_t value) { steplen = value; return *this; }
    Scenario& withStepspace(int32_t value) { stepspace = value; return *this; }
    Scenario& withDirsetup(int32_t value) { dirsetup = value; return *this; }
    Scenario& withDirhold(int32_t value) { dirhold = value; return *this; }
    Scenario& withDirdelay(int32_t value) { dirdelay = value; return *this; }

    Scenario& withFrequency(int32_t frequency, bool enabled = true)
    {
        start();
        stepgen->setFrequency(frequency, enabled);
        return *this;
    }

    Scenario& afterRunning1Second()
    {
        start();
        for (int i = 0; i < threadFreq; i++)
            callUpdate();
        return *this;
    }

    Scenario& afterPulses(int n)
    {
        start();
        auto& testIO = stepgen->io();
        int seen = 0;
        for (int i = 0; i < threadFreq; ++i)
        {
            size_t before = testIO.commands.size();
            callUpdate();
            for (size_t j = before; j < testIO.commands.size(); ++j)
            {
                auto const& c = testIO.commands[j];
                if (c.pin == PinType::StepPin && !c.value)
                    if (++seen == n)
                        return *this;
            }
        }
        fail("did not receive %d pulses (saw %d)", n, seen);
        return *this;
    }

    Scenario& hasStepPulses(int expected)
    {
        int actual = countPulses();
        if (expected != actual)
            fail("expected %d pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasForwardStepPulses(int expected)
    {
        int actual = countPulses(true);
        if (expected != actual)
            fail("expected %d forward pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasReverseStepPulses(int expected)
    {
        int actual = countPulses(false);
        if (expected != actual)
            fail("expected %d reverse pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasJointFeedback(int expected)
    {
        int actual = stepgen->getRawCount();
        if (expected != actual)
            fail("expected jointFeedback of %d, but got %d", expected, actual);
        return *this;
    }

    Scenario& madePulsesOfLength(uint32_t length)
    {
        int n = 0;
        uint32_t riseCycles = 0;
        bool haveRise = false;
        for (auto const& c : stepgen->io().commands)
        {
            if (c.pin != PinType::StepPin) continue;
            if (c.value)
            {
                riseCycles = c.cycles;
                haveRise = true;
            }
            else if (haveRise)
            {
                uint32_t actualLength = c.cycles - riseCycles;
                if (actualLength != length)
                {
                    fail("pulse %d had length %u (expected length %u).", n, actualLength, length);
                    return *this;
                }
                ++n;
                haveRise = false;
            }
        }
        return *this;
    }

    Scenario& sentCommands(std::vector<ExpectedCommand> expected)
    {
        std::vector<TestIO::Command> actual;
        for (auto const& c : stepgen->io().commands)
            if (c.pin != PinType::NoPin)
                actual.push_back(c);
        bool ok = (actual.size() == expected.size());
        for (size_t i = 0; ok && i < expected.size(); ++i)
        {
            auto const& a = actual[i];
            auto const& e = expected[i];
            if (a.cycles != e.cycles || a.pin != e.pin || a.value != e.value)
                ok = false;
        }
        if (!ok)
            fail("produced the wrong calls");
        return *this;
    }
};

// ---

TEST(test_disabled_joint_does_not_step)
{
    Scenario()
        .withFrequency(100, false)
        .afterRunning1Second()
        .hasStepPulses(0)
        ;
}

TEST(test_zero_frequency_does_not_step)
{
    Scenario()
        .withFrequency(0, true)
        .afterRunning1Second()
        .hasStepPulses(0)
        ;
}

TEST(test_half_rate_steps_every_two_updates)
{
    Scenario()
        .withFrequency(THREAD_FREQ / 2, true)
        .afterRunning1Second()
        .hasStepPulses(THREAD_FREQ / 2)
        ;
}

TEST(test_forward_direction_and_count)
{
    Scenario()
        .withFrequency(THREAD_FREQ / 2, true)
        .afterRunning1Second()
        .hasForwardStepPulses(THREAD_FREQ / 2)
        .hasJointFeedback(THREAD_FREQ / 2)
        ;
}

TEST(test_reverse_direction_and_count)
{
    Scenario()
        .withFrequency(-THREAD_FREQ / 2, true)
        .afterRunning1Second()
        .hasReverseStepPulses(THREAD_FREQ / 2)
        .hasJointFeedback(-THREAD_FREQ / 2)
        ;
}

TEST(test_steplen_greater_than_frequency_keeps_pulse_high_for_multiple_ticks)
{
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withFrequency(25)
        .afterPulses(1)
        .sentCommands({
            {0,                      PinType::DirectionPin, false},
            {0,                      PinType::DirectionPin, true},
            {1600 * CYCLES_PER_TICK, PinType::StepPin,      true},
            {1602 * CYCLES_PER_TICK, PinType::StepPin,      false},
        })
        .madePulsesOfLength(2 * CYCLES_PER_TICK)
        ;
}

TEST(test_clamps_maximum_frequency_to_honor_steplen_and_stepspace)
{
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withStepspace(50000)
        .withFrequency(THREAD_FREQ, true)
        .afterRunning1Second()
        .hasStepPulses(10000)
        ;
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withStepspace(50000)
        .withFrequency(-THREAD_FREQ, true)
        .afterRunning1Second()
        .hasStepPulses(10000)
        ;
}

TEST(test_waits_dirsetup_before_pulsing)
{
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withStepspace(50000)
        .withDirsetup(150000)
        .withFrequency(THREAD_FREQ/2)
        .afterPulses(1)
        .sentCommands({
            {0,                   PinType::DirectionPin, false},
            {0,                   PinType::DirectionPin, true},
            {6 * CYCLES_PER_TICK, PinType::StepPin,      true},
            {8 * CYCLES_PER_TICK, PinType::StepPin,      false},
        });
}

TEST(test_waits_dirhold_before_changing_direction)
{
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withStepspace(50000)
        .withDirsetup(75000)
        .withDirhold(150000)
        .withFrequency(-THREAD_FREQ/2)
        .afterPulses(1)
        .withFrequency(THREAD_FREQ/2)
        .afterPulses(1)
        .sentCommands({
            { 0,                    PinType::DirectionPin, false},
            { 0,                    PinType::StepPin,      true},
            { 2 * CYCLES_PER_TICK,  PinType::StepPin,      false},
            { 8 * CYCLES_PER_TICK,  PinType::DirectionPin, true},
            {11 * CYCLES_PER_TICK,  PinType::StepPin,      true},
            {13 * CYCLES_PER_TICK,  PinType::StepPin,      false},
        });
}

TEST(test_waits_dirdelay_before_emitting_a_pulse_in_the_opposite_direction)
{
    Scenario()
        .withThreadFrequency(40000)
        .withSteplen(50000)
        .withStepspace(50000)
        .withDirdelay(150000)
        .withFrequency(-THREAD_FREQ/2)
        .afterPulses(1)
        .withFrequency(THREAD_FREQ/2)
        .afterPulses(1)
        .sentCommands({
            {0,                   PinType::DirectionPin, false},
            {0,                   PinType::StepPin,      true},
            {2 * CYCLES_PER_TICK, PinType::StepPin,      false},
            {3 * CYCLES_PER_TICK, PinType::DirectionPin, true},
            {6 * CYCLES_PER_TICK, PinType::StepPin,      true},
            {8 * CYCLES_PER_TICK, PinType::StepPin,      false},
        });
}

int main()
{
    test_disabled_joint_does_not_step();
    test_zero_frequency_does_not_step();
    test_half_rate_steps_every_two_updates();
    test_forward_direction_and_count();
    test_reverse_direction_and_count();
    test_steplen_greater_than_frequency_keeps_pulse_high_for_multiple_ticks();
    test_clamps_maximum_frequency_to_honor_steplen_and_stepspace();
    test_waits_dirsetup_before_pulsing();
    test_waits_dirhold_before_changing_direction();
    test_waits_dirdelay_before_emitting_a_pulse_in_the_opposite_direction();
    if (0 == failures)
        printf("\nAll tests passed.\n");
    exit(failures ? EXIT_FAILURE : EXIT_SUCCESS);
}
