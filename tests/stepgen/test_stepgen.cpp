#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "../../remora.h"


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

#include "../../modules/stepgen/basic_stepgen.h"

struct TestIO
{
    struct Command
    {
        int      tick;
        uint32_t cycles;
        PinType  pin;
        bool     value;
    };

    std::vector<Command> commands;
    int currentTick = 0;

    TestIO(std::string, std::string) {}

    void schedule(uint32_t cycles, PinType pin, bool value)
    {
        commands.push_back({currentTick, cycles, pin, value});
    }
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

class Scenario
{
private:
    int32_t threadFreq;
    int32_t steplen;
    int32_t stepspace;
    int32_t dirsetup;
    int32_t dirhold;
    int32_t dirdelay;
    std::optional<TestStepgen> stepgen;

    void start()
    {
        if (!stepgen.has_value())
            stepgen.emplace(threadFreq, 0, STEP_PIN, DIR_PIN, steplen, stepspace, dirsetup, dirhold, dirdelay);
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
      : threadFreq(THREAD_FREQ)
      , steplen(0)
      , stepspace(0)
      , dirsetup(0)
      , dirhold(0)
      , dirdelay(0)
    {
    }

    Scenario& dumpCalls(int n = 20)
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
            printf("    tick=%d %s=%s\n", c.tick, pinName, c.value ? "true" : "false");
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
        {
            ++stepgen->io().currentTick;
            stepgen->update();
        }
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
            ++testIO.currentTick;
            stepgen->update();
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

    Scenario& madePulsesOfLength(int length)
    {
        int n = 0;
        int riseTick = -1;
        for (auto const& c : stepgen->io().commands)
        {
            if (c.pin != PinType::StepPin) continue;
            if (c.value)
            {
                riseTick = c.tick;
            }
            else if (riseTick >= 0)
            {
                int actualLength = c.tick - riseTick;
                if (actualLength != length)
                {
                    fail("pulse %d had length %d (expected length %d).", n, actualLength, length);
                    return *this;
                }
                ++n;
                riseTick = -1;
            }
        }
        return *this;
    }

    Scenario& sentCommands(std::vector<TestIO::Command> expected)
    {
        auto const& actual = stepgen->io().commands;
        bool ok = (actual.size() == expected.size());
        for (size_t i = 0; ok && i < expected.size(); ++i)
        {
            auto const& a = actual[i];
            auto const& e = expected[i];
            if (a.tick != e.tick || a.cycles != e.cycles || a.pin != e.pin || a.value != e.value)
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
            {   0, 0, PinType::DirectionPin, false},
            {   1, 0, PinType::DirectionPin, true},
            {1601, 0, PinType::StepPin,      true},
            {1603, 0, PinType::StepPin,      false},
        })
        .madePulsesOfLength(2)
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
            {0, 0, PinType::DirectionPin, false},
            {1, 0, PinType::DirectionPin, true},
            {7, 0, PinType::StepPin,      true},
            {9, 0, PinType::StepPin,      false},
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
            { 0, 0, PinType::DirectionPin, false},
            { 1, 0, PinType::StepPin,      true},
            { 3, 0, PinType::StepPin,      false},
            { 9, 0, PinType::DirectionPin, true},
            {12, 0, PinType::StepPin,      true},
            {14, 0, PinType::StepPin,      false},
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
            {0, 0, PinType::DirectionPin, false},
            {1, 0, PinType::StepPin,      true},
            {3, 0, PinType::StepPin,      false},
            {4, 0, PinType::DirectionPin, true},
            {7, 0, PinType::StepPin,      true},
            {9, 0, PinType::StepPin,      false},
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
