#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "../../remora.h"


// mini test harness

int failures = 0;

void fail(const char* msg, ...)
{
    va_list args;
    ++failures;
    printf("\n\e[31mFAILED\e[0m: ");
    va_start(args, msg);
    vprintf(msg, args);
    va_end(args);
}

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

// --- Pin stub ---

std::map<std::string, bool> pinState;

struct TestPin
{
    std::string name;
    TestPin(std::string name, int) : name(name) {}
    void set(bool v) { pinState[name] = v; }
    bool get() const { return pinState.count(name) ? pinState.at(name) : false; }
};


#define OUTPUT 42

#include "../../modules/stepgen/basic_stepgen.h"

// --- Buffer getter stubs (declared extern in remora.h) ---

rxData_t* getCurrentRxBuffer(RxPingPongBuffer* b) { return &b->rxBuffers[b->currentRxBuffer]; }
txData_t* getCurrentTxBuffer(TxPingPongBuffer* b) { return &b->txBuffers[b->currentTxBuffer]; }

using TestStepgen = BasicStepgen<TestPin>;

static const int32_t THREAD_FREQ = 40000;
static const char* STEP_PIN = "GP02";
static const char* DIR_PIN  = "GP03";

struct Sample
{
    bool step;
    bool dir;
};

class Scenario
{
private:
    RxPingPongBuffer rx;
    TxPingPongBuffer tx;
    std::vector<Sample> samples;

    std::pair<int, int> countPulses()
    {
        std::pair<int, int> result;
        bool last = false;
        for (auto const& sample : samples)
        {
            if (!last && sample.step)
            {
                if (sample.dir)
                    ++result.first;
                else
                    ++result.second;
            }
            last = sample.step;
        }
        return result;
    }

public:
    Scenario()
      : rx{}
      , tx{}
    {
        pinState.clear();
        rx.rxBuffers[0].jointEnable = 1;
    }

    Scenario& jointEnable(uint8_t value) { rx.rxBuffers[0].jointEnable = value; return *this; }
    Scenario& jointFreqCmd(int32_t value) { rx.rxBuffers[0].jointFreqCmd[0] = value; return *this; }

    Scenario& afterRunning1Second()
    {
        // Scenario runs are not parallelizable
        TestStepgen sg(&rx, &tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
        for (int i = 0; i < THREAD_FREQ; i++)
        {
            sg.update();
            samples.push_back(Sample{pinState[STEP_PIN], pinState[DIR_PIN]});
            sg.updatePost();
            samples.push_back(Sample{pinState[STEP_PIN], pinState[DIR_PIN]});
        }
        return *this;
    }

    Scenario& hasStepPulses(int expected)
    {
        auto pulses = countPulses();
        int actual = pulses.first + pulses.second;
        if (expected != actual)
            fail("expected %d pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasForwardStepPulses(int expected)
    {
        auto pulses = countPulses();
        int actual = pulses.first;
        if (expected != actual)
            fail("expected %d forward pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasReverseStepPulses(int expected)
    {
        auto pulses = countPulses();
        int actual = pulses.second;
        if (expected != actual)
            fail("expected %d reverse pulses, but got %d", expected, actual);
        return *this;
    }

    Scenario& hasJointFeedback(int expected)
    {
        int actual = tx.txBuffers[0].jointFeedback[0];
        if (expected != tx.txBuffers[0].jointFeedback[0])
            fail("expected jointFeedback of %d, but got %d", expected, actual);
        return *this;
    }
};

// ---

TEST(test_disabled_joint_does_not_step)
{
    Scenario()
        .jointEnable(0)
        .jointFreqCmd(100)
        .afterRunning1Second()
        .hasStepPulses(0)
        ;
}

TEST(test_zero_frequency_does_not_step)
{
    Scenario()
        .jointFreqCmd(0)
        .afterRunning1Second()
        .hasStepPulses(0)
        ;
}

TEST(test_full_rate_steps_every_update)
{
    Scenario()
        .jointFreqCmd(THREAD_FREQ)
        .afterRunning1Second()
        .hasStepPulses(THREAD_FREQ)
        ;
}

TEST(test_half_rate_steps_every_two_updates)
{
    Scenario()
        .jointFreqCmd(THREAD_FREQ / 2)
        .afterRunning1Second()
        .hasStepPulses(THREAD_FREQ / 2)
        ;
}

TEST(test_forward_direction_and_count)
{
    Scenario()
        .jointFreqCmd(THREAD_FREQ)
        .afterRunning1Second()
        .hasForwardStepPulses(THREAD_FREQ)
        .hasJointFeedback(THREAD_FREQ)
        ;
}

TEST(test_reverse_direction_and_count)
{
    Scenario()
        .jointFreqCmd(-THREAD_FREQ)
        .afterRunning1Second()
        .hasReverseStepPulses(THREAD_FREQ)
        .hasJointFeedback(-THREAD_FREQ)
        ;
}

int main()
{
    test_disabled_joint_does_not_step();
    test_zero_frequency_does_not_step();
    test_full_rate_steps_every_update();
    test_half_rate_steps_every_two_updates();
    test_forward_direction_and_count();
    test_reverse_direction_and_count();
    if (0 == failures)
        printf("\nAll tests passed.\n");
    exit(failures ? EXIT_FAILURE : EXIT_SUCCESS);
}
