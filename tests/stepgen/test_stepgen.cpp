#include <cassert>
#include <cstdio>
#include <cstdint>
#include <map>
#include <string>
#include "../../remora.h"

// --- Pin stub ---

std::map<std::string, bool> pinState;

struct TestPin
{
    std::string name;
    TestPin(std::string name, int) : name(name) {}
    void set(bool v) { pinState[name] = v; }
    bool get() const { return pinState.count(name) ? pinState.at(name) : false; }
};


struct Scenario
{
    RxPingPongBuffer rx;
    TxPingPongBuffer tx;

    Scenario()
      : rx{}
      , tx{}
    {
        // Scenarios are not parallelizable
        pinState.clear();
        rx.rxBuffers[0].jointEnable = 1;
    }

    Scenario& jointEnable(uint8_t value) { rx.rxBuffers[0].jointEnable = value; return *this; }
    Scenario& jointFreqCmd(int32_t value) { rx.rxBuffers[0].jointFreqCmd[0] = value; return *this; }
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

// ---

void test_disabled_joint_does_not_step()
{
    Scenario sc = Scenario()
        .jointEnable(0)
        .jointFreqCmd(THREAD_FREQ)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    sg.update();
    assert(!pinState[STEP_PIN]);
    printf("PASS: disabled joint does not step\n");
}

void test_zero_frequency_does_not_step()
{
    Scenario sc = Scenario()
        .jointFreqCmd(0)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    for (int i = 0; i < 1000; ++i)
        sg.update();
    assert(!pinState[STEP_PIN]);
    printf("PASS: zero frequency does not step\n");
}

void test_full_rate_steps_every_update()
{
    Scenario sc = Scenario()
        .jointFreqCmd(THREAD_FREQ)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    sg.update();
    assert(pinState[STEP_PIN]);
    sg.updatePost();
    assert(!pinState[STEP_PIN]);
    sg.update();
    assert(pinState[STEP_PIN]);
    printf("PASS: full-rate steps every update\n");
}

void test_half_rate_steps_every_two_updates()
{
    Scenario sc = Scenario()
        .jointFreqCmd(THREAD_FREQ / 2)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    sg.update();
    assert(!pinState[STEP_PIN]);
    sg.update();
    assert(pinState[STEP_PIN]);
    printf("PASS: half-rate steps every two updates\n");
}

void test_forward_direction_and_count()
{
    Scenario sc = Scenario()
        .jointFreqCmd(THREAD_FREQ)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    sg.update();
    assert(pinState[DIR_PIN]);
    assert(sc.tx.txBuffers[0].jointFeedback[0] == 1);
    printf("PASS: forward direction and count\n");
}

void test_reverse_direction_and_count()
{
    Scenario sc = Scenario()
        .jointFreqCmd(-THREAD_FREQ)
        ;
    TestStepgen sg(&sc.rx, &sc.tx, THREAD_FREQ, 0, STEP_PIN, DIR_PIN, 0, 0, 0, 0, 0);
    sg.update();
    assert(!pinState[DIR_PIN]);
    assert(sc.tx.txBuffers[0].jointFeedback[0] == -1);
    printf("PASS: reverse direction and count\n");
}

int main()
{
    test_disabled_joint_does_not_step();
    test_zero_frequency_does_not_step();
    test_full_rate_steps_every_update();
    test_half_rate_steps_every_two_updates();
    test_forward_direction_and_count();
    test_reverse_direction_and_count();
    printf("\nAll tests passed.\n");
    return 0;
}
