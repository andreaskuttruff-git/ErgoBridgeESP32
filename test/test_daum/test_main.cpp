// Host unit tests for the Daum protocol and the self-healing bike logic (pio test -e native).
// They mirror ErgoWattControl's DaumCockpitTests.cs and DaumErgometerTests.cs.

#include <unity.h>

#include <string>
#include <vector>

#include "DaumBike.h"
#include "DaumCockpit.h"
#include "SimulatedCockpit.h"

using namespace daum;

namespace {

class FakeClock : public Clock {
public:
    uint32_t now = 1000;
    uint32_t nowMs() override { return now; }
    void sleepMs(uint32_t ms) override { now += ms; }
};

std::vector<std::string> logLines;
void captureLog(const char* line) { logLines.emplace_back(line); }

int linesContaining(const char* text) {
    int n = 0;
    for (const auto& l : logLines)
        if (l.find(text) != std::string::npos)
            n++;
    return n;
}

struct Rig {
    FakeClock clock;
    SimulatedCockpit sim;
    Rig() { sim.useClock(clock); }
};

/// Runs the bike like its task does: call step(), wait what it says.
void run(DaumBike& bike, FakeClock& clock, int steps) {
    for (int i = 0; i < steps; i++)
        clock.sleepMs(bike.step());
}

} // namespace

void setUp() { logLines.clear(); }
void tearDown() {}

// ---- DaumCockpit -------------------------------------------------------------------------------------------------

void test_query_address_is_used_in_later_requests() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    TEST_ASSERT_EQUAL(Result::Ok, cockpit.queryAddress());
    TEST_ASSERT_EQUAL_UINT8(7, cockpit.address());
    RunData d;
    cockpit.readRunData(d);
    TEST_ASSERT_EQUAL_UINT8(0x40, rig.sim.requests.back()[0]);
    TEST_ASSERT_EQUAL_UINT8(7, rig.sim.requests.back()[1]);
}

void test_read_run_data_decodes_the_telegram() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    rig.sim.wattSteps = 30;
    rig.sim.rpm = 92;
    rig.sim.pulse = 135;

    RunData d;
    TEST_ASSERT_EQUAL(Result::Ok, cockpit.readRunData(d));
    TEST_ASSERT_EQUAL(0, d.program);
    TEST_ASSERT_TRUE(d.pedaling); // only bit 0 counts, the stray high bit is ignored
    TEST_ASSERT_EQUAL(150, d.watt);
    TEST_ASSERT_EQUAL(92, d.rpm);
    TEST_ASSERT_EQUAL(30, d.speedKmh);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.3f, d.distanceKm);
    TEST_ASSERT_EQUAL(300, d.timeSeconds);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, d.energyKJ);
    TEST_ASSERT_EQUAL(135, d.pulse);
    TEST_ASSERT_EQUAL(14, d.gear);
}

void test_set_watt_rounds_and_clamps() {
    const int cases[][2] = {{100, 100}, {103, 105}, {10, 25}, {900, 800}};
    for (const auto& c : cases) {
        Rig rig;
        DaumCockpit cockpit(rig.sim, rig.clock);
        cockpit.queryAddress();
        int accepted = 0;
        TEST_ASSERT_EQUAL(Result::Ok, cockpit.setWatt(c[0], accepted));
        TEST_ASSERT_EQUAL(c[1], accepted);
    }
}

void test_get_cockpit_type_reads_the_type_byte() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    uint8_t type = 0;
    TEST_ASSERT_EQUAL(Result::Ok, cockpit.getCockpitType(type));
    TEST_ASSERT_EQUAL_UINT8(40, type);
    TEST_ASSERT_EQUAL(800, DaumCockpit::maxWattForType(type));
    TEST_ASSERT_EQUAL(400, DaumCockpit::maxWattForType(10));
    TEST_ASSERT_EQUAL(400, DaumCockpit::maxWattForType(-1));
}

void test_missing_reply_is_a_timeout() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    rig.sim.inject(SimulatedCockpit::Fault::Silent);
    RunData d;
    const uint32_t before = rig.clock.now;
    TEST_ASSERT_EQUAL(Result::Timeout, cockpit.readRunData(d));
    TEST_ASSERT_GREATER_OR_EQUAL(DaumCockpit::ReplyTimeoutMs, rig.clock.now - before);
}

void test_reply_out_of_step_is_rejected() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    rig.sim.inject(SimulatedCockpit::Fault::WrongReply);
    RunData d;
    TEST_ASSERT_EQUAL(Result::Mismatch, cockpit.readRunData(d));
    TEST_ASSERT_EQUAL(Result::Ok, cockpit.readRunData(d)); // the next request works again
}

void test_cockpit_without_echo_is_not_checked() {
    Rig rig;
    rig.sim.echoes = false;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    TEST_ASSERT_FALSE(cockpit.echoes());
    rig.sim.inject(SimulatedCockpit::Fault::WrongReply);
    RunData d;
    TEST_ASSERT_EQUAL(Result::Ok, cockpit.readRunData(d));
}

void test_requests_keep_the_minimum_gap() {
    Rig rig;
    DaumCockpit cockpit(rig.sim, rig.clock);
    cockpit.queryAddress();
    const uint32_t before = rig.clock.now;
    RunData d;
    cockpit.readRunData(d); // the simulator answers instantly, so only the gap passes
    TEST_ASSERT_EQUAL_UINT32(DaumCockpit::MinGapMs, rig.clock.now - before);
}

// ---- DaumBike ----------------------------------------------------------------------------------------------------

void test_connects_selects_free_training_and_delivers_data() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    const BikeSnapshot s = bike.snapshot();
    TEST_ASSERT_TRUE(s.connected);
    TEST_ASSERT_EQUAL(40, s.cockpitType);
    TEST_ASSERT_EQUAL(800, s.maxWatt);
    TEST_ASSERT_EQUAL(1, rig.sim.count(0x23));
    TEST_ASSERT_EQUAL(1, rig.sim.count(0x21));
    TEST_ASSERT_EQUAL(100, rig.sim.wattSteps * 5); // the default target reached the cockpit
    TEST_ASSERT_EQUAL(80, s.data.rpm);
    TEST_ASSERT_EQUAL(1, linesContaining("Cockpit connected (address 7, type 8008, max. 800 W)"));
}

void test_cockpit_asleep_at_start_is_noted_once_and_the_wait_when_it_answers() {
    Rig rig;
    rig.sim.poweredOn = false;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 4);
    TEST_ASSERT_FALSE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(1, linesContaining("not answering"));
    rig.sim.poweredOn = true;
    run(bike, rig.clock, 1);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(1, linesContaining("connected (address 7"));
    TEST_ASSERT_EQUAL(1, linesContaining(" after "));
}

void test_target_set_before_connecting_keeps_its_value_on_an_8008() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    bike.setTargetWatt(600); // e.g. an app sends ERG targets while the cockpit is still asleep
    run(bike, rig.clock, 1);
    TEST_ASSERT_EQUAL(800, bike.snapshot().maxWatt);
    TEST_ASSERT_EQUAL(600, bike.targetWatt());
    TEST_ASSERT_EQUAL(600, rig.sim.wattSteps * 5);
}

void test_old_cockpit_without_get_version_caps_the_target_at_400_w() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    bike.setTargetWatt(600);
    run(bike, rig.clock, 1);
    TEST_ASSERT_EQUAL(600, rig.sim.wattSteps * 5);

    // The connection drops and comes back to a cockpit that can't tell its type.
    rig.sim.answersGetVersion = false;
    rig.sim.faultOnlyFor = 0x40;
    rig.sim.inject(SimulatedCockpit::Fault::Silent, 3);
    run(bike, rig.clock, 2);
    const BikeSnapshot s = bike.snapshot();
    TEST_ASSERT_TRUE(s.connected);
    TEST_ASSERT_EQUAL(2, s.connects);
    TEST_ASSERT_EQUAL(-1, s.cockpitType);
    TEST_ASSERT_EQUAL(400, s.maxWatt);
    TEST_ASSERT_EQUAL(400, bike.targetWatt());
    TEST_ASSERT_EQUAL(400, s.targetWatt);
    TEST_ASSERT_EQUAL(400, rig.sim.wattSteps * 5);
    bike.setTargetWatt(500); // stays capped for later targets too
    TEST_ASSERT_EQUAL(400, bike.targetWatt());
}

void test_target_power_reaches_the_cockpit() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    bike.setTargetWatt(152);
    TEST_ASSERT_EQUAL(150, bike.targetWatt()); // 5 W raster
    run(bike, rig.clock, 1);
    TEST_ASSERT_EQUAL(150, rig.sim.wattSteps * 5);
    bike.setTargetWatt(5000);
    run(bike, rig.clock, 1);
    TEST_ASSERT_EQUAL(800, rig.sim.wattSteps * 5);
    bike.setTargetWatt(0);
    run(bike, rig.clock, 1);
    TEST_ASSERT_EQUAL(25, rig.sim.wattSteps * 5);
}

void test_unchanged_target_is_not_sent_again() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    bike.setTargetWatt(150);
    run(bike, rig.clock, 1);
    const int setWatts = rig.sim.count(0x51);
    bike.setTargetWatt(150); // apps resend their ERG target all the time
    bike.setTargetWatt(152); // same 5 W step
    run(bike, rig.clock, 2);
    TEST_ASSERT_EQUAL(setWatts, rig.sim.count(0x51));
}

void test_two_missing_replies_are_asked_again_without_reconnecting() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    rig.sim.faultOnlyFor = 0x40;
    rig.sim.inject(SimulatedCockpit::Fault::Silent, 2);
    run(bike, rig.clock, 1);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(1, bike.snapshot().connects);
    TEST_ASSERT_EQUAL(1, linesContaining("answered after 2 retry"));
}

void test_three_missing_replies_count_as_lost_connection_and_reconnect() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    rig.sim.faultOnlyFor = 0x40;
    rig.sim.inject(SimulatedCockpit::Fault::Silent, 3);
    run(bike, rig.clock, 1);
    TEST_ASSERT_FALSE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(1, linesContaining("connection lost (no answer)"));
    run(bike, rig.clock, 1);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(2, bike.snapshot().connects);
    TEST_ASSERT_EQUAL(1, linesContaining("Cockpit reconnected"));
}

void test_reply_out_of_step_is_discarded_and_asked_again() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    rig.sim.faultOnlyFor = 0x40;
    rig.sim.inject(SimulatedCockpit::Fault::WrongReply);
    run(bike, rig.clock, 1);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(1, linesContaining("reply does not match"));
    TEST_ASSERT_EQUAL(1, linesContaining("answered after 1 retry"));
}

void test_cockpit_reset_is_noted_and_free_training_restored() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    bike.setTargetWatt(180);
    run(bike, rig.clock, 1);
    rig.sim.reset();
    run(bike, rig.clock, 2);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(0, rig.sim.program);
    TEST_ASSERT_EQUAL(180, rig.sim.wattSteps * 5);
    TEST_ASSERT_EQUAL(1, linesContaining("instead of free training - set up again (attempt 1)"));
}

void test_cockpit_stuck_in_program_leads_to_reconnect() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    rig.sim.stuckInProgram = 3;
    rig.sim.program = 3;
    run(bike, rig.clock, 5);
    TEST_ASSERT_EQUAL(1, linesContaining("(attempt 3)"));
    TEST_ASSERT_EQUAL(1, linesContaining("stays in a program"));
    TEST_ASSERT_EQUAL(2, bike.snapshot().connects); // rebuilt from scratch
}

void test_cockpit_drifting_from_target_is_corrected() {
    Rig rig;
    DaumBike bike(rig.sim, rig.clock, captureLog);
    run(bike, rig.clock, 2);
    const int setWattsBefore = rig.sim.count(0x51);
    rig.sim.wattSteps = 30; // cockpit on 150 W while the target is 100 W
    run(bike, rig.clock, DaumBike::MaxWattMismatches);
    TEST_ASSERT_EQUAL(setWattsBefore + 1, rig.sim.count(0x51));
    TEST_ASSERT_EQUAL(100, rig.sim.wattSteps * 5);
}

// ---- SimulatedCockpit (live, as in the simulator firmware) -------------------------------------------------------

void test_live_simulator_records_no_requests() {
    FakeClock clock;
    SimulatedCockpit sim;
    sim.enableLive(clock);
    DaumBike bike(sim, clock, captureLog);
    run(bike, clock, 100);
    TEST_ASSERT_TRUE(bike.snapshot().connected);
    TEST_ASSERT_EQUAL(0, static_cast<int>(sim.requests.size())); // runs for hours on the ESP32
}

void test_live_simulator_follows_the_target_after_the_response_time() {
    FakeClock clock;
    SimulatedCockpit sim;
    sim.enableLive(clock);
    DaumBike bike(sim, clock, captureLog);
    run(bike, clock, 2);
    bike.setTargetWatt(200);
    run(bike, clock, 1);
    TEST_ASSERT_EQUAL(100, bike.snapshot().data.watt); // not settled yet
    run(bike, clock, 6);                                // > 2 s at 400 ms per poll
    TEST_ASSERT_EQUAL(200, bike.snapshot().data.watt);
    TEST_ASSERT_INT_WITHIN(7, 88, bike.snapshot().data.rpm);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_query_address_is_used_in_later_requests);
    RUN_TEST(test_read_run_data_decodes_the_telegram);
    RUN_TEST(test_set_watt_rounds_and_clamps);
    RUN_TEST(test_get_cockpit_type_reads_the_type_byte);
    RUN_TEST(test_missing_reply_is_a_timeout);
    RUN_TEST(test_reply_out_of_step_is_rejected);
    RUN_TEST(test_cockpit_without_echo_is_not_checked);
    RUN_TEST(test_requests_keep_the_minimum_gap);
    RUN_TEST(test_connects_selects_free_training_and_delivers_data);
    RUN_TEST(test_cockpit_asleep_at_start_is_noted_once_and_the_wait_when_it_answers);
    RUN_TEST(test_target_set_before_connecting_keeps_its_value_on_an_8008);
    RUN_TEST(test_old_cockpit_without_get_version_caps_the_target_at_400_w);
    RUN_TEST(test_target_power_reaches_the_cockpit);
    RUN_TEST(test_unchanged_target_is_not_sent_again);
    RUN_TEST(test_two_missing_replies_are_asked_again_without_reconnecting);
    RUN_TEST(test_three_missing_replies_count_as_lost_connection_and_reconnect);
    RUN_TEST(test_reply_out_of_step_is_discarded_and_asked_again);
    RUN_TEST(test_cockpit_reset_is_noted_and_free_training_restored);
    RUN_TEST(test_cockpit_stuck_in_program_leads_to_reconnect);
    RUN_TEST(test_cockpit_drifting_from_target_is_corrected);
    RUN_TEST(test_live_simulator_records_no_requests);
    RUN_TEST(test_live_simulator_follows_the_target_after_the_response_time);
    return UNITY_END();
}
