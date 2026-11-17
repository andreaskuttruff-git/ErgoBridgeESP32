// Host unit tests for the FTMS / Cycling Power / Heart Rate byte layouts and the control point logic
// (pio test -e native). Expected bytes are worked out by hand from the Bluetooth SIG specifications.

#include <unity.h>

#include <vector>

#include "FtmsCodec.h"
#include "TrainerControl.h"

using namespace ftms;

namespace {

void assertBytes(const std::vector<uint8_t>& expected, const uint8_t* actual, size_t length) {
    TEST_ASSERT_EQUAL_size_t(expected.size(), length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), actual, expected.size());
}

/// A bike stand-in for the control logic: rounds to 5 W and limits to [25, 800] like DaumBike.
struct FakeBike {
    int target = 100;
    std::vector<int> applied;
    int apply(int watt) {
        int w = (watt + 2) / 5 * 5;
        if (w < 25) w = 25;
        if (w > 800) w = 800;
        target = w;
        applied.push_back(w);
        return w;
    }
};

struct ControlRig {
    FakeBike bike;
    TrainerControl control{[this](int w) { return bike.apply(w); }, [this] { return bike.target; }};

    TrainerControl::Reply write(uint16_t conn, std::vector<uint8_t> data, uint32_t now = 0) {
        return control.handleControlPoint(conn, data.data(), data.size(), now);
    }
};

} // namespace

void setUp() {}
void tearDown() {}

// ---- Encoding -----------------------------------------------------------------------------------------------------

void test_fitness_machine_feature() {
    uint8_t out[8];
    // features: cadence (1), total distance (2), heart rate (10), elapsed time (12), power (14) = 0x00005406
    // target settings: power target (3) = 0x00000008
    assertBytes({0x06, 0x54, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00}, out, encodeFitnessMachineFeature(out));
}

void test_indoor_bike_data_with_heart_rate() {
    BikeValues v;
    v.speedKmh = 30.0f;
    v.rpm = 88.5f;
    v.watt = 150;
    v.pulse = 135;
    v.distanceMeters = 12300;
    v.elapsedSeconds = 300;
    uint8_t out[20];
    // flags: cadence 0x04, distance 0x10, power 0x40, heart rate 0x200, elapsed time 0x800 = 0x0A54
    assertBytes({0x54, 0x0A,                  // flags
                 0xB8, 0x0B,                  // 3000 x 0.01 km/h
                 0xB1, 0x00,                  // 177 x 0.5 rpm
                 0x0C, 0x30, 0x00,            // 12300 m
                 0x96, 0x00,                  // 150 W
                 0x87,                        // 135 bpm
                 0x2C, 0x01},                 // 300 s
                out, encodeIndoorBikeData(v, out));
}

void test_indoor_bike_data_leaves_out_missing_heart_rate() {
    BikeValues v;
    v.watt = 100;
    uint8_t out[20];
    const size_t n = encodeIndoorBikeData(v, out);
    TEST_ASSERT_EQUAL_size_t(13, n);
    TEST_ASSERT_EQUAL_HEX8(0x54, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[1]); // heart rate bit clear
}

void test_supported_power_range() {
    uint8_t out[6];
    assertBytes({0x19, 0x00, 0x20, 0x03, 0x05, 0x00}, out, encodeSupportedPowerRange(25, 800, 5, out));
}

void test_advertising_data_says_indoor_bike() {
    uint8_t out[3];
    assertBytes({0x01, 0x20, 0x00}, out, encodeFtmsAdvertisingData(out));
}

void test_heart_rate_measurement() {
    uint8_t out[2];
    assertBytes({0x00, 0x78}, out, encodeHeartRateMeasurement(120, out));
}

void test_cycling_power_feature_and_measurement() {
    uint8_t out[8];
    assertBytes({0x08, 0x00, 0x00, 0x00}, out, encodeCyclingPowerFeature(out));
    CrankRevolutions crank;
    crank.advance(60, 2000); // two revolutions, the second ends exactly at 2 s = 2048/1024 s
    assertBytes({0x20, 0x00, 0xC8, 0x00, 0x02, 0x00, 0x00, 0x08}, out, encodeCyclingPowerMeasurement(200, crank, out));
}

void test_crank_revolutions_give_back_the_cadence() {
    CrankRevolutions crank;
    crank.advance(90, 1000);
    const uint16_t revs1 = crank.revolutions();
    const uint16_t time1 = crank.lastEventTime1024();
    for (int i = 0; i < 10; i++)
        crank.advance(90, 1000);
    const double rpm = (crank.revolutions() - revs1) * 60.0 * 1024.0 / static_cast<uint16_t>(crank.lastEventTime1024() - time1);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 90.0, rpm);
    TEST_ASSERT_EQUAL(16, crank.revolutions()); // 90 rpm for 11 s = 16.5 revolutions
}

void test_crank_event_time_wraps_at_64_seconds() {
    CrankRevolutions crank;
    for (int i = 0; i < 70; i++)
        crank.advance(60, 1000);
    TEST_ASSERT_EQUAL(70, crank.revolutions());
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(70 * 1024), crank.lastEventTime1024()); // 71680 mod 65536
}

void test_crank_stands_still_without_cadence() {
    CrankRevolutions crank;
    crank.advance(60, 1500);
    crank.advance(0, 5000);
    TEST_ASSERT_EQUAL(1, crank.revolutions());
    TEST_ASSERT_EQUAL_UINT16(1024, crank.lastEventTime1024());
}

// ---- Control point ------------------------------------------------------------------------------------------------

void test_target_power_needs_control_first() {
    ControlRig rig;
    auto reply = rig.write(1, {0x05, 0x96, 0x00});
    assertBytes({0x80, 0x05, 0x05}, reply.response, reply.responseLength); // control not permitted
    TEST_ASSERT_EQUAL_size_t(0, reply.statusLength);
    TEST_ASSERT_TRUE(rig.bike.applied.empty());
}

void test_request_control_then_set_target_power() {
    ControlRig rig;
    auto reply = rig.write(1, {0x00});
    assertBytes({0x80, 0x00, 0x01}, reply.response, reply.responseLength);
    TEST_ASSERT_EQUAL_UINT16(1, rig.control.controller());

    reply = rig.write(1, {0x05, 0x98, 0x00}); // 152 W
    assertBytes({0x80, 0x05, 0x01}, reply.response, reply.responseLength);
    assertBytes({0x08, 0x96, 0x00}, reply.status, reply.statusLength); // target power changed: 150 W as taken
    TEST_ASSERT_EQUAL(150, rig.bike.target);
}

void test_second_app_may_not_take_control() {
    ControlRig rig;
    rig.write(1, {0x00});
    auto reply = rig.write(2, {0x00});
    assertBytes({0x80, 0x00, 0x05}, reply.response, reply.responseLength);
    reply = rig.write(2, {0x05, 0xC8, 0x00});
    assertBytes({0x80, 0x05, 0x05}, reply.response, reply.responseLength);
    TEST_ASSERT_EQUAL(100, rig.bike.target);
}

void test_invalid_and_unsupported_requests() {
    ControlRig rig;
    rig.write(1, {0x00});
    auto reply = rig.write(1, {0x05, 0x96}); // power field cut short
    assertBytes({0x80, 0x05, 0x03}, reply.response, reply.responseLength);
    reply = rig.write(1, {0x05, 0xFF, 0xFF}); // -1 W
    assertBytes({0x80, 0x05, 0x03}, reply.response, reply.responseLength);
    reply = rig.write(1, {0x11, 0, 0, 0, 0, 0, 0}); // simulation: not offered yet
    assertBytes({0x80, 0x11, 0x02}, reply.response, reply.responseLength);
    reply = rig.write(1, {0x04, 0x10}); // resistance level: not offered
    assertBytes({0x80, 0x04, 0x02}, reply.response, reply.responseLength);
    reply = rig.write(1, {});
    TEST_ASSERT_EQUAL_size_t(0, reply.responseLength);
    TEST_ASSERT_TRUE(rig.bike.applied.empty());
}

void test_start_and_stop() {
    ControlRig rig;
    rig.write(1, {0x00});
    auto reply = rig.write(1, {0x07});
    assertBytes({0x80, 0x07, 0x01}, reply.response, reply.responseLength);
    assertBytes({0x04}, reply.status, reply.statusLength);
    reply = rig.write(1, {0x08, 0x02});
    assertBytes({0x80, 0x08, 0x01}, reply.response, reply.responseLength);
    assertBytes({0x02, 0x02}, reply.status, reply.statusLength);
    reply = rig.write(1, {0x08, 0x07});
    assertBytes({0x80, 0x08, 0x03}, reply.response, reply.responseLength);
}

void test_reset_lowers_the_target_and_keeps_control() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0x2C, 0x01}); // 300 W
    auto reply = rig.write(1, {0x01});
    assertBytes({0x80, 0x01, 0x01}, reply.response, reply.responseLength);
    assertBytes({0x01}, reply.status, reply.statusLength);
    TEST_ASSERT_EQUAL(TrainerControl::FallbackWatt, rig.bike.target);
    // Some apps go on without asking for control again.
    TEST_ASSERT_EQUAL_UINT16(1, rig.control.controller());
    reply = rig.write(1, {0x05, 0xC8, 0x00});
    assertBytes({0x80, 0x05, 0x01}, reply.response, reply.responseLength);
    TEST_ASSERT_EQUAL(200, rig.bike.target);
}

void test_reset_then_disconnect_leaves_no_hard_load() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0x2C, 0x01}); // 300 W
    rig.write(1, {0x01});              // end of the workout
    uint8_t status[3];
    assertBytes({0xFF}, status, rig.control.onDisconnect(1, 1000, status));
    TEST_ASSERT_EQUAL(TrainerControl::FallbackWatt, rig.bike.target);
}

void test_unsupported_request_without_control_says_not_supported() {
    ControlRig rig;
    auto reply = rig.write(1, {0x11, 0, 0, 0, 0, 0, 0});
    assertBytes({0x80, 0x11, 0x02}, reply.response, reply.responseLength);
}

void test_another_app_may_take_over_during_the_hold() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0xFA, 0x00}); // 250 W
    uint8_t status[3];
    rig.control.onDisconnect(1, 1000, status);
    auto reply = rig.write(2, {0x00}, 3000);
    assertBytes({0x80, 0x00, 0x01}, reply.response, reply.responseLength);
    TEST_ASSERT_EQUAL_UINT16(2, rig.control.controller());
    TEST_ASSERT_EQUAL_size_t(0, rig.control.tick(30000, status)); // the new app is in charge, no fallback
    TEST_ASSERT_EQUAL(250, rig.bike.target);
}

void test_disconnect_holds_the_target_then_falls_back() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0xFA, 0x00}); // 250 W
    uint8_t status[3];
    TEST_ASSERT_EQUAL_size_t(0, rig.control.onDisconnect(2, 1000, status)); // not the controller
    assertBytes({0xFF}, status, rig.control.onDisconnect(1, 1000, status));  // control permission lost
    TEST_ASSERT_EQUAL_size_t(0, rig.control.tick(1000 + TrainerControl::HoldAfterDisconnectMs - 1, status));
    TEST_ASSERT_EQUAL(250, rig.bike.target);
    assertBytes({0x08, 0x50, 0x00}, status, rig.control.tick(1000 + TrainerControl::HoldAfterDisconnectMs, status));
    TEST_ASSERT_EQUAL(TrainerControl::FallbackWatt, rig.bike.target);
    TEST_ASSERT_EQUAL_size_t(0, rig.control.tick(60000, status)); // only once
}

void test_reconnect_within_the_hold_keeps_the_target() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0xFA, 0x00});
    uint8_t status[3];
    rig.control.onDisconnect(1, 1000, status);
    auto reply = rig.write(3, {0x00}, 5000); // the app is back on a new connection
    assertBytes({0x80, 0x00, 0x01}, reply.response, reply.responseLength);
    TEST_ASSERT_EQUAL_size_t(0, rig.control.tick(30000, status));
    TEST_ASSERT_EQUAL(250, rig.bike.target);
}

void test_low_target_is_not_raised_by_the_fallback() {
    ControlRig rig;
    rig.write(1, {0x00});
    rig.write(1, {0x05, 0x32, 0x00}); // 50 W
    uint8_t status[3];
    rig.control.onDisconnect(1, 0, status);
    TEST_ASSERT_EQUAL_size_t(0, rig.control.tick(20000, status));
    TEST_ASSERT_EQUAL(50, rig.bike.target);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_fitness_machine_feature);
    RUN_TEST(test_indoor_bike_data_with_heart_rate);
    RUN_TEST(test_indoor_bike_data_leaves_out_missing_heart_rate);
    RUN_TEST(test_supported_power_range);
    RUN_TEST(test_advertising_data_says_indoor_bike);
    RUN_TEST(test_heart_rate_measurement);
    RUN_TEST(test_cycling_power_feature_and_measurement);
    RUN_TEST(test_crank_revolutions_give_back_the_cadence);
    RUN_TEST(test_crank_event_time_wraps_at_64_seconds);
    RUN_TEST(test_crank_stands_still_without_cadence);
    RUN_TEST(test_target_power_needs_control_first);
    RUN_TEST(test_request_control_then_set_target_power);
    RUN_TEST(test_second_app_may_not_take_control);
    RUN_TEST(test_invalid_and_unsupported_requests);
    RUN_TEST(test_start_and_stop);
    RUN_TEST(test_reset_lowers_the_target_and_keeps_control);
    RUN_TEST(test_reset_then_disconnect_leaves_no_hard_load);
    RUN_TEST(test_unsupported_request_without_control_says_not_supported);
    RUN_TEST(test_another_app_may_take_over_during_the_hold);
    RUN_TEST(test_disconnect_holds_the_target_then_falls_back);
    RUN_TEST(test_reconnect_within_the_hold_keeps_the_target);
    RUN_TEST(test_low_target_is_not_raised_by_the_fallback);
    return UNITY_END();
}
