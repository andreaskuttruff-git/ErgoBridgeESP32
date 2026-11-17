#pragma once

#include <cstddef>
#include <cstdint>

// Byte layouts of the Bluetooth SIG services the bridge offers: Fitness Machine (FTMS, 0x1826), Cycling Power
// (0x1818) and Heart Rate (0x180D). Pure functions without BLE stack, so they run in host unit tests.
// All multi-byte fields are little-endian, as GATT requires.

namespace ftms {

// ---- UUIDs --------------------------------------------------------------------------------------------------------

constexpr uint16_t FitnessMachineService = 0x1826;
constexpr uint16_t FitnessMachineFeatureChar = 0x2ACC;
constexpr uint16_t IndoorBikeDataChar = 0x2AD2;
constexpr uint16_t TrainingStatusChar = 0x2AD3;
constexpr uint16_t SupportedPowerRangeChar = 0x2AD8;
constexpr uint16_t ControlPointChar = 0x2AD9;
constexpr uint16_t FitnessMachineStatusChar = 0x2ADA;

constexpr uint16_t CyclingPowerService = 0x1818;
constexpr uint16_t CyclingPowerMeasurementChar = 0x2A63;
constexpr uint16_t CyclingPowerFeatureChar = 0x2A65;
constexpr uint16_t SensorLocationChar = 0x2A5D;

constexpr uint16_t HeartRateService = 0x180D;
constexpr uint16_t HeartRateMeasurementChar = 0x2A37;

constexpr uint16_t DeviceInformationService = 0x180A;

// ---- Values the bike reports --------------------------------------------------------------------------------------

struct BikeValues {
    /// Instantaneous speed in km/h.
    float speedKmh = 0;
    /// Instantaneous cadence in rpm.
    float rpm = 0;
    int watt = 0;
    /// 0 = no heart rate (the field is left out).
    int pulse = 0;
    uint32_t distanceMeters = 0;
    uint16_t elapsedSeconds = 0;
};

// ---- Fitness Machine ----------------------------------------------------------------------------------------------

/// Fitness Machine Feature (0x2ACC): 4 bytes machine features, 4 bytes target setting features.
size_t encodeFitnessMachineFeature(uint8_t* out);

/// Indoor Bike Data (0x2AD2): speed, cadence, total distance, power, heart rate (if any), elapsed time. Max. 15 bytes.
size_t encodeIndoorBikeData(const BikeValues& values, uint8_t* out);

/// Supported Power Range (0x2AD8): minimum, maximum, increment in watt.
size_t encodeSupportedPowerRange(int minWatt, int maxWatt, int incrementWatt, uint8_t* out);

/// Advertising service data for 0x1826: "machine available" and the type "indoor bike" - apps filter on this.
size_t encodeFtmsAdvertisingData(uint8_t* out);

/// Control point op codes (FTMS 4.16.1).
enum class OpCode : uint8_t {
    RequestControl = 0x00,
    Reset = 0x01,
    SetTargetResistance = 0x04,
    SetTargetPower = 0x05,
    StartOrResume = 0x07,
    StopOrPause = 0x08,
    SetIndoorBikeSimulation = 0x11,
    ResponseCode = 0x80,
};

/// Control point result codes (FTMS 4.16.2.22).
enum class ResultCode : uint8_t {
    Success = 0x01,
    OpCodeNotSupported = 0x02,
    InvalidParameter = 0x03,
    OperationFailed = 0x04,
    ControlNotPermitted = 0x05,
};

/// Fitness Machine Status op codes (FTMS 4.17).
enum class StatusCode : uint8_t {
    Reset = 0x01,
    StoppedOrPausedByUser = 0x02,
    StartedOrResumedByUser = 0x04,
    TargetPowerChanged = 0x08,
    ControlPermissionLost = 0xFF,
};

/// Control point response: 0x80, the request's op code, the result.
size_t encodeControlPointResponse(uint8_t requestOpCode, ResultCode result, uint8_t* out);

// ---- Cycling Power ------------------------------------------------------------------------------------------------

/// Cycling Power Feature (0x2A65): crank revolution data supported.
size_t encodeCyclingPowerFeature(uint8_t* out);

/// Turns a cadence into the cumulative crank revolutions + last crank event time that Cycling Power Measurement
/// carries; apps and watches compute the cadence back from them.
class CrankRevolutions {
public:
    /// Advances by `elapsedMs` at `rpm`.
    void advance(float rpm, uint32_t elapsedMs);
    uint16_t revolutions() const { return revolutions_; }
    /// Time of the last whole revolution in 1/1024 s, wrapping at 64 s like the field.
    uint16_t lastEventTime1024() const { return lastEvent1024_; }

private:
    uint16_t revolutions_ = 0;
    uint16_t lastEvent1024_ = 0;
    /// Time since the start in 1/1024 s, kept as double so the fractions don't drift.
    double time1024_ = 0;
    /// Fraction of a revolution already ridden since the last whole one.
    double fraction_ = 0;
};

/// Cycling Power Measurement (0x2A63): instantaneous power + crank revolution data.
size_t encodeCyclingPowerMeasurement(int watt, const CrankRevolutions& crank, uint8_t* out);

// ---- Heart Rate ---------------------------------------------------------------------------------------------------

/// Heart Rate Measurement (0x2A37), 8-bit value.
size_t encodeHeartRateMeasurement(int pulse, uint8_t* out);

} // namespace ftms
