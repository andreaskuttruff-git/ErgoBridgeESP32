#include "FtmsCodec.h"

#include <cmath>

namespace ftms {

namespace {

size_t putU8(uint8_t* out, unsigned value) {
    out[0] = static_cast<uint8_t>(value);
    return 1;
}

size_t putU16(uint8_t* out, unsigned value) {
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    return 2;
}

size_t putS16(uint8_t* out, int value) {
    return putU16(out, static_cast<uint16_t>(static_cast<int16_t>(value)));
}

size_t putU24(uint8_t* out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    return 3;
}

size_t putU32(uint8_t* out, uint32_t value) {
    putU16(out, value & 0xFFFF);
    putU16(out + 2, value >> 16);
    return 4;
}

unsigned clampUnsigned(double value, unsigned max) {
    if (!(value > 0)) return 0; // also NaN
    if (value > max) return max;
    return static_cast<unsigned>(std::lround(value));
}

int clampSigned16(int value) {
    if (value < -32768) return -32768;
    if (value > 32767) return 32767;
    return value;
}

// Fitness Machine Features (FTMS 4.3.1.1)
constexpr uint32_t FeatureCadence = 1u << 1;
constexpr uint32_t FeatureTotalDistance = 1u << 2;
constexpr uint32_t FeatureHeartRate = 1u << 10;
constexpr uint32_t FeatureElapsedTime = 1u << 12;
constexpr uint32_t FeaturePowerMeasurement = 1u << 14;
// Target Setting Features (FTMS 4.3.1.2)
constexpr uint32_t TargetPower = 1u << 3;

// Indoor Bike Data flags (FTMS 4.9.1.1); bit 0 clear = instantaneous speed present
constexpr uint16_t IbdCadence = 1u << 2;
constexpr uint16_t IbdTotalDistance = 1u << 4;
constexpr uint16_t IbdPower = 1u << 6;
constexpr uint16_t IbdHeartRate = 1u << 9;
constexpr uint16_t IbdElapsedTime = 1u << 11;

// Cycling Power (CPS 3.2 / 3.3)
constexpr uint16_t CpmCrankRevolutionData = 1u << 5;
constexpr uint32_t CpfCrankRevolutionData = 1u << 3;

} // namespace

size_t encodeFitnessMachineFeature(uint8_t* out) {
    size_t n = putU32(out, FeatureCadence | FeatureTotalDistance | FeatureHeartRate | FeatureElapsedTime |
                               FeaturePowerMeasurement);
    n += putU32(out + n, TargetPower);
    return n;
}

size_t encodeIndoorBikeData(const BikeValues& v, uint8_t* out) {
    uint16_t flags = IbdCadence | IbdTotalDistance | IbdPower | IbdElapsedTime;
    if (v.pulse > 0)
        flags |= IbdHeartRate;
    size_t n = putU16(out, flags);
    n += putU16(out + n, clampUnsigned(v.speedKmh * 100.0, 0xFFFF)); // 0.01 km/h
    n += putU16(out + n, clampUnsigned(v.rpm * 2.0, 0xFFFF));        // 0.5 rpm
    n += putU24(out + n, v.distanceMeters > 0xFFFFFF ? 0xFFFFFF : v.distanceMeters);
    n += putS16(out + n, clampSigned16(v.watt));
    if (v.pulse > 0)
        n += putU8(out + n, v.pulse > 255 ? 255 : static_cast<unsigned>(v.pulse));
    n += putU16(out + n, v.elapsedSeconds);
    return n;
}

size_t encodeSupportedPowerRange(int minWatt, int maxWatt, int incrementWatt, uint8_t* out) {
    size_t n = putS16(out, clampSigned16(minWatt));
    n += putS16(out + n, clampSigned16(maxWatt));
    n += putU16(out + n, static_cast<unsigned>(incrementWatt));
    return n;
}

size_t encodeFtmsAdvertisingData(uint8_t* out) {
    size_t n = putU8(out, 0x01);         // Fitness Machine Available
    n += putU16(out + n, 1u << 5);       // Fitness Machine Type: Indoor Bike
    return n;
}

size_t encodeControlPointResponse(uint8_t requestOpCode, ResultCode result, uint8_t* out) {
    out[0] = static_cast<uint8_t>(OpCode::ResponseCode);
    out[1] = requestOpCode;
    out[2] = static_cast<uint8_t>(result);
    return 3;
}

size_t encodeCyclingPowerFeature(uint8_t* out) {
    return putU32(out, CpfCrankRevolutionData);
}

void CrankRevolutions::advance(float rpm, uint32_t elapsedMs) {
    const double elapsed1024 = elapsedMs * 1.024;
    if (rpm > 0 && elapsedMs > 0) {
        fraction_ += static_cast<double>(rpm) * elapsedMs / 60000.0;
        if (fraction_ >= 1) {
            const unsigned whole = static_cast<unsigned>(fraction_);
            fraction_ -= whole;
            revolutions_ = static_cast<uint16_t>(revolutions_ + whole);
            // The last whole revolution ended `fraction_` revolutions before now.
            const double eventTime1024 = time1024_ + elapsed1024 - fraction_ * 60000.0 / rpm * 1.024;
            lastEvent1024_ = static_cast<uint16_t>(static_cast<uint64_t>(std::llround(eventTime1024)) & 0xFFFF);
        }
    } else {
        fraction_ = 0; // standing still: the next revolution starts from the crank's position when pedalling resumes
    }
    time1024_ += elapsed1024;
}

size_t encodeCyclingPowerMeasurement(int watt, const CrankRevolutions& crank, uint8_t* out) {
    size_t n = putU16(out, CpmCrankRevolutionData);
    n += putS16(out + n, clampSigned16(watt));
    n += putU16(out + n, crank.revolutions());
    n += putU16(out + n, crank.lastEventTime1024());
    return n;
}

size_t encodeHeartRateMeasurement(int pulse, uint8_t* out) {
    out[0] = 0x00; // 8-bit value, no sensor contact information
    out[1] = static_cast<uint8_t>(pulse < 0 ? 0 : pulse > 255 ? 255 : pulse);
    return 2;
}

} // namespace ftms
