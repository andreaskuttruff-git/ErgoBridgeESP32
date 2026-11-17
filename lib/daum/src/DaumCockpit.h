#pragma once

#include "CockpitPort.h"

namespace daum {

/// Outcome of one request. Timeout and Mismatch are worth retrying; the cockpit discards stale input each time.
enum class Result : uint8_t {
    Ok,
    /// No complete answer in time.
    Timeout,
    /// An answer that doesn't start with our command and address: out of step, e.g. the rest of a late reply.
    Mismatch,
};

const char* toString(Result result);

/// The cockpit's Run_Data telegram (0x40).
struct RunData {
    uint8_t program = 0;
    uint8_t person = 0;
    bool pedaling = false;
    uint16_t watt = 0;
    uint8_t rpm = 0;
    uint8_t speedKmh = 0;
    float distanceKm = 0;
    uint16_t timeSeconds = 0;
    float energyKJ = 0;
    uint8_t pulse = 0;
    uint8_t pulseState = 0;
    uint8_t gear = 0;
};

/// Cockpit type byte from Get_Version (0x73).
enum CockpitType : uint8_t {
    TypeCardio = 10,
    TypeFitness = 20,
    TypeVitaDeLuxe = 30,
    Type8008 = 40,
    Type8080 = 50,
    TypeTherapie = 60,
};

/// Talks the daum electronic "ergo_bike" RS232 protocol (interface_2003_offiz191203.pdf), the subset needed for
/// free training (program 0: watt control + telemetry). Port of ErgoWattControl's DaumCockpit.cs.
class DaumCockpit {
public:
    /// The cockpit needs at least ~50 ms between packets or it can corrupt its own state.
    static constexpr uint32_t MinGapMs = 60;
    /// Longest wait for a complete answer.
    static constexpr uint32_t ReplyTimeoutMs = 1200;
    static constexpr int MinWatt = 25;
    static constexpr int WattStep = 5;

    DaumCockpit(CockpitPort& port, Clock& clock);

    uint8_t address() const { return address_; }
    /// True once the cockpit answered Get_Address with the command byte first; only then are replies checked.
    bool echoes() const { return echoes_; }

    Result queryAddress();
    /// Check_Cockpit (0x10): firmware/version byte.
    Result checkCockpit(uint8_t& version);
    /// Get_Version (0x73): cockpit type byte; very old cockpits don't answer (Timeout).
    Result getCockpitType(uint8_t& type);
    /// Switches to program 0 (free training) so the cockpit accepts Set_Watt.
    Result selectFreeTraining();
    Result startProgram(bool& started);
    Result stopProgram(bool& stopped);
    /// Sets the target power; `accepted` is the value the cockpit took (5 W steps).
    Result setWatt(int watt, int& accepted);
    Result readRunData(RunData& data);

    /// Highest power the cockpit type accepts: 800 W for the 8008/8080 series, otherwise 400 W.
    static int maxWattForType(int type);
    static const char* describeType(int type);
    /// Decodes a 19-byte Run_Data reply.
    static RunData parseRunData(const uint8_t* reply);

private:
    Result transact(const uint8_t* request, size_t requestLength, uint8_t* reply, size_t replyLength);
    void waitForMinGap();

    CockpitPort& port_;
    Clock& clock_;
    uint8_t address_ = 0;
    bool echoes_ = false;
    bool sentBefore_ = false;
    uint32_t lastSendMs_ = 0;
};

} // namespace daum
