#pragma once

#include <cstdint>

#include "DaumBike.h"
#include "FtmsCodec.h"
#include "TrainerControl.h"

class NimBLEServer;
class NimBLECharacteristic;

/// Offers the bike over Bluetooth LE: Fitness Machine (FTMS, ERG mode = target power), Cycling Power, Heart Rate,
/// Device Information, plus a small ErgoBridge status service (is the cockpit connected, which type).
class BleBridge {
public:
    static constexpr const char* DeviceName = "ErgoBridge";
    static constexpr const char* FirmwareVersion = "0.2.0";
    /// Live values go out this often (FTMS recommends 1 Hz).
    static constexpr uint32_t NotifyEveryMs = 1000;

    explicit BleBridge(daum::DaumBike& bike);

    void begin();
    /// Call from the main loop: sends live values, applies the disconnect fallback, refreshes ranges and status.
    void update(uint32_t nowMs);

    uint8_t connectedCount() const;
    uint16_t controller() const { return control_.controller(); }

    // Called from the NimBLE callbacks.
    void onConnect(uint16_t conn);
    void onDisconnect(uint16_t conn, int reason);
    void onControlPointWrite(uint16_t conn, const uint8_t* data, size_t length);

private:
    void sendLiveValues(const daum::BikeSnapshot& s, uint32_t nowMs);
    void refreshPowerRange(const daum::BikeSnapshot& s);
    void refreshStatus(const daum::BikeSnapshot& s);

    daum::DaumBike& bike_;
    ftms::TrainerControl control_;
    ftms::CrankRevolutions crank_;

    NimBLEServer* server_ = nullptr;
    NimBLECharacteristic* indoorBikeData_ = nullptr;
    NimBLECharacteristic* powerRange_ = nullptr;
    NimBLECharacteristic* controlPoint_ = nullptr;
    NimBLECharacteristic* machineStatus_ = nullptr;
    NimBLECharacteristic* powerMeasurement_ = nullptr;
    NimBLECharacteristic* heartRate_ = nullptr;
    NimBLECharacteristic* bridgeStatus_ = nullptr;

    uint32_t lastNotifyMs_ = 0;
    bool notifiedBefore_ = false;
    int publishedTargetLimit_ = -1;
    uint8_t publishedStatus_[9] = {};
    bool statusPublished_ = false;
};
