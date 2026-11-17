#include "BleBridge.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <cstring>

namespace {

/// ErgoBridge status service (own 128-bit UUIDs): lets an app tell "bridge there, cockpit missing" apart.
constexpr const char* BridgeServiceUuid = "c7b6d000-4e2a-4f69-9a5e-2d0e8b3f1a01";
/// Read/notify, 9 bytes: version (1), flags (bit 0: cockpit connected), cockpit type (int8, -1 = unknown),
/// max. watt of the cockpit (uint16), current target limit (uint16), target watt (uint16).
constexpr const char* BridgeStatusCharUuid = "c7b6d001-4e2a-4f69-9a5e-2d0e8b3f1a01";

/// Sensor location "rear hub": the eddy-current brake sits at the flywheel.
constexpr uint8_t SensorLocationRearHub = 13;

BleBridge* instance = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo& info) override { instance->onConnect(info.getConnHandle()); }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo& info, int reason) override {
        instance->onDisconnect(info.getConnHandle(), reason);
    }
};

class ControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& info) override {
        const NimBLEAttValue value = characteristic->getValue();
        instance->onControlPointWrite(info.getConnHandle(), value.data(), value.size());
    }
};

ServerCallbacks serverCallbacks;
ControlPointCallbacks controlPointCallbacks;

void setFixedValue(NimBLECharacteristic* c, const uint8_t* data, size_t length) {
    c->setValue(data, length);
}

/// Stores the value and notifies only the connections that subscribed to it. notify(data, length) would send to
/// every connected peer regardless of its subscription (NimBLE 2.x).
void notifySubscribers(NimBLECharacteristic* c, const uint8_t* data, size_t length) {
    c->setValue(data, length);
    c->notify();
}

} // namespace

BleBridge::BleBridge(daum::DaumBike& bike)
    : bike_(bike),
      control_([&bike](int watt) {
                   bike.setTargetWatt(watt);
                   return bike.targetWatt();
               },
               [&bike] { return bike.targetWatt(); }) {}

uint8_t BleBridge::connectedCount() const {
    return server_ == nullptr ? 0 : server_->getConnectedCount();
}

void BleBridge::begin() {
    instance = this;
    NimBLEDevice::init(DeviceName);
    server_ = NimBLEDevice::createServer();
    server_->setCallbacks(&serverCallbacks, false);
    server_->advertiseOnDisconnect(true);

    uint8_t buffer[16];

    // Fitness Machine
    NimBLEService* fm = server_->createService(NimBLEUUID(ftms::FitnessMachineService));
    setFixedValue(fm->createCharacteristic(NimBLEUUID(ftms::FitnessMachineFeatureChar), NIMBLE_PROPERTY::READ),
                  buffer, ftms::encodeFitnessMachineFeature(buffer));
    indoorBikeData_ = fm->createCharacteristic(NimBLEUUID(ftms::IndoorBikeDataChar), NIMBLE_PROPERTY::NOTIFY);
    powerRange_ = fm->createCharacteristic(NimBLEUUID(ftms::SupportedPowerRangeChar), NIMBLE_PROPERTY::READ);
    controlPoint_ = fm->createCharacteristic(NimBLEUUID(ftms::ControlPointChar),
                                             NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
    controlPoint_->setCallbacks(&controlPointCallbacks);
    machineStatus_ = fm->createCharacteristic(NimBLEUUID(ftms::FitnessMachineStatusChar), NIMBLE_PROPERTY::NOTIFY);
    refreshPowerRange(bike_.snapshot());

    // Cycling Power
    NimBLEService* cp = server_->createService(NimBLEUUID(ftms::CyclingPowerService));
    powerMeasurement_ =
        cp->createCharacteristic(NimBLEUUID(ftms::CyclingPowerMeasurementChar), NIMBLE_PROPERTY::NOTIFY);
    setFixedValue(cp->createCharacteristic(NimBLEUUID(ftms::CyclingPowerFeatureChar), NIMBLE_PROPERTY::READ),
                  buffer, ftms::encodeCyclingPowerFeature(buffer));
    setFixedValue(cp->createCharacteristic(NimBLEUUID(ftms::SensorLocationChar), NIMBLE_PROPERTY::READ),
                  &SensorLocationRearHub, 1);

    // Heart Rate
    NimBLEService* hr = server_->createService(NimBLEUUID(ftms::HeartRateService));
    heartRate_ = hr->createCharacteristic(NimBLEUUID(ftms::HeartRateMeasurementChar), NIMBLE_PROPERTY::NOTIFY);

    // Device Information
    NimBLEService* di = server_->createService(NimBLEUUID(ftms::DeviceInformationService));
    di->createCharacteristic(NimBLEUUID(static_cast<uint16_t>(0x2A29)), NIMBLE_PROPERTY::READ)->setValue("ErgoBridge");
    di->createCharacteristic(NimBLEUUID(static_cast<uint16_t>(0x2A24)), NIMBLE_PROPERTY::READ)
        ->setValue("Daum ergo_bike bridge");
    di->createCharacteristic(NimBLEUUID(static_cast<uint16_t>(0x2A26)), NIMBLE_PROPERTY::READ)
        ->setValue(FirmwareVersion);

    // ErgoBridge status
    NimBLEService* bs = server_->createService(BridgeServiceUuid);
    bridgeStatus_ = bs->createCharacteristic(BridgeStatusCharUuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    refreshStatus(bike_.snapshot());

    server_->start();

    // Advertising: everything in the advertisement itself (30 of 31 bytes), because passive scanners - Windows'
    // default, many watches - never see a scan response. FTMS service data tells apps "indoor bike".
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData data;
    data.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    data.addServiceUUID(NimBLEUUID(ftms::FitnessMachineService));
    data.addServiceUUID(NimBLEUUID(ftms::CyclingPowerService));
    data.addServiceUUID(NimBLEUUID(ftms::HeartRateService));
    data.setServiceData(NimBLEUUID(ftms::FitnessMachineService), buffer, ftms::encodeFtmsAdvertisingData(buffer));
    data.setName(DeviceName);
    if (!advertising->setAdvertisementData(data))
        Serial.println("BLE: advertisement data too long");
    advertising->enableScanResponse(false);
    advertising->start();
    Serial.println("BLE: advertising as \"ErgoBridge\" (FTMS, Cycling Power, Heart Rate)");
}

void BleBridge::onConnect(uint16_t conn) {
    Serial.printf("BLE: connection %u opened (%u open)\n", conn, connectedCount());
    // Keep advertising so a second app (e.g. a watch for power and pulse) can connect too.
    NimBLEDevice::startAdvertising();
}

void BleBridge::onDisconnect(uint16_t conn, int reason) {
    uint8_t status[3];
    const size_t n = control_.onDisconnect(conn, millis(), status);
    Serial.printf("BLE: connection %u closed (reason 0x%X)%s\n", conn, reason,
                  n > 0 ? " - it had control, target kept for 10 s" : "");
    if (n > 0)
        notifySubscribers(machineStatus_, status, n);
}

void BleBridge::onControlPointWrite(uint16_t conn, const uint8_t* data, size_t length) {
    const ftms::TrainerControl::Reply reply = control_.handleControlPoint(conn, data, length, millis());
    if (reply.responseLength > 0) {
        controlPoint_->indicate(reply.response, reply.responseLength, conn);
        Serial.printf("BLE: control point op 0x%02X from %u -> result %u\n", length > 0 ? data[0] : 0, conn,
                      reply.response[2]);
    }
    if (reply.statusLength > 0)
        notifySubscribers(machineStatus_, reply.status, reply.statusLength);
}

void BleBridge::update(uint32_t nowMs) {
    uint8_t status[3];
    const size_t n = control_.tick(nowMs, status);
    if (n > 0) {
        notifySubscribers(machineStatus_, status, n);
        Serial.printf("BLE: no app in control for 10 s - target lowered to %d W\n", bike_.targetWatt());
    }

    const daum::BikeSnapshot s = bike_.snapshot();
    refreshPowerRange(s);
    refreshStatus(s);
    if (!notifiedBefore_ || nowMs - lastNotifyMs_ >= NotifyEveryMs)
        sendLiveValues(s, nowMs);
}

void BleBridge::sendLiveValues(const daum::BikeSnapshot& s, uint32_t nowMs) {
    const uint32_t elapsed = notifiedBefore_ ? nowMs - lastNotifyMs_ : 0;
    lastNotifyMs_ = nowMs;
    notifiedBefore_ = true;

    ftms::BikeValues v;
    if (s.connected) {
        v.rpm = s.data.rpm;
        // Standing still the cockpit still shows its set power; nobody produces it, so report 0.
        v.watt = s.data.rpm > 0 ? s.data.watt : 0;
        v.speedKmh = s.data.speedKmh;
        v.pulse = s.data.pulse;
        v.distanceMeters = static_cast<uint32_t>(s.data.distanceKm * 1000.0f + 0.5f);
        v.elapsedSeconds = s.data.timeSeconds;
    }
    crank_.advance(v.rpm, elapsed);

    uint8_t buffer[20];
    notifySubscribers(indoorBikeData_, buffer, ftms::encodeIndoorBikeData(v, buffer));
    notifySubscribers(powerMeasurement_, buffer, ftms::encodeCyclingPowerMeasurement(v.watt, crank_, buffer));
    if (v.pulse > 0)
        notifySubscribers(heartRate_, buffer, ftms::encodeHeartRateMeasurement(v.pulse, buffer));
}

void BleBridge::refreshPowerRange(const daum::BikeSnapshot& s) {
    if (s.targetLimit == publishedTargetLimit_)
        return;
    publishedTargetLimit_ = s.targetLimit;
    uint8_t buffer[6];
    powerRange_->setValue(buffer,
                          ftms::encodeSupportedPowerRange(daum::DaumCockpit::MinWatt, s.targetLimit,
                                                          daum::DaumCockpit::WattStep, buffer));
}

void BleBridge::refreshStatus(const daum::BikeSnapshot& s) {
    uint8_t value[9];
    value[0] = 1; // layout version
    value[1] = s.connected ? 0x01 : 0x00;
    value[2] = static_cast<uint8_t>(static_cast<int8_t>(s.cockpitType));
    value[3] = static_cast<uint8_t>(s.maxWatt & 0xFF);
    value[4] = static_cast<uint8_t>(s.maxWatt >> 8);
    value[5] = static_cast<uint8_t>(s.targetLimit & 0xFF);
    value[6] = static_cast<uint8_t>(s.targetLimit >> 8);
    value[7] = static_cast<uint8_t>(s.targetWatt & 0xFF);
    value[8] = static_cast<uint8_t>(s.targetWatt >> 8);
    if (statusPublished_ && std::memcmp(value, publishedStatus_, sizeof value) == 0)
        return;
    std::memcpy(publishedStatus_, value, sizeof value);
    bridgeStatus_->setValue(value, sizeof value);
    if (statusPublished_)
        bridgeStatus_->notify();
    statusPublished_ = true;
}
