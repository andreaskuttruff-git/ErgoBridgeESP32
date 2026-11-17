#include "DaumCockpit.h"

#include <cmath>

namespace daum {

const char* toString(Result result) {
    switch (result) {
    case Result::Ok: return "ok";
    case Result::Timeout: return "no answer";
    case Result::Mismatch: return "reply does not match request";
    }
    return "?";
}

DaumCockpit::DaumCockpit(CockpitPort& port, Clock& clock) : port_(port), clock_(clock) {}

void DaumCockpit::waitForMinGap() {
    if (!sentBefore_)
        return;
    const uint32_t elapsed = clock_.nowMs() - lastSendMs_;
    if (elapsed < MinGapMs)
        clock_.sleepMs(MinGapMs - elapsed);
}

Result DaumCockpit::transact(const uint8_t* request, size_t requestLength, uint8_t* reply, size_t replyLength) {
    waitForMinGap();
    port_.discardInput();
    port_.write(request, requestLength);
    lastSendMs_ = clock_.nowMs();
    sentBefore_ = true;

    size_t received = 0;
    while (received < replyLength) {
        const uint32_t elapsed = clock_.nowMs() - lastSendMs_;
        if (elapsed >= ReplyTimeoutMs)
            return Result::Timeout;
        const size_t n = port_.read(reply + received, replyLength - received, ReplyTimeoutMs - elapsed);
        if (n == 0)
            return Result::Timeout;
        received += n;
    }
    // A reply that doesn't start with our command (and address) is out of step; its values would be garbage.
    if (echoes_ && (reply[0] != request[0] || (requestLength > 1 && reply[1] != request[1]))) {
        clock_.sleepMs(MinGapMs); // let the rest of the stray telegram arrive, the next request discards it
        return Result::Mismatch;
    }
    return Result::Ok;
}

Result DaumCockpit::queryAddress() {
    const uint8_t request[] = {0x11};
    uint8_t reply[2];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok) {
        echoes_ = reply[0] == 0x11;
        address_ = reply[1];
    }
    return r;
}

Result DaumCockpit::checkCockpit(uint8_t& version) {
    const uint8_t request[] = {0x10, address_};
    uint8_t reply[3];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        version = reply[2];
    return r;
}

Result DaumCockpit::getCockpitType(uint8_t& type) {
    const uint8_t request[] = {0x73, address_};
    uint8_t reply[11];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        type = reply[10];
    return r;
}

Result DaumCockpit::selectFreeTraining() {
    const uint8_t request[] = {0x23, address_, 0x00};
    uint8_t reply[4];
    return transact(request, sizeof request, reply, sizeof reply);
}

Result DaumCockpit::startProgram(bool& started) {
    const uint8_t request[] = {0x21, address_};
    uint8_t reply[3];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        started = reply[2] == 0x01;
    return r;
}

Result DaumCockpit::stopProgram(bool& stopped) {
    const uint8_t request[] = {0x22, address_};
    uint8_t reply[3];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        stopped = reply[2] == 0x01;
    return r;
}

Result DaumCockpit::setWatt(int watt, int& accepted) {
    // Protocol range; the bike logic enforces the limit of the actual cockpit type.
    if (watt < MinWatt) watt = MinWatt;
    if (watt > 800) watt = 800;
    const uint8_t steps = static_cast<uint8_t>(std::lround(watt / static_cast<double>(WattStep)));
    const uint8_t request[] = {0x51, address_, steps};
    uint8_t reply[3];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        accepted = reply[2] * WattStep;
    return r;
}

Result DaumCockpit::readRunData(RunData& data) {
    const uint8_t request[] = {0x40, address_};
    uint8_t reply[19];
    const Result r = transact(request, sizeof request, reply, sizeof reply);
    if (r == Result::Ok)
        data = parseRunData(reply);
    return r;
}

RunData DaumCockpit::parseRunData(const uint8_t* r) {
    RunData d;
    d.program = r[2];
    d.person = r[3];
    // Firmware quirk per spec: the pedaling flag can arrive with a stray high bit set, so only look at bit 0.
    d.pedaling = (r[4] & 0x01) != 0;
    d.watt = static_cast<uint16_t>(r[5] * WattStep);
    d.rpm = r[6];
    d.speedKmh = r[7];
    d.distanceKm = static_cast<float>(r[8] | (r[9] << 8)) / 10.0f;
    d.timeSeconds = static_cast<uint16_t>(r[10] | (r[11] << 8));
    d.energyKJ = static_cast<float>(r[12] | (r[13] << 8)) / 10.0f;
    d.pulse = r[14];
    d.pulseState = r[15];
    d.gear = r[16];
    return d;
}

int DaumCockpit::maxWattForType(int type) {
    return type == Type8008 || type == Type8080 ? 800 : 400;
}

const char* DaumCockpit::describeType(int type) {
    switch (type) {
    case TypeCardio: return "Cardio";
    case TypeFitness: return "Fitness";
    case TypeVitaDeLuxe: return "Vita De Luxe";
    case Type8008: return "8008";
    case Type8080: return "8080";
    case TypeTherapie: return "Therapie";
    default: return "unknown";
    }
}

} // namespace daum
