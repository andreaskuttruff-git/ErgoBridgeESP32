#include "SimulatedCockpit.h"

#include <algorithm>
#include <cmath>

namespace daum {

namespace {
/// The real cockpit needs about this long to settle on a new power.
constexpr uint32_t ResponseTimeMs = 2000;
} // namespace

void SimulatedCockpit::enableLive(Clock& clock) {
    clock_ = &clock;
    live_ = true;
    lastAdvanceMs_ = clock.nowMs();
    targetSteps_ = wattSteps;
    targetSinceMs_ = lastAdvanceMs_;
    rpm = 88;
}

int SimulatedCockpit::count(uint8_t command) const {
    return static_cast<int>(std::count_if(requests.begin(), requests.end(),
                                          [&](const std::vector<uint8_t>& r) { return !r.empty() && r[0] == command; }));
}

void SimulatedCockpit::inject(Fault fault, int times) {
    for (int i = 0; i < times; i++)
        faults_.push_back(fault);
}

void SimulatedCockpit::reset() {
    program = 1;
    wattSteps = 5;
    targetSteps_ = 5;
}

void SimulatedCockpit::discardInput() {
    pending_.clear();
}

void SimulatedCockpit::write(const uint8_t* data, size_t length) {
    std::vector<uint8_t> request(data, data + length);
    if (!live_)
        requests.push_back(request); // tests only: the simulator firmware runs for hours and would run out of heap
    if (!poweredOn)
        return;
    advance();
    std::vector<uint8_t> reply = answer(request);
    if (reply.empty())
        return;
    if ((faultOnlyFor < 0 || faultOnlyFor == request[0]) && !faults_.empty()) {
        const Fault fault = faults_.front();
        faults_.pop_front();
        if (fault == Fault::Silent)
            return;
        reply[0] = static_cast<uint8_t>(reply[0] ^ 0x5A); // a stray byte where the command echo should be
    }
    pending_.insert(pending_.end(), reply.begin(), reply.end());
}

size_t SimulatedCockpit::read(uint8_t* buffer, size_t length, uint32_t timeoutMs) {
    if (pending_.empty()) {
        // Like a real timeout, so the bridge's timing (and a fake clock) see the time pass.
        if (clock_ != nullptr)
            clock_->sleepMs(timeoutMs);
        return 0;
    }
    size_t n = 0;
    while (n < length && !pending_.empty()) {
        buffer[n++] = pending_.front();
        pending_.pop_front();
    }
    return n;
}

void SimulatedCockpit::advance() {
    if (!live_)
        return;
    const uint32_t now = clock_->nowMs();
    const double dt = (now - lastAdvanceMs_) / 1000.0;
    lastAdvanceMs_ = now;

    // Power settles on the target after the cockpit's response time.
    if (wattSteps != targetSteps_ && now - targetSinceMs_ >= ResponseTimeMs)
        wattSteps = targetSteps_;

    // Cadence wobbles around 88 rpm, pulse drifts with the power.
    const double t = now / 1000.0;
    rpm = static_cast<int>(std::lround(88 + 4 * std::sin(t / 7.0) + 2 * std::sin(t / 1.3)));
    const int wantedPulse = 90 + wattSteps * 5 / 4;
    if (pulse < wantedPulse) pulse++;
    else if (pulse > wantedPulse) pulse--;

    if (rpm > 0 && dt > 0 && dt < 10) {
        seconds_ += dt;
        distanceKm_ += 30.0 / 3600.0 * dt;
        energyKJ_ += wattSteps * 5 * dt / 1000.0;
    }
}

std::vector<uint8_t> SimulatedCockpit::answer(const std::vector<uint8_t>& r) {
    const uint8_t first = echoes ? r[0] : 0x00;
    switch (r[0]) {
    case 0x11:
        return {first, address};
    case 0x10:
        return {first, address, 0x2A};
    case 0x73: {
        if (!answersGetVersion)
            return {};
        std::vector<uint8_t> version(11, 0);
        version[0] = first;
        version[1] = address;
        version[10] = cockpitType;
        return version;
    }
    case 0x23:
        program = stuckInProgram >= 0 ? stuckInProgram : r[2];
        return {first, address, r[2], 0};
    case 0x21:
    case 0x22:
        return {first, address, 1};
    case 0x51:
        if (live_) {
            if (r[2] != targetSteps_)
                targetSinceMs_ = clock_->nowMs();
            targetSteps_ = r[2];
        } else {
            wattSteps = r[2];
        }
        return {first, address, r[2]};
    case 0x40: {
        std::vector<uint8_t> d(19, 0);
        const int distance = static_cast<int>(distanceKm_ * 10);
        const int seconds = static_cast<int>(seconds_);
        const int energy = static_cast<int>(energyKJ_ * 10);
        d[0] = first;
        d[1] = address;
        d[2] = static_cast<uint8_t>(program);
        d[4] = 0x81; // pedalling, with the stray high bit the firmware sometimes sends
        d[5] = static_cast<uint8_t>(wattSteps);
        d[6] = static_cast<uint8_t>(rpm);
        d[7] = 30;
        d[8] = static_cast<uint8_t>(distance & 0xFF);
        d[9] = static_cast<uint8_t>(distance >> 8);
        d[10] = static_cast<uint8_t>(seconds & 0xFF);
        d[11] = static_cast<uint8_t>(seconds >> 8);
        d[12] = static_cast<uint8_t>(energy & 0xFF);
        d[13] = static_cast<uint8_t>(energy >> 8);
        d[14] = static_cast<uint8_t>(pulse);
        d[16] = 14;
        return d;
    }
    default:
        return {}; // unknown command: no answer, like a real cockpit
    }
}

} // namespace daum
