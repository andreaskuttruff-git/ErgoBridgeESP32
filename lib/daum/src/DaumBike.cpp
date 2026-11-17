#include "DaumBike.h"

#include <cstdarg>
#include <cstdio>

namespace daum {

DaumBike::DaumBike(CockpitPort& port, Clock& clock, LogSink log)
    : cockpit_(port, clock), clock_(clock), log_(log) {}

void DaumBike::logf(const char* format, ...) {
    if (log_ == nullptr)
        return;
    char line[160];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof line, format, args);
    va_end(args);
    log_(line);
}

int DaumBike::storeTarget(int watt, int limit) {
    // Round to the cockpit's 5 W raster so the polled power can match the target exactly.
    if (watt < 0) watt = 0;
    int rounded = (watt + DaumCockpit::WattStep / 2) / DaumCockpit::WattStep * DaumCockpit::WattStep;
    if (rounded < DaumCockpit::MinWatt) rounded = DaumCockpit::MinWatt;
    if (rounded > limit) rounded = limit;
    // Value first, then the flag: step() clears the flag before it reads the value.
    target_.store(rounded);
    snapshot_.targetWatt = rounded;
    targetPending_.store(true);
    return rounded;
}

void DaumBike::setTargetWatt(int watt) {
    std::lock_guard<std::mutex> lock(mutex_);
    storeTarget(watt, targetLimit_.load());
}

BikeSnapshot DaumBike::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

template <typename Request>
Result DaumBike::retry(const char* what, Request request) {
    Result r = Result::Ok;
    for (int attempt = 0; attempt <= QuickRetries; attempt++) {
        r = request();
        if (r == Result::Ok) {
            if (attempt > 0)
                logf("Cockpit: %s answered after %d retry/retries", what, attempt);
            return r;
        }
        if (r == Result::Mismatch)
            logf("Cockpit: %s: %s", what, toString(r));
    }
    return r;
}

uint32_t DaumBike::step() {
    if (!connected_) {
        if (!connect())
            return RetryDelayMs;
        return 0; // poll right away
    }
    if (!poll())
        return RetryDelayMs;
    return PollDelayMs;
}

bool DaumBike::connect() {
    const uint32_t now = clock_.nowMs();
    const Result r = cockpit_.queryAddress();
    if (r != Result::Ok) {
        // Not connected yet (cockpit asleep, off or unplugged): noted once per reason, not every 3 s.
        if (!waiting_) {
            waiting_ = true;
            waitingSinceMs_ = now;
        }
        if (r != notedFailure_) {
            notedFailure_ = r;
            logf("Cockpit not answering (%s) - retrying every %u s", toString(r),
                 static_cast<unsigned>(RetryDelayMs / 1000));
        }
        return false;
    }
    if (!queryCockpitType() || !initialize()) {
        logf("Cockpit answered but setting it up failed - retrying");
        return false;
    }

    connected_ = true;
    reinitsInRow_ = 0;
    wattMismatches_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.connected = true;
        snapshot_.connects++;
    }
    const BikeSnapshot s = snapshot();
    const uint32_t since = everConnected_ ? lostAtMs_ : waitingSinceMs_;
    const bool waited = everConnected_ || waiting_;
    logf("Cockpit %s (address %u, type %s, max. %d W)%s%u%s", everConnected_ ? "reconnected" : "connected",
         cockpit_.address(), s.cockpitType < 0 ? "unknown" : DaumCockpit::describeType(s.cockpitType), s.maxWatt,
         waited ? " after " : "", waited ? static_cast<unsigned>((clock_.nowMs() - since) / 1000) : 0u,
         waited ? " s" : "");
    everConnected_ = true;
    waiting_ = false;
    notedFailure_ = Result::Ok;
    return true;
}

bool DaumBike::queryCockpitType() {
    uint8_t type = 0;
    const Result r = cockpit_.getCockpitType(type);
    int cockpitType = -1;
    if (r == Result::Ok)
        cockpitType = type;
    else if (r != Result::Timeout)
        return false; // out of step: the whole connect is tried again
    // Timeout: older cockpits don't know Get_Version - stay with the safe 400 W.
    maxWatt_ = DaumCockpit::maxWattForType(cockpitType);
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.cockpitType = cockpitType;
    snapshot_.maxWatt = maxWatt_;
    targetLimit_.store(maxWatt_);
    if (target_.load() > maxWatt_)
        storeTarget(maxWatt_, maxWatt_);
    return true;
}

bool DaumBike::initialize() {
    if (retry("Get_Address", [&] { return cockpit_.queryAddress(); }) != Result::Ok)
        return false;
    if (retry("free training", [&] { return cockpit_.selectFreeTraining(); }) != Result::Ok)
        return false;
    bool started = false;
    if (retry("program start", [&] { return cockpit_.startProgram(started); }) != Result::Ok)
        return false;
    targetPending_.store(false);
    int accepted = 0;
    const int watt = target_.load();
    return retry("Set_Watt", [&] { return cockpit_.setWatt(watt, accepted); }) == Result::Ok;
}

void DaumBike::markDisconnected(const char* reason) {
    connected_ = false;
    lostAtMs_ = clock_.nowMs();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.connected = false;
    }
    logf("Cockpit connection lost (%s)", reason);
}

bool DaumBike::poll() {
    if (targetPending_.exchange(false)) {
        int accepted = 0;
        const int watt = target_.load(); // only the latest pending value matters
        const Result r = retry("Set_Watt", [&] { return cockpit_.setWatt(watt, accepted); });
        if (r != Result::Ok) {
            targetPending_.store(true); // send it again after reconnecting
            markDisconnected(toString(r));
            return false;
        }
    }

    RunData data;
    const Result r = retry("Run_Data", [&] { return cockpit_.readRunData(data); });
    if (r != Result::Ok) {
        markDisconnected(toString(r));
        return false;
    }

    // After a cockpit reset (e.g. static discharge) it may answer again but has left free training or fallen back
    // to its default power, so restore our state.
    if (data.program != 0) {
        if (++reinitsInRow_ > MaxReinitsInRow) {
            markDisconnected("cockpit stays in a program instead of free training");
            return false;
        }
        logf("Cockpit in program %u instead of free training - set up again (attempt %d)", data.program,
             reinitsInRow_);
        if (!initialize()) {
            markDisconnected("setting up again failed");
            return false;
        }
        return true;
    }
    reinitsInRow_ = 0;

    const int target = target_.load();
    wattMismatches_ = data.watt == target ? 0 : wattMismatches_ + 1;
    if (wattMismatches_ >= MaxWattMismatches) {
        int accepted = 0;
        if (retry("Set_Watt", [&] { return cockpit_.setWatt(target, accepted); }) != Result::Ok) {
            markDisconnected("Set_Watt failed");
            return false;
        }
        wattMismatches_ = 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.data = data;
    snapshot_.dataAtMs = clock_.nowMs();
    return true;
}

} // namespace daum
