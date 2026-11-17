#pragma once

#include <atomic>
#include <mutex>

#include "DaumCockpit.h"

namespace daum {

/// What the rest of the firmware (BLE, serial console) sees of the bike.
struct BikeSnapshot {
    bool connected = false;
    /// Valid while connected: the latest Run_Data.
    RunData data;
    /// Clock time of `data`.
    uint32_t dataAtMs = 0;
    /// Cockpit type from Get_Version; -1 = not known (not connected yet, or an old cockpit that can't tell).
    int cockpitType = -1;
    /// Highest power this cockpit accepts (400 W until the type is known).
    int maxWatt = 400;
    /// The power the bridge asks the cockpit for.
    int targetWatt = 100;
    /// Connections set up since start (1 after the first connect).
    uint32_t connects = 0;
};

/// The Daum cockpit as a self-healing device: connects, keeps retrying while the cockpit is off or unplugged,
/// polls live data and restores free training + target power after a cockpit reset.
/// Port of ErgoWattControl's DaumErgometer.cs, as a step function instead of a thread: the caller (a FreeRTOS task
/// on the ESP32, the test loop on the host) calls step() and waits the returned time before calling it again.
class DaumBike {
public:
    static constexpr uint32_t RetryDelayMs = 3000;
    static constexpr uint32_t PollDelayMs = 400;
    /// A missing or out-of-step answer is asked again this often before the connection counts as lost.
    static constexpr int QuickRetries = 2;
    /// Re-initialisations in a row after which the connection is rebuilt from scratch.
    static constexpr int MaxReinitsInRow = 3;
    /// Polls in a row with a power different from the target after which Set_Watt is sent again.
    static constexpr int MaxWattMismatches = 3;

    DaumBike(CockpitPort& port, Clock& clock, LogSink log = nullptr);

    /// Thread-safe. Rounded to the 5 W raster and limited to [25 W, max of the cockpit] - 800 W, the protocol's
    /// limit, until the cockpit type is known; capped again then. Sent with the next step.
    void setTargetWatt(int watt);
    int targetWatt() const { return target_.load(); }

    /// Thread-safe copy of the current state.
    BikeSnapshot snapshot() const;

    /// Does one unit of work (connect attempt or one poll) and returns how long to wait before the next call.
    uint32_t step();

private:
    bool connect();
    bool queryCockpitType();
    bool initialize();
    bool poll();
    void markDisconnected(const char* reason);
    /// Rounds, clamps to [25 W, limit] and stores the target. Caller holds mutex_.
    int storeTarget(int watt, int limit);

    template <typename Request>
    Result retry(const char* what, Request request);

    void logf(const char* format, ...);

    DaumCockpit cockpit_;
    Clock& clock_;
    LogSink log_;

    std::atomic<int> target_{100};
    /// Upper limit for targets: the protocol's 800 W until Get_Version told the real one.
    std::atomic<int> targetLimit_{800};
    std::atomic<bool> targetPending_{false};

    mutable std::mutex mutex_;
    BikeSnapshot snapshot_;

    // Only touched by step().
    bool connected_ = false;
    int maxWatt_ = 400;
    int reinitsInRow_ = 0;
    int wattMismatches_ = 0;
    bool everConnected_ = false;
    uint32_t lostAtMs_ = 0;
    bool waiting_ = false;
    uint32_t waitingSinceMs_ = 0;
    Result notedFailure_ = Result::Ok;
};

} // namespace daum
