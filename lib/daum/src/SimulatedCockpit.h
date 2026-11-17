#pragma once

#include <deque>
#include <vector>

#include "CockpitPort.h"

namespace daum {

/// A Daum cockpit in software behind CockpitPort: answers the commands the bridge uses like the real one (command
/// byte and address first) and can be told to stay silent, answer out of step or fall out of free training.
/// Port of ErgoWattControl's Tests/SimulatedCockpit.cs. With `live` set (simulator firmware), cadence, pulse and
/// counters move like on a ride and the power follows a new target with the cockpit's delay.
class SimulatedCockpit : public CockpitPort {
public:
    enum class Fault { Silent, WrongReply };

    uint8_t address = 7;
    /// False: like an old cockpit that doesn't echo the command (the bridge must not check then).
    bool echoes = true;
    uint8_t cockpitType = 40; // 8008
    /// False: like an old cockpit that doesn't know Get_Version (no answer).
    bool answersGetVersion = true;

    int program = 0;
    /// The program the cockpit falls back to again and again (a stuck cockpit); -1 = behaves.
    int stuckInProgram = -1;
    /// Power the cockpit brakes with, in 5 W steps.
    int wattSteps = 20;
    int rpm = 80;
    int pulse = 120;
    /// False: like a cockpit that is off or unplugged - no answer at all.
    bool poweredOn = true;

    /// Every request as written, in order (not recorded in live mode).
    std::vector<std::vector<uint8_t>> requests;

    /// Lets a missing answer take its timeout on `clock` (like a real port) instead of returning at once.
    void useClock(Clock& clock) { clock_ = &clock; }
    /// Enables the ride-like behaviour; `clock` drives it.
    void enableLive(Clock& clock);

    int count(uint8_t command) const;
    /// The next requests (one fault each) go wrong in this way.
    void inject(Fault fault, int times = 1);
    /// Only requests of this command consume injected faults (-1 = all).
    int faultOnlyFor = -1;

    /// Simulates a reset (e.g. static discharge): the cockpit is back in its default program and power.
    void reset();

    void discardInput() override;
    void write(const uint8_t* data, size_t length) override;
    size_t read(uint8_t* buffer, size_t length, uint32_t timeoutMs) override;

private:
    std::vector<uint8_t> answer(const std::vector<uint8_t>& request);
    void advance();

    std::deque<uint8_t> pending_;
    std::deque<Fault> faults_;

    Clock* clock_ = nullptr;
    bool live_ = false;
    uint32_t lastAdvanceMs_ = 0;
    int targetSteps_ = 20;
    uint32_t targetSinceMs_ = 0;
    double seconds_ = 300;
    double distanceKm_ = 12.3;
    double energyKJ_ = 100;
};

} // namespace daum
