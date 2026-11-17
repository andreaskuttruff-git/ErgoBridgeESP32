#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>

#include "FtmsCodec.h"

namespace ftms {

/// The FTMS control point: who may control the trainer, what each request does, and what happens when the
/// controlling app goes away. Independent of the BLE stack; thread-safe (BLE callbacks and the main loop call it).
class TrainerControl {
public:
    /// Sends a target power to the bike and returns the value the bike took (rounded, limited).
    using ApplyTarget = std::function<int(int watt)>;
    /// The bike's current target power.
    using CurrentTarget = std::function<int()>;

    static constexpr uint16_t NoConnection = 0xFFFF;
    /// After the controlling app disconnects, its last target is kept this long (an app restart, a short drop-out).
    static constexpr uint32_t HoldAfterDisconnectMs = 10000;
    /// Then a higher target is lowered to this, so nobody pedals against a hard load the app no longer controls.
    /// Reset lowers it to this at once.
    static constexpr int FallbackWatt = 80;

    struct Reply {
        /// Control point response to indicate to the writer; length 0 = nothing (empty write).
        uint8_t response[3];
        size_t responseLength = 0;
        /// Fitness Machine Status to notify to all; length 0 = none.
        uint8_t status[3];
        size_t statusLength = 0;
    };

    TrainerControl(ApplyTarget applyTarget, CurrentTarget currentTarget);

    /// A write to the control point by connection `conn`.
    Reply handleControlPoint(uint16_t conn, const uint8_t* data, size_t length, uint32_t nowMs);

    /// A connection closed. Returns the length of a status to notify to the others (Control Permission Lost) or 0.
    size_t onDisconnect(uint16_t conn, uint32_t nowMs, uint8_t* status);

    /// Call regularly. Returns the length of a status to notify (Target Power Changed) when the fallback applied.
    size_t tick(uint32_t nowMs, uint8_t* status);

    /// The connection in control, or NoConnection.
    uint16_t controller() const;

private:
    Reply respond(uint8_t opCode, ResultCode result);

    ApplyTarget applyTarget_;
    CurrentTarget currentTarget_;

    mutable std::mutex mutex_;
    uint16_t controller_ = NoConnection;
    bool fallbackPending_ = false;
    uint32_t controllerLostAtMs_ = 0;
};

} // namespace ftms
