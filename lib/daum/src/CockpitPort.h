#pragma once

#include <cstddef>
#include <cstdint>

namespace daum {

/// The byte channel to the cockpit: the ESP32 UART, or a simulated cockpit in tests and in the simulator build.
class CockpitPort {
public:
    virtual ~CockpitPort() = default;
    virtual void discardInput() = 0;
    virtual void write(const uint8_t* data, size_t length) = 0;
    /// Reads up to `length` bytes, waiting at most `timeoutMs` for the first one. Returns 0 on timeout.
    virtual size_t read(uint8_t* buffer, size_t length, uint32_t timeoutMs) = 0;
};

/// Time source, so the protocol logic runs unchanged on the ESP32 and in host unit tests with a fake clock.
class Clock {
public:
    virtual ~Clock() = default;
    virtual uint32_t nowMs() = 0;
    virtual void sleepMs(uint32_t ms) = 0;
};

/// Receives one line of diagnostic text (no trailing newline).
using LogSink = void (*)(const char* line);

} // namespace daum
