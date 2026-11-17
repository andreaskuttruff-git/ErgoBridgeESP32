// ErgoBridge firmware entry point.
// Milestone 1: the cockpit task (connect, poll, set power) and a serial console; Bluetooth FTMS follows in M2.

#include <Arduino.h>

#include "DaumBike.h"
#include "SimulatedCockpit.h"

namespace {

constexpr uint32_t ConsoleBaud = 115200;
constexpr int CockpitRxPin = 16;
constexpr int CockpitTxPin = 17;
constexpr uint32_t CockpitBaud = 9600;
constexpr uint32_t StatusEveryMs = 2000;

class ArduinoClock : public daum::Clock {
public:
    uint32_t nowMs() override { return millis(); }
    void sleepMs(uint32_t ms) override { delay(ms); }
};

/// The cockpit on a hardware UART (through a MAX3232 level shifter), 9600 8N1.
class UartCockpitPort : public daum::CockpitPort {
public:
    explicit UartCockpitPort(HardwareSerial& serial) : serial_(serial) {}

    void begin() { serial_.begin(CockpitBaud, SERIAL_8N1, CockpitRxPin, CockpitTxPin); }

    void discardInput() override {
        while (serial_.available() > 0)
            serial_.read();
    }

    void write(const uint8_t* data, size_t length) override {
        serial_.write(data, length);
        serial_.flush();
    }

    size_t read(uint8_t* buffer, size_t length, uint32_t timeoutMs) override {
        const uint32_t start = millis();
        while (serial_.available() == 0) {
            if (millis() - start >= timeoutMs)
                return 0;
            delay(1);
        }
        size_t n = 0;
        while (n < length && serial_.available() > 0)
            buffer[n++] = static_cast<uint8_t>(serial_.read());
        return n;
    }

private:
    HardwareSerial& serial_;
};

ArduinoClock arduinoClock;
#ifdef ERGOBRIDGE_SIMULATED_COCKPIT
daum::SimulatedCockpit cockpitPort;
#else
UartCockpitPort cockpitPort(Serial2);
#endif

void logLine(const char* line) {
    Serial.printf("[%7.1f] %s\n", millis() / 1000.0, line);
}

daum::DaumBike bike(cockpitPort, arduinoClock, logLine);

void cockpitTask(void*) {
    for (;;) {
        const uint32_t wait = bike.step();
        vTaskDelay(pdMS_TO_TICKS(wait > 0 ? wait : 1));
    }
}

void printStatus() {
    const daum::BikeSnapshot s = bike.snapshot();
    if (!s.connected) {
        Serial.printf("cockpit: not connected, target %d W\n", s.targetWatt);
        return;
    }
    Serial.printf("cockpit: %3u W (target %3d, max %d) %3u rpm %3u bpm  %5.1f km  %u s  program %u\n", s.data.watt,
                  s.targetWatt, s.maxWatt, s.data.rpm, s.data.pulse, s.data.distanceKm, s.data.timeSeconds,
                  s.data.program);
}

/// Console commands (115200 baud): "w 150" sets 150 W, "s" prints the status, "h" lists the commands.
void handleConsole() {
    static char line[32];
    static size_t length = 0;
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c != '\n' && c != '\r') {
            if (length < sizeof line - 1)
                line[length++] = c;
            continue;
        }
        line[length] = '\0';
        length = 0;
        if (line[0] == 'w') {
            const int watt = atoi(line + 1);
            bike.setTargetWatt(watt);
            Serial.printf("target %d W\n", bike.targetWatt());
        } else if (line[0] == 's') {
            printStatus();
        } else if (line[0] != '\0') {
            Serial.println("commands: w <watt> = set power, s = status");
        }
    }
}

} // namespace

void setup() {
    Serial.begin(ConsoleBaud);
    delay(200);
#ifdef ERGOBRIDGE_SIMULATED_COCKPIT
    cockpitPort.enableLive(arduinoClock);
    Serial.println("ErgoBridge - simulated cockpit");
#else
    cockpitPort.begin();
    Serial.println("ErgoBridge - cockpit on UART2 (GPIO16 RX, GPIO17 TX)");
#endif
    // Core 1: the cockpit; core 0 is left to the Bluetooth stack (M2).
    xTaskCreatePinnedToCore(cockpitTask, "cockpit", 6144, nullptr, 2, nullptr, 1);
}

void loop() {
    static uint32_t lastStatus = 0;
    handleConsole();
    if (millis() - lastStatus >= StatusEveryMs) {
        lastStatus = millis();
        printStatus();
    }
    delay(20);
}
