# ErgoBridge

ESP32 firmware that connects to the RS232 port of a **daum electronic ergo_bike** cockpit (developed on an
ergo_bike 8008 TRS) and will offer the bike as a standard **Bluetooth LE smart trainer** (Fitness Machine
Service, FTMS). Training apps, starting with [ErgoWattControl](https://github.com/andreaskuttruff-git/ErgoWattControl)
and later Zwift, Rouvy, Kinomap, MyWhoosh or TrainerRoad, can then read power, cadence and heart rate and set
the brake power.

The Daum protocol code is a port of ErgoWattControl's tested C# implementation (address query, cockpit type
→ 400/800 W limit, free training, Set_Watt in 5 W steps, Run_Data, reply echo check, quick retries,
reconnect and re-initialisation after a cockpit reset).

## Status

| Milestone | Content | State |
|---|---|---|
| M1 | Daum protocol, self-healing bike logic, simulated cockpit, unit tests, serial console | done |
| M2 | Bluetooth LE: FTMS (ERG / target power), Cycling Power, Heart Rate, Device Information | next |
| M3 | FTMS driver in ErgoWattControl | |
| M4 | RS232 hardware (MAX3232) on the real cockpit | |
| M5 | Real rides, reconnect tests | |
| M6 | Simulation mode (grade → power) for Zwift & co. | |

## Hardware

- ESP32 dev board (ESP32-WROOM-32 / `esp32dev`)
- MAX3232 RS232 ↔ TTL module (3.3 V): cockpit TX → MAX3232 → **GPIO16 (RX2)**, **GPIO17 (TX2)** → MAX3232 → cockpit RX, GND
- the cockpit's 9-pin PC socket is on its underside; check the cable's pinout before connecting
- 5 V USB power supply

## Build

[PlatformIO](https://platformio.org/) (VS Code extension or `pip install platformio`):

```
pio run -e esp32-sim -t upload   # board only, built-in simulated cockpit
pio run -e esp32 -t upload       # real cockpit on UART2
pio device monitor               # 115200 baud; "w 150" sets 150 W, "s" prints the status
pio test -e native               # unit tests on the PC (needs gcc/g++ on the PATH)
```

## License

MIT, see [LICENSE](LICENSE).
