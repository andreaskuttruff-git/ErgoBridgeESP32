# ErgoBridge

ESP32 firmware that connects to the RS232 port of a **daum electronic ergo_bike** cockpit (developed on an
ergo_bike 8008 TRS) and offers the bike as a standard **Bluetooth LE smart trainer** (Fitness Machine
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
| M2 | Bluetooth LE: FTMS (ERG / target power), Cycling Power, Heart Rate, Device Information | done, not yet tried on a board |
| M3 | FTMS driver in ErgoWattControl | next |
| M4 | RS232 hardware (MAX3232) on the real cockpit | |
| M5 | Real rides, reconnect tests | |
| M6 | Simulation mode (grade → power) for Zwift & co. | |

## Bluetooth

The bridge advertises as **ErgoBridge** with these services:

| Service | Content |
|---|---|
| Fitness Machine 0x1826 | Indoor Bike Data (speed, cadence, distance, power, heart rate, time; 1 Hz), Supported Power Range (25 W … cockpit maximum, 5 W steps), Control Point: Request Control, Reset, **Set Target Power**, Start, Stop/Pause; Fitness Machine Status |
| Cycling Power 0x1818 | power and crank revolutions (for watches and apps that only read CPS) |
| Heart Rate 0x180D | the cockpit's pulse, when it has one |
| Device Information 0x180A | manufacturer, model, firmware version |
| ErgoBridge status `c7b6d000-4e2a-4f69-9a5e-2d0e8b3f1a01` | characteristic `c7b6d001-…`: version, cockpit connected, cockpit type, max. watt, target limit, target watt |

Behaviour worth knowing:
- Only the connection that sent *Request Control* may set the power; others can read.
- When the controlling app disconnects, its target is kept for 10 s (app restart), then lowered to 80 W if it was higher. *Reset* lowers it to 80 W at once.
- While the cockpit isn't pedalled, the reported power is 0 W.
- Simulation mode (grade) and resistance levels are not supported yet.

## Hardware

- ESP32 dev board (ESP32-WROOM-32 / `esp32dev`)
- MAX3232 RS232 ↔ TTL converter for 3.3 V with **both drivers usable** (T1 for transmit, T2 for DTR). Many cheap
  DB9 modules only bring out channel 1; then DTR has to be soldered to the chip (T2OUT = pin 7, T2IN = pin 10).
- the original Daum PC cable (it stays between bridge and cockpit, see below)
- 5 V USB power supply

### Wiring

![Wiring: USB power supply, ESP32, MAX3232, DB9 wired like a PC, Daum PC cable](docs/wiring.svg)

| ESP32-WROOM-32 | MAX3232 | DB9 (wired like a PC's COM port) | Purpose |
|---|---|---|---|
| 3V3 | VCC | – | supply: **3.3 V, not 5 V** – the ESP32 pins are not 5 V tolerant |
| GND | GND | (pin 5) | ground; the Daum cable apparently doesn't use pin 5 |
| GPIO17 (TX2) | T1IN → T1OUT | pin 3 (TxD) | ESP32 transmits to the cockpit |
| GPIO16 (RX2) | R1OUT ← R1IN | pin 2 (RxD) | cockpit transmits to the ESP32 |
| – | T2IN to GND → T2OUT | pin 4 (DTR) | **always on**, positive voltage – required, see below |

The Daum PC cable, as measured: pins 2 and 3 run straight through (not crossed), pin 4 (DTR) goes through
4.8 kΩ onto the pin-2 line, and pin 5 seems not to be connected. The cockpit's interface is probably
opto-isolated: its output only pulls the receive line (pin 2) low; the high level comes from DTR through the
4.8 kΩ in the cable. So, exactly like a PC (ErgoWattControl sets `DtrEnable`), the bridge has to hold DTR at a
positive RS232 level, or nothing arrives from the cockpit. With T2IN tied to GND, the MAX3232's second driver
puts about +5.5 V on DTR; a bare 3.3 V TTL connection without the MAX3232 doesn't work.

- MAX3232 modules label their TTL pins inconsistently (TXD/RXD from either side); swapped RX/TX does no harm,
  there is just no answer.
- The cockpit has a male DB9 connector, the Daum cable two female ends. The bridge's DB9 must be male, like the
  PC's COM port; with a female module connector use a male/male gender changer (straight 1:1, **not** null-modem).
- Testing on a PC without the cockpit: leave out the MAX3232 and connect GPIO16/17 and GND straight to a 3.3 V
  USB-TTL adapter (TX to RX, RX to TX).

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
