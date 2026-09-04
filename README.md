# VCanSim — Virtual SocketCAN ECU & Telemetry Simulator

VCanSim is a small, production-oriented Embedded Linux / C++20 reference
project. It simulates an engine control unit (ECU) that broadcasts realistic
vehicle telemetry over a Linux SocketCAN interface and a receiver that decodes,
validates, and renders the data on a live terminal dashboard.

The project is deliberately dependency-light: only Linux SocketCAN, the C++20
standard library, CMake, and GoogleTest (fetched automatically when not
installed) are required. No heap allocation occurs in the cyclic CAN
send/receive paths.

---

## Features

- `engine_ecu` — deterministic engine and vehicle simulation
  - 10 ms engine telemetry (`0x100`) and 100 ms diagnostics (`0x200`) cycles
  - Accelerating RPM, automatic gear shifting, vehicle speed, coolant warm-up,
    battery voltage ripple, and a cooling-failure overheat scenario
  - `std::chrono::steady_clock` absolute deadlines to avoid long-term drift
  - Graceful shutdown on `SIGINT` / `SIGTERM`
- `dashboard_rx` — live ANSI terminal dashboard
  - Kernel-level `CAN_RAW_FILTER` filters for exactly `0x100` and `0x200`
  - Verifies CRC-8 and the rolling 4-bit alive counter
  - Reports dropped diagnostic frames and CRC errors
  - Detects stale links / missing ECUs and shows active fault alerts
  - Non-blocking, timeout-based receive with no allocation in the loop
- `test_can_protocol` — GoogleTest suite covering serialization, scaling,
  boundaries, endianness, saturation, CRC, counter wrap, and corruption
  detection. Runs without `vcan0` or root privileges.
- Header-only protocol with portable big-endian packing using `std::bit_cast`,
  `std::span`, and fixed-width integer types.

---

## Project layout

```text
socketcan-sim/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── can_protocol.hpp          # 0x100 / 0x200 codec, CRC-8, scaling
│   └── socketcan_driver.hpp      # RAII raw SocketCAN wrapper
├── src/
│   ├── dashboard_rx.cpp          # filtered receiver + terminal dashboard
│   ├── engine_ecu.cpp            # engine physics + cyclic broadcaster
│   └── socketcan_driver.cpp      # sockets, filters, timeouts, signals
├── scripts/
│   └── setup_vcan.sh             # create and bring up vcan0
└── tests/
    └── test_can_protocol.cpp     # GoogleTest protocol suite
```

---

## Prerequisites

- Linux with the `vcan` kernel module (standard on Ubuntu/Debian)
- A C++20 compiler (tested with GCC 13)
- CMake ≥ 3.20
- `iproute2` (the `ip` command)
- Optional: `can-utils` for `candump` / `cansend`
- GoogleTest is optional; CMake fetches v1.15.2 automatically if no system
  package is found

On Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential cmake iproute2 can-utils
# Optional system GoogleTest (otherwise CMake fetches it):
sudo apt install libgtest-dev
```

---

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The configured compiler flags include:

```text
-std=c++20 -Wall -Wextra -Wpedantic -O2
```

Executables are written to:

```text
build/engine_ecu
build/dashboard_rx
build/test_can_protocol
```

---

## Set up the virtual CAN interface

```bash
sudo ./scripts/setup_vcan.sh vcan0
```

The script loads the `vcan` module if available, creates `vcan0` when needed,
brings it up, and sets a practical TX queue length. Verify it with:

```bash
ip -details link show vcan0
```

If you do not want to use the script:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan   # only if it does not already exist
sudo ip link set up vcan0
```

---

## Run

Open two terminals. In the first, start the ECU:

```bash
./build/engine_ecu --iface vcan0
```

In the second, start the dashboard:

```bash
./build/dashboard_rx --iface vcan0
```

Press `Ctrl+C` in either program for a clean shutdown.

### Overheat / fault demonstration

The default simulation stays healthy (coolant stabilizes below ~100 °C). To
demonstrate the fault path, inject a cooling-system failure after 30 seconds:

```bash
./build/engine_ecu --iface vcan0 --fault-after 30
```

The coolant then continues climbing past 110 °C, which sets the overheat
warning and the check-engine MIL on `0x200`. The dashboard shows red alerts.

### Inspect raw frames

With `can-utils` installed:

```bash
candump vcan0
candump -L vcan0        # log format suitable for replay
```

Example output:

```text
vcan0  100   [8]  2E E0 82 50 27 42 00 00
vcan0  200   [8]  00 35 DA 00 00 00 04 F7
```

### Command-line options

```text
engine_ecu --iface <name> --fault-after <seconds>
dashboard_rx --iface <name>
```

Both programs support `-h` / `--help`.

---

## CAN protocol

All multi-byte values use big-endian (network byte order). Classic CAN payloads
are 8 bytes. Scaling values are chosen to match typical automotive signal
conventions.

### `0x100` — Engine telemetry (every 10 ms)

| Bytes | Signal | Type | Scale | Range |
|------:|--------|------|-------|-------|
| 0–1 | Engine speed | `uint16` BE | 0.25 rpm/bit | 0–16383.75 rpm |
| 2 | Coolant temperature | `uint8` | 1 °C/bit, offset −40 °C | −40–215 °C |
| 3 | Throttle position | `uint8` | 0.5 %/bit | 0–127.5 % |
| 4–5 | Vehicle speed | `uint16` BE | 0.01 km/h/bit | 0–655.35 km/h |
| 6–7 | Reserved | — | — | transmitted as `0x00` |

Physical to raw encoding:

```text
rpm_raw      = round(rpm / 0.25)
coolant_raw  = round(coolant_c + 40)
throttle_raw = round(throttle_pct / 0.5)
speed_raw    = round(speed_kph / 0.01)
```

Out-of-range physical values are saturated to the representable range. The pack
and unpack helpers live in `include/can_protocol.hpp`.

### `0x200` — Diagnostics and faults (every 100 ms)

| Bytes | Signal | Description |
|------:|--------|-------------|
| 0 | Flags | bit 0 = check-engine MIL, bit 1 = overheat warning, bits 2–7 reserved |
| 1–2 | Battery voltage | `uint16` BE, 1 mV/bit |
| 3–5 | Reserved | `0x00` |
| 6 | Alive counter | 4-bit rolling counter in the low nibble; high nibble reserved |
| 7 | CRC-8 | CRC-8/SAE-J1850 over bytes 0–6 |

The 4-bit counter and the CRC-8 occupy the two final payload bytes: the rolling
counter in byte 6, immediately followed by the full 8-bit checksum in byte 7.
This keeps the fields adjacent while preserving a complete CRC-8.

CRC parameters:

```text
polynomial : 0x1D (x^8 + x^4 + x^3 + x^2 + 1)
initial    : 0xFF
xor-out    : 0xFF
reflected  : no
check      : crc8("123456789") = 0x4B
```

The receiver treats a CRC mismatch as a corrupted frame and does not update its
decoded diagnostic state. Valid frames are checked against the expected counter
value `(last + 1) mod 16`; a mismatch is accumulated as a dropped-frame count.
Counter wraparound `15 -> 0` is handled correctly.

---

## Testing

```bash
ctest --test-dir build --output-on-failure
```

or run the binary directly:

```bash
./build/test_can_protocol
```

The suite does not open a CAN socket, so it works in CI containers and on
machines without `vcan` or elevated privileges.

Coverage includes:

- big-endian serialization for `uint16` and payload byte order
- nominal, minimum, maximum, and saturated engine values
- quantisation and round-trip precision
- reserved bytes
- CRC-8 standard check vector
- diagnostic flag independence and layout
- alive counter wraparound and nibble masking
- rejection of every corrupted data byte and a corrupted CRC byte

---

## Design notes

- **No real-time allocations.** `SocketCanDriver::send` and
  `SocketCanDriver::receive` use stack objects only. Frame encoding writes into
  caller-provided `std::span<std::uint8_t, 8>` buffers.
- **Portable byte order.** `to_be16` / `from_be16` use `std::bit_cast` and
  `std::endian` so the wire format is identical on little- and big-endian
  hosts. Bit-masking is used for flags, the counter nibble, and CRC arithmetic.
- **RAII sockets.** The file descriptor is closed by the destructor, and move
  construction/assignment transfer ownership without double-close.
- **Kernel filtering.** `dashboard_rx` installs exact-match `CAN_RAW_FILTER`
  entries, so unrelated bus traffic never enters user space.
- **Bounded I/O.** The socket is non-blocking; reads use `poll()` with a
  timeout, sends retry once on a busy TX queue, and every loop observes the
  async-signal-safe shutdown flag.
- **Deterministic timing.** The ECU uses absolute `steady_clock` deadlines. If
  a scheduling pause exceeds one period, it resynchronises rather than emitting
  a burst of late frames.
- **Graceful shutdown.** `SIGINT` and `SIGTERM` set a `volatile sig_atomic_t`;
  handlers install without `SA_RESTART` so `poll()` wakes promptly. The
  dashboard restores the terminal cursor on exit.

---

## Troubleshooting

**`socket(PF_CAN)` fails / `No such device`**

Create and bring up the interface first:

```bash
sudo ./scripts/setup_vcan.sh vcan0
ip link show vcan0
```

**`engine_ecu` starts but the dashboard says `NO SIGNAL`**

Make sure both programs use the same interface name and that the interface is
`UP`. The dashboard also shows the age of the last frames.

**No `candump` command**

```bash
sudo apt install can-utils
```

**CMake cannot fetch GoogleTest**

Install a system copy (`sudo apt install libgtest-dev`) or configure with
`-DBUILD_TESTING=OFF` if tests are not needed.

**Permission denied for `setup_vcan.sh`**

The script must run as root:

```bash
sudo ./scripts/setup_vcan.sh vcan0
```
