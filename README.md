# nmea2k-ble-libraries

NMEA 2000 over Bluetooth Low Energy for ESP32.

An ESP32 on the NMEA 2000 bus (the **bridge**) streams every message it receives over BLE. A display or any other ESP32 (the **client**) connects to it and maps the PGNs it cares about to its own functions. No drop cable to the display, and several displays can share one bridge.

```
 NMEA 2000 bus ──CAN──▶ ESP32 bridge ──BLE──▶ display 1  onPGN(128267, showDepth)
 (depth, engine,        N2kBleBridge   ├────▶ display 2  onPGN(127488, showRpm)
  tanks, GPS ...)                      └────▶ ... up to 4 clients, each with its own filter
```

- **Any PGN.** The bridge forwards every message, after the NMEA2000 library has put fast-packets back together. Nothing is decoded on the bridge, so new devices on the bus need no bridge update.
- **Normal `tN2kMsg` on the client.** Every `ParseN2k…()` function of the NMEA2000 library works unchanged. A handler written for a wired bus (`tNMEA2000::SetMsgHandler`) can be passed to `onAny()` as is.
- **Only what you use crosses the air.** The client tells the bridge which PGNs it has handlers for; the bridge filters per client.
- **Never blocks your loop.** Scanning, connecting and reconnecting run in a background task. Handlers run in `loop()` by default, so LVGL code can update widgets directly.
- **Modular.** Receiving is built in. Sending (TX) will be a separate module with its own feature bit (see [Roadmap](#roadmap)).

## Install

Arduino IDE: copy this folder to `Documents/Arduino/libraries/`, and install from the Library Manager:

| Library | Used by |
|---|---|
| NimBLE-Arduino **2.x** (h2zero) | bridge and client |
| NMEA2000 (Timo Lappalainen) | bridge and client |
| NMEA2000_esp32_twai (Sergei P.) | bridge only, the ESP32-S3 CAN driver. Plain ESP32: NMEA2000_esp32 also works |

PlatformIO: `lib_deps = https://github.com/fredrikrudin/nmea2k-ble-libraries` (dependencies are in `library.json`).

Tested to compile with arduino-esp32 3.3.12, NimBLE-Arduino 2.5.2 and NMEA2000 4.25.1 on ESP32-S3.

## Bridge

```cpp
#define ESP32_CAN_TX_PIN GPIO_NUM_6
#define ESP32_CAN_RX_PIN GPIO_NUM_0
#include <NMEA2000_esp32_twai.h>
#include <N2kBle.h>

NMEA2000_esp32_twai n2k(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN);
N2kBleBridge bridge;

void setup() {
  n2k.SetMode(tNMEA2000::N2km_ListenOnly);
  bridge.attachTo(n2k);   // every message the library receives goes to BLE
  n2k.Open();
  bridge.begin();         // advertises as "N2K-BLE"
}

void loop() {
  n2k.ParseMessages();
  bridge.loop();
}
```

The bridge only listens; it never writes to the bus. Fast-packet PGNs the NMEA2000 library doesn't know (proprietary ones) must be added with `n2k.ExtendFastPacketMessages()`, or they arrive as separate 8-byte frames. See `examples/Bridge`.

## Client

```cpp
#include <N2kBle.h>
#include <N2kMessages.h>

N2kBleClient n2k;

void setup() {
  n2k.onPGN(128267, [](const tN2kMsg &m) {          // water depth
    unsigned char sid; double depth, offset, range;
    if (ParseN2kWaterDepth(m, sid, depth, offset, range)) showDepth(depth);
  });
  n2k.onPGN(127505, [](const tN2kMsg &m) {          // every tank
    unsigned char inst; tN2kFluidType type; double level, cap;
    if (ParseN2kFluidLevel(m, inst, type, level, cap)) showTank(inst, type, level);
  });
  n2k.begin();   // finds the bridge, connects, reconnects - in the background
}

void loop() {
  n2k.loop();    // handlers run here
}
```

### Mapping PGNs to functions

| Call | Runs for |
|---|---|
| `onPGN(pgn, fn)` | every message with that PGN |
| `onPGN(pgn, source, fn)` | that PGN from one device (source address), e.g. one of two engine gateways |
| `onAny(fn)` | every message. Also makes the bridge send every PGN, so use it for sniffing or for a handler that already does its own `switch (msg.PGN)` |
| `onConnection(fn)` | `fn(true)` when connected to a bridge, `fn(false)` when it is lost |

Several handlers per PGN are allowed. Handlers can be added after `begin()`; the filter on the bridge is updated automatically.

`ageMs(pgn)` tells how long ago a PGN arrived (`UINT32_MAX` if never), for blanking stale values on a display. `msg.MsgTime` is when the message reached the client.

### Reusing a wired handler

If your firmware already has a handler for a wired bus, nothing in it changes:

```cpp
void onMsg(const tN2kMsg &msg);   // the one you pass to SetMsgHandler() today

n2k.onAny(onMsg);                 // BLE
// or, to keep the bridge filter tight:
for (unsigned long pgn : {127488UL, 127489UL, 127505UL, 128267UL}) n2k.onPGN(pgn, onMsg);
```

### When your sketch already scans BLE

NimBLE has a single scanner. If your sketch scans for RuuviTags, Victron devices and so on, let it keep the scanner and pass advertisements on:

```cpp
void onResult(const NimBLEAdvertisedDevice *dev) override {
  // ... your decoders ...
  n2k.offerAdvertisement(dev);
}

N2kBleClient::Config cfg;
cfg.ownScan = false;
n2k.begin(cfg);
```

A passive scan is enough: the bridge's service UUID is in the advertisement itself. Save `n2k.bridgeAddress()` (NVS) and pass it as `cfg.bridgeAddress` next boot, and the client connects directly without waiting for a scan. See `examples/ClientSharedScan`.

### Client configuration

| `Config` field | Default | |
|---|---|---|
| `bridgeAddress` | none | connect to this address, no scanning |
| `bridgeName` | none | only a bridge with this name (needs an active scan) |
| `ownScan` | `true` | `false` when the sketch scans and calls `offerAdvertisement()` |
| `dispatchInTask` | `false` | `true`: handlers run in the client task right away (must be thread safe) |
| `queueBytes` | 8192 | received data waiting for `loop()`; in PSRAM when available |
| `connIntervalMs` | 30 | longer leaves more radio time for WiFi |
| `retryDelayMs` | 3000 | between connection attempts |
| `taskCore`, `taskStack`, `taskPriority` | 0, 4096, 2 | raise `taskStack` if handlers run in the task |

### Statistics

`stats()` on both sides. On the client, `bridgeDropped` counts messages the bridge had to drop for this client because BLE could not keep up; if it grows, narrow the filter (avoid `onAny()`), or raise `queueBytes` on the bridge. `queueDropped` means `loop()` is called too rarely.

## How it works

- The bridge attaches a handler to the NMEA2000 library, so it receives complete messages: up to 223 bytes, fast-packets and ISO multi-packets already reassembled.
- Each message becomes a 7-byte header + data. As many as fit go into one BLE notification (at MTU 247, about 16 eight-byte messages per notification). A message larger than one notification is split into fragments.
- Each client has its own queue on the bridge. When a queue is full the **oldest** messages are dropped, because on a display the newest value matters. The drop count travels in the next packet header.
- The client's BLE callback only copies the notification into a queue. `loop()` decodes it into `tN2kMsg` and calls your handlers.

The byte format is in [PROTOCOL.md](PROTOCOL.md), so other clients (a phone app, a Python script) can talk to a bridge too.

## Memory

- Bridge: `queueBytes` × `maxClients` (default 4 kB × 2) plus NimBLE's connection buffers.
- Client: `queueBytes` (8 kB, PSRAM if present) plus one BLE connection in internal RAM.

## Roadmap

Modules plug in with `addModule()` before `begin()` (`src/N2kBleModule.h`) and announce themselves with a feature bit in the Info characteristic, so either side knows what the other supports.

- **TX module** (`N2kBleBridgeTx` / `N2kBleClientTx`): the client writes messages in the same record format to the TX characteristic, already reserved as `b4060005-…`. The bridge sends them as a proper N2K node (`N2km_ListenAndNode`, address claim). Planned safety: bonded clients only, and a PGN allow-list on the bridge so a phone can't send autopilot or switching commands unless you allow it.
- Optional per-PGN rate limit on the bridge for very busy buses.

## Tests

Host tests for the wire format, the queues, the dispatcher and the NMEA2000 parsers on received messages:

```sh
NMEA2000_SRC=/path/to/NMEA2000/src ./test/run_tests.sh
```

## License

Copyright (c) 2026 Fredrik Rudin.

Licensed under [Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)](https://creativecommons.org/licenses/by-nc/4.0/). See [LICENSE.md](LICENSE.md).

The libraries this depends on keep their own licenses (NimBLE-Arduino: Apache 2.0, NMEA2000 and NMEA2000_esp32_twai: MIT); they are not included here.

NMEA 2000® is a registered trademark of the National Marine Electronics Association. This project is not affiliated with or certified by NMEA.
