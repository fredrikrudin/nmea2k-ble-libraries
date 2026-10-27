# Changelog

## 0.1.0 - 2026-10-27

First version: receive only.

- `N2kBleBridge`: streams every NMEA 2000 message from the bus to up to 4 BLE clients, with a PGN filter and a send queue per client, fragmentation of large messages, and drop counters.
- `N2kBleClient`: finds and connects to a bridge in a background task, reconnects, and hands messages to handlers as `tN2kMsg` (`onPGN`, `onPGN` per source, `onAny`). Can share the sketch's BLE scanner (`offerAdvertisement`). Sends its PGN filter automatically.
- `N2kBleModule.h`: plug-in points for later modules (TX). TX characteristic UUID and feature bit reserved.
- Wire protocol v1 (PROTOCOL.md), host unit tests, four examples.
