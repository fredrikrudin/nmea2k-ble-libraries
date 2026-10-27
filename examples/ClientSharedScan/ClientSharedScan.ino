// nmea2k-ble-libraries - ClientSharedScan example
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
//
// For sketches that already scan BLE themselves (RuuviTag, Victron, ...).
// NimBLE has one scanner, so the client doesn't start its own: your scan
// callback hands each advertisement to offerAdvertisement() as well.
//
// Once found, the bridge address is saved, and next boot connects straight to
// it without scanning (bridgeAddress). Your sketch can keep scanning on its own
// schedule; NimBLE pauses a running scan while the client connects.

#include <N2kBle.h>
#include <Preferences.h>

N2kBleClient n2k;
Preferences prefs;
static char savedAddr[18] = "";

class MyScan : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    // ... your RuuviTag / Victron decoding here ...
    n2k.offerAdvertisement(dev);  // one line for the bridge
  }
};
static MyScan myScan;

// A handler written for a wired bus works as-is: same signature as for
// tNMEA2000::SetMsgHandler().
void onN2kMsg(const tN2kMsg &m) {
  Serial.printf("PGN %lu from %u\n", (unsigned long)m.PGN, m.Source);
}

void setup() {
  Serial.begin(115200);

  // Your existing scanner
  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&myScan, true);
  scan->setActiveScan(false);  // the bridge's service UUID is in the advertisement itself
  scan->setMaxResults(0);

  prefs.begin("n2kble", false);
  prefs.getString("bridge", savedAddr, sizeof(savedAddr));

  n2k.onAny(onN2kMsg);
  n2k.onConnection([](bool up) {
    if (!up) return;
    std::string a = n2k.bridgeAddress();
    if (a != savedAddr) {
      prefs.putString("bridge", a.c_str());
      strncpy(savedAddr, a.c_str(), sizeof(savedAddr) - 1);
    }
  });

  N2kBleClient::Config cfg;
  cfg.ownScan = false;
  cfg.bridgeAddress = savedAddr[0] ? savedAddr : nullptr;
  cfg.connIntervalMs = 45;  // leave radio time for WiFi
  n2k.begin(cfg);
}

void loop() {
  n2k.loop();

  // your scan schedule, e.g. 3 s every 10 s
  static uint32_t next = 0;
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (!scan->isScanning() && (int32_t)(millis() - next) >= 0) {
    scan->start(3000, false, true);
    next = millis() + 10000;
  }
}
