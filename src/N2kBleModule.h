// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleModule.h - plug-in points for optional features (e.g. sending / TX).
//
// The bridge and the client do RX themselves. Anything else is a module that
// is added before begin(): it gets the GATT service to add its own
// characteristics to, and announces itself with a feature bit in Info so the
// other side knows whether it can use it. A bridge without the TX module simply
// never sets kFeatureTx, and a client with a TX module then refuses to send.
#pragma once
#include <stdint.h>

class NimBLEService;
class NimBLERemoteService;
class N2kBleBridge;
class N2kBleClient;

class N2kBleBridgeModule {
 public:
  virtual ~N2kBleBridgeModule() = default;
  // Feature bits this module adds to Info (see n2kble::Feature).
  virtual uint8_t features() const = 0;
  // Called once from N2kBleBridge::begin(), before the service is started.
  virtual bool setup(NimBLEService *service, N2kBleBridge &bridge) = 0;
  // Called from N2kBleBridge::loop().
  virtual void loop() {}
  // Called from the BLE host task: keep these short, no blocking.
  virtual void onClientConnected(uint16_t connHandle) { (void)connHandle; }
  virtual void onClientDisconnected(uint16_t connHandle) { (void)connHandle; }
};

class N2kBleClientModule {
 public:
  virtual ~N2kBleClientModule() = default;
  // Feature bits the bridge must announce for this module to be used.
  virtual uint8_t requiredFeatures() const = 0;
  // Called from the client's connection task after the RX stream is set up.
  // Return false if the module could not attach (it is then inactive until the
  // next connection; RX keeps working).
  virtual bool onConnected(NimBLERemoteService *service, N2kBleClient &client) = 0;
  virtual void onDisconnected() {}
  // Called from N2kBleClient::loop().
  virtual void loop() {}
};
