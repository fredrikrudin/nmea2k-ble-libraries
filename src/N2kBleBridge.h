// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleBridge.h - runs on an ESP32 connected to the NMEA 2000 bus and streams
// every received message to up to N BLE clients, each with its own PGN filter.
//
//   NMEA2000_esp32_twai n2k(TX_PIN, RX_PIN);
//   N2kBleBridge bridge;
//   void setup() {
//     n2k.SetMode(tNMEA2000::N2km_ListenOnly);
//     bridge.attachTo(n2k);            // receive every message from the library
//     n2k.Open();
//     bridge.begin();
//   }
//   void loop() { n2k.ParseMessages(); bridge.loop(); }
#pragma once
#include <NMEA2000.h>
#include <NimBLEDevice.h>

#include "N2kBleModule.h"
#include "N2kBleProtocol.h"
#include "N2kBleQueue.h"

#include <atomic>

struct N2kBleBridgeConfig {
  const char *deviceName  = "N2K-BLE";  // advertised name (scan response)
  uint8_t  maxClients     = 2;          // <= kMaxClients and <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS
  size_t   queueBytes     = 4096;       // per client; about 1 s of a busy bus
  uint16_t mtu            = 247;        // preferred ATT MTU
  uint8_t  maxPacketsPerLoop = 8;       // per client and loop() call
  int8_t   txPowerDbm     = 9;
};

class N2kBleBridge {
 public:
  static constexpr uint8_t kMaxClients = 4;
  static constexpr uint8_t kMaxModules = 4;

  using Config = N2kBleBridgeConfig;

  struct Stats {
    uint32_t busMessages;   // seen on the N2K bus
    uint32_t sent;          // records handed to BLE (all clients)
    uint32_t dropped;       // records dropped because a client queue was full
    uint32_t notifyBusy;    // notify() refused (BLE buffers full), retried later
    uint8_t  clients;
  };

  // Optional modules (e.g. a future N2kBleBridgeTx). Add before begin().
  bool addModule(N2kBleBridgeModule &m);

  // Starts the BLE server. Initialises NimBLE if nobody else has.
  bool begin(const Config &cfg = Config());

  // Feed it messages: either attachTo() once, or call handleN2kMsg() from your
  // own handler. Must be called from the same task as loop().
  void attachTo(tNMEA2000 &n2k);
  void handleN2kMsg(const tN2kMsg &msg);

  // Sends queued messages. Call often (every loop()).
  void loop();

  Stats stats() const;
  uint8_t features() const { return features_; }
  NimBLEServer *server() const { return server_; }

  // ---- internal, used by the NimBLE callback adaptors (BLE host task)
  void hostConnected(uint16_t conn, uint16_t mtu);
  void hostDisconnected(uint16_t conn);
  void hostMtu(uint16_t conn, uint16_t mtu);
  void hostSubscribe(uint16_t conn, bool on);
  void hostFilter(uint16_t conn, const uint8_t *data, size_t len);

 private:
  // Written by the host task under lock_, read by loop() under lock_.
  struct Shared {
    bool     active = false;
    uint16_t conn = 0;
    uint16_t mtu = 23;
    bool     subscribed = false;
    uint32_t epoch = 0;  // bumped on every new connection in this slot
    bool     filterDirty = false;
    int      filterCount = 0;  // 0 = every PGN
    uint32_t filter[n2kble::kMaxFilterPgns];
  };
  // Owned by the loop task.
  struct Session {
    uint32_t epoch = 0;
    bool     live = false;  // connected and subscribed (loop's view)
    uint16_t conn = 0;
    uint16_t mtu = 23;
    int      filterCount = 0;
    uint32_t filter[n2kble::kMaxFilterPgns];
    n2kble::RecordQueue queue;
    n2kble::Packetizer  pz;
    bool wants(uint32_t pgn) const;
  };

  class MsgHandler : public tNMEA2000::tMsgHandler {
   public:
    explicit MsgHandler(N2kBleBridge &b) : tNMEA2000::tMsgHandler(0, nullptr), b_(b) {}
    void HandleMsg(const tN2kMsg &m) override { b_.handleN2kMsg(m); }
   private:
    N2kBleBridge &b_;
  };

  void syncSessions();
  void updateInfo();
  int  slotOf(uint16_t conn) const;  // host side, under lock

  Config cfg_;
  NimBLEServer *server_ = nullptr;
  NimBLECharacteristic *rxChr_ = nullptr, *filterChr_ = nullptr, *infoChr_ = nullptr;
  tNMEA2000 *n2k_ = nullptr;
  MsgHandler handler_{*this};
  N2kBleBridgeModule *modules_[kMaxModules] = {};
  uint8_t moduleCount_ = 0;
  uint8_t features_ = n2kble::kFeatureRx;

  n2kble::Lock lock_;
  Shared  shared_[kMaxClients];
  Session sessions_[kMaxClients];
  std::atomic<uint8_t> connected_{0};

  uint32_t busMessages_ = 0, sent_ = 0, dropped_ = 0, notifyBusy_ = 0;
  uint32_t lastInfoMs_ = 0;
  uint8_t  pkt_[517];
  uint8_t  rec_[n2kble::kMaxRecord];
  bool     started_ = false;
};
