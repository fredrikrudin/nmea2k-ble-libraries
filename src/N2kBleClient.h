// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleClient.h - connects to an N2kBleBridge and hands every received NMEA
// 2000 message to your functions as a normal tN2kMsg.
//
//   N2kBleClient n2k;
//   void setup() {
//     n2k.onPGN(128267, [](const tN2kMsg &m) {      // water depth
//       unsigned char sid; double depth, offset, range;
//       if (ParseN2kWaterDepth(m, sid, depth, offset, range)) showDepth(depth);
//     });
//     n2k.begin();
//   }
//   void loop() { n2k.loop(); }   // handlers run here, in your task
//
// Connecting, discovery, reconnecting and the PGN filter are handled by a
// background FreeRTOS task, so loop() never blocks (LVGL keeps running).
#pragma once
#include <N2kMsg.h>
#include <NimBLEDevice.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include "N2kBleDispatcher.h"
#include "N2kBleModule.h"
#include "N2kBleProtocol.h"
#include "N2kBleQueue.h"

struct N2kBleClientConfig {
  // Which bridge. Address wins over name; with neither, the first bridge found is used.
  const char *bridgeAddress = nullptr;  // "aa:bb:cc:dd:ee:ff" - connects without scanning
  const char *bridgeName    = nullptr;  // needs an active scan (name is in the scan response)

  // true : the client runs its own BLE scan until it has found the bridge.
  // false: your sketch already scans (RuuviTag, Victron, ...) and calls
  //        offerAdvertisement() from its NimBLEScanCallbacks::onResult().
  bool ownScan = true;

  // false: handlers run in loop() (safe for LVGL and most sketches).
  // true : handlers run in the client's own task, right after reception;
  //        your handlers must then be thread safe. loop() is optional.
  bool dispatchInTask = false;

  size_t   queueBytes       = 8192;  // received packets waiting for loop()
  uint16_t mtu              = 247;
  uint16_t connIntervalMs   = 30;    // longer = more radio time left for WiFi
  uint32_t connectTimeoutMs = 5000;
  uint32_t retryDelayMs     = 3000;
  uint8_t  failuresBeforeRescan = 3; // found bridges only; a fixed address is retried forever
  int      taskCore     = 0;         // tskNO_AFFINITY for any core
  uint32_t taskStack    = 4096;
  uint8_t  taskPriority = 2;
};

class N2kBleClient {
 public:
  using Handler = N2kBleDispatcher::Handler;
  static constexpr uint8_t kMaxModules = 4;

  using Config = N2kBleClientConfig;

  struct Stats {
    uint32_t packets;        // notifications received
    uint32_t messages;       // N2K messages delivered to the dispatcher
    uint32_t queueDropped;   // packets lost because loop() was too slow
    uint32_t bridgeDropped;  // messages the bridge dropped for us (BLE too slow, use a filter)
    uint32_t errors;         // malformed packets
    uint32_t connects;       // successful connections since begin()
  };

  // ---- what to do with messages (before or after begin())
  void onPGN(uint32_t pgn, Handler h);
  void onPGN(uint32_t pgn, uint8_t source, Handler h);
  void onAny(Handler h);  // every message; also makes the bridge send every PGN
  void onConnection(std::function<void(bool connected)> cb);

  // Optional modules (e.g. a future N2kBleClientTx). Add before begin().
  bool addModule(N2kBleClientModule &m);

  // Initialises NimBLE if nobody else has, and starts the connection task.
  bool begin(const Config &cfg = Config());
  // Delivers received messages to the handlers. Call every loop().
  void loop();

  // When cfg.ownScan is false: pass every advertisement here. Cheap, safe
  // from the NimBLE scan callback.
  void offerAdvertisement(const NimBLEAdvertisedDevice *dev);

  bool        connected() const { return state_ == State::Connected; }
  // Milliseconds since this PGN was last received, UINT32_MAX if never or no handler.
  uint32_t    ageMs(uint32_t pgn);
  // Address of the bridge last connected to: save it (NVS) and pass it as
  // bridgeAddress next time to skip scanning.
  std::string bridgeAddress();
  n2kble::Info bridgeInfo();
  Stats       stats() const;

  // For modules
  NimBLEClient *bleClient() const { return client_; }

 private:
  enum class State : uint8_t { Searching, Connecting, Connected };

  static void taskEntry(void *arg);
  void task();
  bool connectAndSetup();
  void teardown();
  void writeFilter();
  void drain();
  void onNotify(const uint8_t *data, size_t len);
  static void sink(const n2kble::Record &r, void *ctx);

  Config cfg_;
  N2kBleDispatcher dispatcher_;
  std::recursive_mutex dispMutex_;
  std::function<void(bool)> connCb_;
  N2kBleClientModule *modules_[kMaxModules] = {};
  uint8_t moduleCount_ = 0;
  bool    moduleActive_[kMaxModules] = {};

  n2kble::PacketQueue queue_;
  n2kble::Reassembler reasm_;   // consumer side only
  std::mutex lock_;             // guards target_, haveTarget_, info_, address_ (may allocate, so not a spinlock)
  NimBLEAddress target_;
  bool     haveTarget_ = false;
  bool     fixedTarget_ = false;
  std::string address_;
  n2kble::Info info_;

  NimBLEClient *client_ = nullptr;
  NimBLERemoteCharacteristic *filterChr_ = nullptr;
  void *taskHandle_ = nullptr;
  std::atomic<State> state_{State::Searching};
  std::atomic<uint32_t> connEpoch_{0};  // bumped on connect and on disconnect
  uint32_t consumerEpoch_ = 0;
  bool     reportedConnected_ = false;
  uint32_t filterRevisionSent_ = 0xffffffff;
  uint8_t  failures_ = 0;

  std::atomic<uint32_t> packets_{0}, messages_{0}, connects_{0};
  uint8_t  pkt_[520];
  bool     started_ = false;
};
