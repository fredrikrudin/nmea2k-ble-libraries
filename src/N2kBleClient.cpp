// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleClient.cpp
#include "N2kBleClient.h"

#include <Arduino.h>

using namespace n2kble;

namespace {

const NimBLEUUID &serviceUuid() {
  static NimBLEUUID u(kServiceUuid);
  return u;
}

// Used only when the client owns the scan.
class ScanCB : public NimBLEScanCallbacks {
 public:
  explicit ScanCB(N2kBleClient &c) : c_(c) {}
  void onResult(const NimBLEAdvertisedDevice *dev) override { c_.offerAdvertisement(dev); }
 private:
  N2kBleClient &c_;
};

inline uint16_t msToConnUnits(uint16_t ms) { return (uint16_t)((uint32_t)ms * 4 / 5); }  // 1.25 ms units

}  // namespace

// ------------------------------------------------------------ registration
void N2kBleClient::onPGN(uint32_t pgn, Handler h) {
  std::lock_guard<std::recursive_mutex> g(dispMutex_);
  dispatcher_.onPGN(pgn, std::move(h));
}
void N2kBleClient::onPGN(uint32_t pgn, uint8_t source, Handler h) {
  std::lock_guard<std::recursive_mutex> g(dispMutex_);
  dispatcher_.onPGN(pgn, source, std::move(h));
}
void N2kBleClient::onAny(Handler h) {
  std::lock_guard<std::recursive_mutex> g(dispMutex_);
  dispatcher_.onAny(std::move(h));
}
void N2kBleClient::onConnection(std::function<void(bool)> cb) { connCb_ = std::move(cb); }

bool N2kBleClient::addModule(N2kBleClientModule &m) {
  if (started_ || moduleCount_ >= kMaxModules) return false;
  modules_[moduleCount_++] = &m;
  return true;
}

// ------------------------------------------------------------------- begin
bool N2kBleClient::begin(const Config &cfg) {
  if (started_) return true;
  cfg_ = cfg;
  if (!queue_.begin(cfg_.queueBytes)) return false;

  if (!NimBLEDevice::isInitialized()) NimBLEDevice::init("");
  NimBLEDevice::setMTU(cfg_.mtu);

  if (cfg_.bridgeAddress && *cfg_.bridgeAddress) {
    target_ = NimBLEAddress(std::string(cfg_.bridgeAddress), BLE_ADDR_PUBLIC);
    haveTarget_ = fixedTarget_ = true;
  }

  if (cfg_.ownScan && !fixedTarget_) {
    static ScanCB scanCb(*this);
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCb, false);
    scan->setActiveScan(cfg_.bridgeName != nullptr);  // the name is only in the scan response
    scan->setInterval(100);
    scan->setWindow(50);
  }

  client_ = NimBLEDevice::createClient();
  if (!client_) return false;
  uint16_t itvl = msToConnUnits(cfg_.connIntervalMs);
  if (itvl < 6) itvl = 6;
  client_->setConnectionParams(itvl, itvl + 8, 0, 400);  // 4 s supervision timeout
  client_->setConnectTimeout(cfg_.connectTimeoutMs);

  TaskHandle_t h = nullptr;
  BaseType_t ok = xTaskCreatePinnedToCore(taskEntry, "n2kble", cfg_.taskStack, this, cfg_.taskPriority, &h,
                                          cfg_.taskCore);
  taskHandle_ = h;
  started_ = ok == pdPASS;
  return started_;
}

// ---------------------------------------------------------- advertisements
void N2kBleClient::offerAdvertisement(const NimBLEAdvertisedDevice *dev) {
  if (fixedTarget_ || state_ != State::Searching || !dev) return;
  if (!dev->isAdvertisingService(serviceUuid())) return;
  if (cfg_.bridgeName && dev->getName() != cfg_.bridgeName) return;
  std::lock_guard<std::mutex> g(lock_);
  if (haveTarget_) return;
  target_ = dev->getAddress();
  haveTarget_ = true;
}

// -------------------------------------------------------- connection task
void N2kBleClient::taskEntry(void *arg) { static_cast<N2kBleClient *>(arg)->task(); }

void N2kBleClient::task() {
  uint32_t lastInfoMs = 0;
  for (;;) {
    switch (state_) {
      case State::Searching: {
        bool have;
        { std::lock_guard<std::mutex> g(lock_); have = haveTarget_; }
        if (have) {
          if (cfg_.ownScan && NimBLEDevice::getScan()->isScanning()) NimBLEDevice::getScan()->stop();
          state_ = State::Connecting;
          break;
        }
        if (cfg_.ownScan && !NimBLEDevice::getScan()->isScanning()) NimBLEDevice::getScan()->start(0, false, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      }

      case State::Connecting:
        if (connectAndSetup()) {
          failures_ = 0;
          connects_++;
          connEpoch_++;
          state_ = State::Connected;
          lastInfoMs = millis();
        } else {
          teardown();
          vTaskDelay(pdMS_TO_TICKS(cfg_.retryDelayMs));
          if (!fixedTarget_ && ++failures_ >= cfg_.failuresBeforeRescan) {
            failures_ = 0;
            std::lock_guard<std::mutex> g(lock_);
            haveTarget_ = false;  // that bridge is gone: look again
            state_ = State::Searching;
          }
        }
        break;

      case State::Connected:
        if (!client_->isConnected()) {
          teardown();
          connEpoch_++;
          state_ = State::Connecting;  // same bridge first
          vTaskDelay(pdMS_TO_TICKS(500));
          break;
        }
        {
          uint32_t rev;
          { std::lock_guard<std::recursive_mutex> g(dispMutex_); rev = dispatcher_.revision(); }
          if (rev != filterRevisionSent_) writeFilter();
        }
        if (millis() - lastInfoMs > 10000) {  // bridge counters, for stats()
          lastInfoMs = millis();
          NimBLERemoteService *svc = client_->getService(serviceUuid());
          NimBLERemoteCharacteristic *c = svc ? svc->getCharacteristic(kInfoUuid) : nullptr;
          if (c) {
            NimBLEAttValue v = c->readValue();
            Info i;
            if (decodeInfo(v.data(), v.length(), i)) { std::lock_guard<std::mutex> g(lock_); info_ = i; }
          }
        }
        if (cfg_.dispatchInTask) {
          drain();
          ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));  // woken by each notification
        } else {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        break;
    }
  }
}

bool N2kBleClient::connectAndSetup() {
  NimBLEAddress addr;
  { std::lock_guard<std::mutex> g(lock_); addr = target_; }
  if (!client_->connect(addr, true, false, true)) return false;

  NimBLERemoteService *svc = client_->getService(serviceUuid());
  if (!svc) return false;

  Info info;
  NimBLERemoteCharacteristic *infoChr = svc->getCharacteristic(kInfoUuid);
  if (infoChr) {
    NimBLEAttValue v = infoChr->readValue();
    if (!decodeInfo(v.data(), v.length(), info)) info = Info();
  }
  if (info.version != kProtocolVersion) {
    log_e("N2kBle: bridge speaks protocol v%u, this library v%u", info.version, kProtocolVersion);
    return false;
  }

  // Filter first, so the bridge doesn't start with everything.
  filterChr_ = svc->getCharacteristic(kFilterUuid);
  filterRevisionSent_ = 0xffffffff;
  writeFilter();

  NimBLERemoteCharacteristic *rx = svc->getCharacteristic(kRxStreamUuid);
  if (!rx || !rx->canNotify()) return false;
  bool sub = rx->subscribe(true, [this](NimBLERemoteCharacteristic *, uint8_t *d, size_t n, bool) { onNotify(d, n); });
  if (!sub) return false;

  for (uint8_t i = 0; i < moduleCount_; i++) {
    uint8_t need = modules_[i]->requiredFeatures();
    moduleActive_[i] = (info.features & need) == need && modules_[i]->onConnected(svc, *this);
  }

  std::lock_guard<std::mutex> g(lock_);
  info_ = info;
  address_ = addr.toString();
  return true;
}

void N2kBleClient::teardown() {
  for (uint8_t i = 0; i < moduleCount_; i++) {
    if (moduleActive_[i]) modules_[i]->onDisconnected();
    moduleActive_[i] = false;
  }
  filterChr_ = nullptr;
  if (client_->isConnected()) client_->disconnect();
}

void N2kBleClient::writeFilter() {
  std::vector<uint32_t> pgns;
  uint32_t rev;
  bool limited;
  {
    std::lock_guard<std::recursive_mutex> g(dispMutex_);
    limited = dispatcher_.wantedPgns(pgns);
    rev = dispatcher_.revision();
  }
  if (!filterChr_) { filterRevisionSent_ = rev; return; }  // an old bridge: gets everything
  uint8_t buf[kMaxFilterPgns * 3];
  size_t n = 0;
  if (limited && pgns.size() <= kMaxFilterPgns) n = encodeFilter(pgns.data(), pgns.size(), buf, sizeof(buf));
  // n == 0: every PGN (onAny() used, or more PGNs than the filter holds)
  if (filterChr_->writeValue(buf, n, true)) filterRevisionSent_ = rev;
}

// ---------------------------------------------------------------- receive
void N2kBleClient::onNotify(const uint8_t *data, size_t len) {  // BLE host task
  packets_++;
  queue_.push(data, len);
  if (cfg_.dispatchInTask && taskHandle_) xTaskNotifyGive((TaskHandle_t)taskHandle_);
}

void N2kBleClient::sink(const Record &r, void *ctx) {
  N2kBleClient *self = static_cast<N2kBleClient *>(ctx);
  std::lock_guard<std::recursive_mutex> g(self->dispMutex_);
  self->dispatcher_.dispatch(r, millis());
  self->messages_++;
}

void N2kBleClient::drain() {
  if (consumerEpoch_ != connEpoch_) {  // new link: a half record from the old one is garbage
    consumerEpoch_ = connEpoch_;
    reasm_.reset();
  }
  for (int i = 0; i < 64; i++) {  // bounded, so one busy second can't stall the UI
    size_t n = queue_.pop(pkt_, sizeof(pkt_));
    if (n == 0) break;
    reasm_.feed(pkt_, n, sink, this);
  }
}

void N2kBleClient::loop() {
  if (!started_) return;
  if (!cfg_.dispatchInTask) drain();

  bool c = connected();
  if (c != reportedConnected_) {
    reportedConnected_ = c;
    if (connCb_) connCb_(c);
  }
  for (uint8_t i = 0; i < moduleCount_; i++) if (moduleActive_[i]) modules_[i]->loop();
}

// ---------------------------------------------------------------- queries
uint32_t N2kBleClient::ageMs(uint32_t pgn) {
  std::lock_guard<std::recursive_mutex> g(dispMutex_);
  return dispatcher_.ageMs(pgn, millis());
}

std::string N2kBleClient::bridgeAddress() {
  std::lock_guard<std::mutex> g(lock_);
  return address_;
}

Info N2kBleClient::bridgeInfo() {
  std::lock_guard<std::mutex> g(lock_);
  return info_;
}

N2kBleClient::Stats N2kBleClient::stats() const {
  return {packets_.load(), messages_.load(), queue_.dropped.load(), reasm_.senderDropped, reasm_.errors, connects_.load()};
}
