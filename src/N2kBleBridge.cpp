// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleBridge.cpp
#include "N2kBleBridge.h"

#include <Arduino.h>

using namespace n2kble;

// ------------------------------------------------- NimBLE callback adaptors
namespace {

class ServerCB : public NimBLEServerCallbacks {
 public:
  explicit ServerCB(N2kBleBridge &b) : b_(b) {}
  void onConnect(NimBLEServer *, NimBLEConnInfo &ci) override { b_.hostConnected(ci.getConnHandle(), ci.getMTU()); }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &ci, int) override { b_.hostDisconnected(ci.getConnHandle()); }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &ci) override { b_.hostMtu(ci.getConnHandle(), mtu); }
 private:
  N2kBleBridge &b_;
};

class RxCB : public NimBLECharacteristicCallbacks {
 public:
  explicit RxCB(N2kBleBridge &b) : b_(b) {}
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &ci, uint16_t sub) override {
    b_.hostSubscribe(ci.getConnHandle(), (sub & 1) != 0);
  }
 private:
  N2kBleBridge &b_;
};

class FilterCB : public NimBLECharacteristicCallbacks {
 public:
  explicit FilterCB(N2kBleBridge &b) : b_(b) {}
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &ci) override {
    NimBLEAttValue v = c->getValue();
    b_.hostFilter(ci.getConnHandle(), v.data(), v.length());
  }
 private:
  N2kBleBridge &b_;
};

}  // namespace

// ------------------------------------------------------------------- setup
bool N2kBleBridge::addModule(N2kBleBridgeModule &m) {
  if (started_ || moduleCount_ >= kMaxModules) return false;
  modules_[moduleCount_++] = &m;
  return true;
}

void N2kBleBridge::attachTo(tNMEA2000 &n2k) {
  n2k_ = &n2k;
  n2k.AttachMsgHandler(&handler_);
}

bool N2kBleBridge::begin(const Config &cfg) {
  if (started_) return true;
  cfg_ = cfg;
  if (cfg_.maxClients == 0) cfg_.maxClients = 1;
  if (cfg_.maxClients > kMaxClients) cfg_.maxClients = kMaxClients;
  if (cfg_.maxClients > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) cfg_.maxClients = CONFIG_BT_NIMBLE_MAX_CONNECTIONS;
  if (cfg_.mtu > sizeof(pkt_) + kAttOverhead) cfg_.mtu = sizeof(pkt_) + kAttOverhead;

  for (uint8_t i = 0; i < cfg_.maxClients; i++) {
    if (!sessions_[i].queue.begin(cfg_.queueBytes)) return false;
  }

  if (!NimBLEDevice::isInitialized()) NimBLEDevice::init(cfg_.deviceName);
  NimBLEDevice::setMTU(cfg_.mtu);
  NimBLEDevice::setPower(cfg_.txPowerDbm);

  static ServerCB serverCb(*this);  // one bridge per device
  static RxCB rxCb(*this);
  static FilterCB filterCb(*this);

  server_ = NimBLEDevice::createServer();
  server_->setCallbacks(&serverCb, false);
  server_->advertiseOnDisconnect(true);

  NimBLEService *svc = server_->createService(kServiceUuid);
  rxChr_ = svc->createCharacteristic(kRxStreamUuid, NIMBLE_PROPERTY::NOTIFY);
  rxChr_->setCallbacks(&rxCb);
  filterChr_ = svc->createCharacteristic(kFilterUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  filterChr_->setCallbacks(&filterCb);
  infoChr_ = svc->createCharacteristic(kInfoUuid, NIMBLE_PROPERTY::READ);

  for (uint8_t i = 0; i < moduleCount_; i++) {
    if (modules_[i]->setup(svc, *this)) features_ |= modules_[i]->features();
  }
  updateInfo();
  server_->start();  // also starts the service

  // Service UUID in the advertisement (works with passive scans), name in the
  // scan response: both do not fit in 31 bytes.
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData, scanData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.addServiceUUID(kServiceUuid);
  scanData.setName(cfg_.deviceName);
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->enableScanResponse(true);
  started_ = adv->start();
  return started_;
}

// --------------------------------------------------------- host task side
int N2kBleBridge::slotOf(uint16_t conn) const {
  for (uint8_t i = 0; i < cfg_.maxClients; i++) {
    if (shared_[i].active && shared_[i].conn == conn) return i;
  }
  return -1;
}

void N2kBleBridge::hostConnected(uint16_t conn, uint16_t mtu) {
  int slot = -1;
  {
    LockGuard g(lock_);
    for (uint8_t i = 0; i < cfg_.maxClients; i++) {
      if (!shared_[i].active) { slot = i; break; }
    }
    if (slot >= 0) {
      Shared &s = shared_[slot];
      s.active = true;
      s.conn = conn;
      s.mtu = mtu;
      s.subscribed = false;
      s.epoch++;
      s.filterCount = 0;
      s.filterDirty = true;
      connected_++;
    }
  }
  if (slot < 0) {  // full: refuse politely
    server_->disconnect(conn);
    return;
  }
  for (uint8_t i = 0; i < moduleCount_; i++) modules_[i]->onClientConnected(conn);
  if (connected_ < cfg_.maxClients) NimBLEDevice::startAdvertising();
}

void N2kBleBridge::hostDisconnected(uint16_t conn) {
  bool was = false;
  {
    LockGuard g(lock_);
    int slot = slotOf(conn);
    if (slot >= 0) {
      shared_[slot].active = false;
      shared_[slot].subscribed = false;
      connected_--;
      was = true;
    }
  }
  if (was) for (uint8_t i = 0; i < moduleCount_; i++) modules_[i]->onClientDisconnected(conn);
}

void N2kBleBridge::hostMtu(uint16_t conn, uint16_t mtu) {
  LockGuard g(lock_);
  int slot = slotOf(conn);
  if (slot >= 0) shared_[slot].mtu = mtu;
}

void N2kBleBridge::hostSubscribe(uint16_t conn, bool on) {
  LockGuard g(lock_);
  int slot = slotOf(conn);
  if (slot >= 0) shared_[slot].subscribed = on;
}

void N2kBleBridge::hostFilter(uint16_t conn, const uint8_t *data, size_t len) {
  uint32_t pgns[kMaxFilterPgns];
  int n = decodeFilter(data, len, pgns, kMaxFilterPgns);
  if (n < 0) return;  // malformed: keep the old filter
  LockGuard g(lock_);
  int slot = slotOf(conn);
  if (slot < 0) return;
  Shared &s = shared_[slot];
  memcpy(s.filter, pgns, n * sizeof(uint32_t));
  s.filterCount = n;
  s.filterDirty = true;
}

// --------------------------------------------------------- loop task side
bool N2kBleBridge::Session::wants(uint32_t pgn) const {
  if (filterCount == 0) return true;
  for (int i = 0; i < filterCount; i++) if (filter[i] == pgn) return true;
  return false;
}

void N2kBleBridge::syncSessions() {
  LockGuard g(lock_);
  for (uint8_t i = 0; i < cfg_.maxClients; i++) {
    Shared &s = shared_[i];
    Session &w = sessions_[i];
    if (w.epoch != s.epoch) {  // a new connection in this slot: start clean
      w.epoch = s.epoch;
      w.queue.clear();
      w.pz = Packetizer();
    }
    w.live = s.active && s.subscribed;
    w.conn = s.conn;
    w.mtu = s.mtu;
    if (s.filterDirty) {
      memcpy(w.filter, s.filter, s.filterCount * sizeof(uint32_t));
      w.filterCount = s.filterCount;
      s.filterDirty = false;
    }
    if (!s.active && w.queue.used()) w.queue.clear();
  }
}

void N2kBleBridge::handleN2kMsg(const tN2kMsg &msg) {
  busMessages_++;
  if (!started_ || connected_ == 0) return;
  syncSessions();  // cheap: a handful of fields under a spinlock

  Record r;
  if (msg.DataLen < 0 || msg.DataLen > (int)kMaxData) return;
  r.pgn = msg.PGN;
  r.priority = msg.Priority;
  r.source = msg.Source;
  r.destination = msg.Destination;
  r.length = (uint8_t)msg.DataLen;
  memcpy(r.data, msg.Data, r.length);
  size_t n = encodeRecord(r, rec_, sizeof(rec_));
  if (n == 0) return;

  for (uint8_t i = 0; i < cfg_.maxClients; i++) {
    Session &w = sessions_[i];
    if (!w.live || !w.wants(r.pgn)) continue;
    uint32_t before = w.queue.pendingDropped();
    w.queue.push(rec_, n);
    dropped_ += w.queue.pendingDropped() - before;
  }
}

void N2kBleBridge::loop() {
  if (!started_) return;
  syncSessions();

  for (uint8_t i = 0; i < cfg_.maxClients; i++) {
    Session &w = sessions_[i];
    if (!w.live) continue;
    for (uint8_t k = 0; k < cfg_.maxPacketsPerLoop; k++) {
      uint8_t dropped = w.queue.droppedForHeader();
      size_t len = w.pz.build(w.queue, pkt_, w.mtu, dropped);
      if (len == 0) break;
      if (!rxChr_->notify(pkt_, len, w.conn)) {  // BLE buffers full (or the link just went)
        w.pz.rollback();
        notifyBusy_++;
        break;
      }
      size_t bytes = w.pz.commit();
      w.queue.pop(bytes);
      w.queue.ackDropped(dropped);
      if (bytes) {
        // count records in the packet: whole-record packets only pop on send
        const uint8_t *p = pkt_ + kPacketHeader;
        size_t left = len - kPacketHeader;
        if (pkt_[1] == kPktRecords) {
          while (left >= kRecordHeader) {
            size_t sz = peekRecordSize(p, left);
            if (!sz || sz > left) break;
            sent_++;
            p += sz;
            left -= sz;
          }
        } else {
          sent_++;  // first fragment of a large record
        }
      }
    }
  }

  for (uint8_t i = 0; i < moduleCount_; i++) modules_[i]->loop();

  if (millis() - lastInfoMs_ > 1000) {
    lastInfoMs_ = millis();
    updateInfo();
  }
}

void N2kBleBridge::updateInfo() {
  if (!infoChr_) return;
  Info info;
  info.features = features_;
  info.n2kSource = n2k_ ? n2k_->GetN2kSource() : 0xff;
  info.clients = connected_;
  info.busMessages = busMessages_;
  info.dropped = dropped_;
  uint8_t buf[kInfoSize];
  size_t n = encodeInfo(info, buf, sizeof(buf));
  infoChr_->setValue(buf, n);
}

N2kBleBridge::Stats N2kBleBridge::stats() const {
  return {busMessages_, sent_, dropped_, notifyBusy_, connected_.load()};
}
