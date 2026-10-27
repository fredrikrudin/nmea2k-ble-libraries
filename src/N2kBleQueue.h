// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleQueue.h - byte ring buffers used by the bridge and the client.
//
//   RecordQueue : encoded records waiting to be sent to one client (bridge).
//                 Full -> the oldest records are dropped: on a display the
//                 newest value is the one that matters.
//   PacketQueue : notifications received from the bridge, waiting to be
//                 decoded in loop() (client). Written by the BLE host task,
//                 read by the application task, so it is locked.
//
// No Arduino dependency except the optional PSRAM allocation and the ESP32
// spinlock, both behind ESP_PLATFORM, so the queues can be tested on a PC.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>

#include "N2kBleProtocol.h"

#if defined(ESP_PLATFORM)
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#endif

namespace n2kble {

// Spinlock on ESP32 (safe between tasks on both cores), nothing elsewhere.
class Lock {
 public:
#if defined(ESP_PLATFORM)
  void lock() { portENTER_CRITICAL(&mux_); }
  void unlock() { portEXIT_CRITICAL(&mux_); }
 private:
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
#else
  void lock() {}
  void unlock() {}
#endif
};

struct LockGuard {
  explicit LockGuard(Lock &l) : l_(l) { l_.lock(); }
  ~LockGuard() { l_.unlock(); }
  Lock &l_;
};

// Buffers go to PSRAM when the board has it: internal RAM is what NimBLE,
// WiFi and LVGL draw buffers fight over.
inline uint8_t *allocBuffer(size_t n) {
#if defined(ESP_PLATFORM)
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_8BIT);
  return (uint8_t *)p;
#else
  return (uint8_t *)malloc(n);
#endif
}
inline void freeBuffer(uint8_t *p) {
#if defined(ESP_PLATFORM)
  heap_caps_free(p);
#else
  free(p);
#endif
}

// Plain byte ring. Not locked; the wrappers below decide.
class ByteRing {
 public:
  ByteRing() = default;
  ByteRing(const ByteRing &) = delete;
  ByteRing &operator=(const ByteRing &) = delete;
  ~ByteRing() { if (buf_) freeBuffer(buf_); }

  bool begin(size_t capacity) {
    if (buf_) freeBuffer(buf_);
    buf_ = allocBuffer(capacity);
    cap_ = buf_ ? capacity : 0;
    head_ = used_ = 0;
    return buf_ != nullptr;
  }
  size_t capacity() const { return cap_; }
  size_t used() const { return used_; }
  size_t freeSpace() const { return cap_ - used_; }
  void   clear() { head_ = used_ = 0; }

  // All or nothing.
  bool push(const uint8_t *d, size_t n) {
    if (n > freeSpace()) return false;
    size_t tail = (head_ + used_) % cap_;
    size_t first = cap_ - tail < n ? cap_ - tail : n;
    memcpy(buf_ + tail, d, first);
    memcpy(buf_, d + first, n - first);
    used_ += n;
    return true;
  }
  // Copies n bytes starting `skip` bytes after the head. False if not that many.
  bool copyOut(size_t skip, uint8_t *d, size_t n) const {
    if (skip + n > used_) return false;
    size_t start = (head_ + skip) % cap_;
    size_t first = cap_ - start < n ? cap_ - start : n;
    memcpy(d, buf_ + start, first);
    memcpy(d + first, buf_, n - first);
    return true;
  }
  void pop(size_t n) {
    if (n > used_) n = used_;
    head_ = (head_ + n) % (cap_ ? cap_ : 1);
    used_ -= n;
    if (used_ == 0) head_ = 0;
  }

 private:
  uint8_t *buf_ = nullptr;
  size_t cap_ = 0, head_ = 0, used_ = 0;
};

// Encoded records for one client. Used from one task only (the bridge loop).
class RecordQueue {
 public:
  bool begin(size_t capacity) { dropped_ = 0; return ring_.begin(capacity); }
  void clear() { ring_.clear(); dropped_ = 0; }
  size_t used() const { return ring_.used(); }

  // Adds an encoded record, dropping the oldest records if there is no room.
  // Returns false only if the record can never fit.
  bool push(const uint8_t *rec, size_t n) {
    if (n > ring_.capacity()) { dropped_++; return false; }
    while (ring_.freeSpace() < n) {
      uint8_t hdr[kRecordHeader];
      if (!ring_.copyOut(0, hdr, sizeof(hdr))) { ring_.clear(); break; }
      size_t sz = peekRecordSize(hdr, sizeof(hdr));
      if (sz == 0) { ring_.clear(); break; }  // cannot happen; recover anyway
      ring_.pop(sz);
      dropped_++;
    }
    return ring_.push(rec, n);
  }
  // Interface for Packetizer::build().
  size_t peek(uint8_t *buf, size_t cap, size_t skip) const {
    uint8_t hdr[kRecordHeader];
    if (!ring_.copyOut(skip, hdr, sizeof(hdr))) return 0;
    size_t sz = peekRecordSize(hdr, sizeof(hdr));
    if (sz == 0 || sz > cap || !ring_.copyOut(skip, buf, sz)) return 0;
    return sz;
  }
  void pop(size_t bytes) { ring_.pop(bytes); }

  // Records dropped and not yet reported in a packet header (saturating at 255).
  uint8_t droppedForHeader() const { return dropped_ > 255 ? 255 : (uint8_t)dropped_; }
  // The packet carrying droppedForHeader() was sent.
  void    ackDropped(uint8_t d) { dropped_ -= d < dropped_ ? d : dropped_; }
  uint32_t pendingDropped() const { return dropped_; }

 private:
  ByteRing ring_;
  uint32_t dropped_ = 0;
};

// Raw packets, [len16][bytes] each. Written by one task, read by another.
class PacketQueue {
 public:
  // Call before any other task uses the queue (allocates; no spinlock around malloc).
  bool begin(size_t capacity) { dropped = 0; return ring_.begin(capacity); }
  void clear() { LockGuard g(lock_); ring_.clear(); }

  // Full -> the new packet is dropped (the reader may be mid-way through the old ones).
  bool push(const uint8_t *pkt, size_t n) {
    if (n > 0xffff) return false;
    uint8_t len[2] = {(uint8_t)n, (uint8_t)(n >> 8)};
    LockGuard g(lock_);
    if (ring_.freeSpace() < n + 2) { dropped++; return false; }
    ring_.push(len, 2);
    ring_.push(pkt, n);
    return true;
  }
  // Copies the oldest packet into buf. Returns its size, 0 if none.
  // A packet larger than cap is discarded (and counted) rather than blocking the queue.
  size_t pop(uint8_t *buf, size_t cap) {
    LockGuard g(lock_);
    uint8_t len[2];
    if (!ring_.copyOut(0, len, 2)) return 0;
    size_t n = len[0] | (size_t)len[1] << 8;
    if (n > cap) { ring_.pop(2 + n); dropped++; return 0; }
    ring_.copyOut(2, buf, n);
    ring_.pop(2 + n);
    return n;
  }
  std::atomic<uint32_t> dropped{0};

 private:
  ByteRing ring_;
  Lock lock_;
};

}  // namespace n2kble
