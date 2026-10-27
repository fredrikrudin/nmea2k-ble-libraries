// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleProtocol.cpp - encode / decode of the wire format
#include "N2kBleProtocol.h"

#include <string.h>

namespace n2kble {

static inline void put24(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; }
static inline uint32_t get24(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16; }
static inline void put32(uint8_t *p, uint32_t v) { put24(p, v); p[3] = v >> 24; }
static inline uint32_t get32(const uint8_t *p) { return get24(p) | (uint32_t)p[3] << 24; }

// ------------------------------------------------------------------- records
size_t encodeRecord(const Record &r, uint8_t *out, size_t cap) {
  if (r.length > kMaxData || r.pgn > 0x3ffff) return 0;
  size_t n = recordSize(r.length);
  if (n > cap) return 0;
  put24(out, r.pgn);
  out[3] = r.priority & 7;
  out[4] = r.source;
  out[5] = r.destination;
  out[6] = r.length;
  memcpy(out + kRecordHeader, r.data, r.length);
  return n;
}

size_t peekRecordSize(const uint8_t *in, size_t len) {
  if (len < kRecordHeader) return 0;
  if (get24(in) > 0x3ffff || in[6] > kMaxData) return 0;
  return recordSize(in[6]);
}

size_t decodeRecord(const uint8_t *in, size_t len, Record &out) {
  size_t n = peekRecordSize(in, len);
  if (n == 0 || n > len) return 0;
  out.pgn = get24(in);
  out.priority = in[3] & 7;
  out.source = in[4];
  out.destination = in[5];
  out.length = in[6];
  memcpy(out.data, in + kRecordHeader, out.length);
  return n;
}

// ---------------------------------------------------------------- packetizer
size_t Packetizer::commit() {
  if (pendingFrag_) fragOffset_ = pendingFragOffset_;
  pendingFrag_ = false;
  pendingFragOffset_ = 0;
  size_t n = pendingBytes_;
  pendingBytes_ = 0;
  return n;
}

void Packetizer::rollback() {
  pendingFrag_ = false;
  pendingFragOffset_ = 0;
  pendingBytes_ = 0;
}

// --------------------------------------------------------------- reassembler
size_t Reassembler::feed(const uint8_t *pkt, size_t len, Sink sink, void *ctx) {
  if (len < kPacketHeader) { errors++; return 0; }
  lastVersion = pkt[0];
  if (pkt[0] != kProtocolVersion) { errors++; return 0; }  // a newer bridge: don't guess
  senderDropped += pkt[2];
  const uint8_t type = pkt[1];
  const uint8_t *p = pkt + kPacketHeader;
  size_t n = len - kPacketHeader;
  Record r;
  size_t delivered = 0;

  switch (type) {
    case kPktRecords: {
      if (inFrag_) { errors++; reset(); }  // the end of a fragmented record went missing
      while (n > 0) {
        size_t used = decodeRecord(p, n, r);
        if (used == 0) { errors++; break; }
        sink(r, ctx);
        delivered++;
        p += used;
        n -= used;
      }
      break;
    }
    case kPktFragFirst:
      if (inFrag_) errors++;  // previous chain broken, start over
      have_ = 0;
      inFrag_ = true;
      // fall through
    case kPktFragMore:
    case kPktFragLast: {
      if (!inFrag_ || have_ + n > sizeof(buf_)) { errors++; reset(); break; }
      memcpy(buf_ + have_, p, n);
      have_ += n;
      size_t want = peekRecordSize(buf_, have_);
      if (have_ >= kRecordHeader && want == 0) { errors++; reset(); break; }
      if (type == kPktFragLast) {
        if (want != have_ || decodeRecord(buf_, have_, r) == 0) errors++;
        else { sink(r, ctx); delivered++; }
        reset();
      }
      break;
    }
    default:
      errors++;  // unknown packet type from a newer bridge
  }
  records += delivered;
  return delivered;
}

// -------------------------------------------------------------------- filter
size_t encodeFilter(const uint32_t *pgns, size_t n, uint8_t *out, size_t cap) {
  if (n > kMaxFilterPgns || n * 3 > cap) return 0;
  for (size_t i = 0; i < n; i++) put24(out + 3 * i, pgns[i]);
  return n * 3;
}

int decodeFilter(const uint8_t *in, size_t len, uint32_t *pgns, size_t maxPgns) {
  if (len % 3 != 0) return -1;
  size_t n = len / 3;
  if (n > maxPgns) return -1;
  for (size_t i = 0; i < n; i++) pgns[i] = get24(in + 3 * i);
  return (int)n;
}

// ---------------------------------------------------------------------- info
size_t encodeInfo(const Info &i, uint8_t *out, size_t cap) {
  if (cap < kInfoSize) return 0;
  out[0] = i.version;
  out[1] = i.features;
  out[2] = i.n2kSource;
  out[3] = i.clients;
  put32(out + 4, i.busMessages);
  put32(out + 8, i.dropped);
  return kInfoSize;
}

bool decodeInfo(const uint8_t *in, size_t len, Info &out) {
  if (len < 4) return false;  // newer versions may append fields; older ones are shorter
  out.version = in[0];
  out.features = in[1];
  out.n2kSource = in[2];
  out.clients = in[3];
  out.busMessages = len >= 8 ? get32(in + 4) : 0;
  out.dropped = len >= 12 ? get32(in + 8) : 0;
  return true;
}

}  // namespace n2kble
