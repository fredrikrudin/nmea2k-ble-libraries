// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleProtocol.h - the wire format shared by the bridge and the client.
//
// Pure C++ with no Arduino, NimBLE or NMEA2000 dependency, so it can be unit
// tested on a PC and reused by other implementations (a phone app, a Python
// script, ...). See PROTOCOL.md for the byte-level description.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace n2kble {

// ---------------------------------------------------------------- GATT layout
// One primary service. Every characteristic is optional for a client except
// the RX stream; a client checks the feature bits in Info before using one.
constexpr uint8_t kProtocolVersion = 1;

constexpr const char *kServiceUuid  = "b4060001-e746-4cf5-a400-0d257fc6e7ba";
constexpr const char *kRxStreamUuid = "b4060002-e746-4cf5-a400-0d257fc6e7ba";  // notify : bridge -> client, N2K messages
constexpr const char *kFilterUuid   = "b4060003-e746-4cf5-a400-0d257fc6e7ba";  // write  : client -> bridge, PGN filter
constexpr const char *kInfoUuid     = "b4060004-e746-4cf5-a400-0d257fc6e7ba";  // read   : version, features, counters
constexpr const char *kTxUuid       = "b4060005-e746-4cf5-a400-0d257fc6e7ba";  // write  : client -> bridge, N2K messages (TX module, reserved)

// Feature bits in Info. A bridge sets one bit per module it runs.
enum Feature : uint8_t {
  kFeatureRx = 0x01,  // RX stream + filter (always present)
  kFeatureTx = 0x02,  // TX characteristic (N2kBleBridgeTx module)
};

// ------------------------------------------------------------------- records
// One N2K message, after fast-packet / ISO multi-packet reassembly on the bus.
constexpr size_t kMaxData     = 223;  // = tN2kMsg::MaxDataLen
constexpr size_t kRecordHeader = 7;   // pgn(3, LE) priority src dst len
constexpr size_t kMaxRecord   = kRecordHeader + kMaxData;

struct Record {
  uint32_t pgn = 0;           // 18-bit PGN, as tN2kMsg::PGN
  uint8_t  priority = 6;
  uint8_t  source = 0;
  uint8_t  destination = 0xff;
  uint8_t  length = 0;        // bytes used in data
  uint8_t  data[kMaxData];
};

inline size_t recordSize(uint8_t dataLen) { return kRecordHeader + dataLen; }

// Writes the record to out. Returns the bytes written, 0 if it does not fit or is invalid.
size_t encodeRecord(const Record &r, uint8_t *out, size_t cap);
// Reads one record from in. Returns the bytes consumed, 0 if in is too short or invalid.
size_t decodeRecord(const uint8_t *in, size_t len, Record &out);
// Size of the encoded record starting at in (needs kRecordHeader bytes), 0 if invalid.
size_t peekRecordSize(const uint8_t *in, size_t len);

// ------------------------------------------------------------------- packets
// Every notification on the RX stream (and every write to TX) is one packet:
//   [0] protocol version
//   [1] packet type
//   [2] messages the sender dropped since its previous packet (saturates at 255)
//   [3..] payload
// RECORDS carries one or more whole records back to back. A record larger than
// one packet is split over FRAG_FIRST, FRAG_MORE..., FRAG_LAST. BLE delivers
// notifications in order, so fragments need no sequence numbers.
constexpr size_t kPacketHeader = 3;
constexpr size_t kAttOverhead  = 3;  // ATT notification header inside the MTU

enum PacketType : uint8_t {
  kPktRecords   = 1,
  kPktFragFirst = 2,
  kPktFragMore  = 3,
  kPktFragLast  = 4,
};

// Payload bytes available in one packet for a given ATT MTU.
inline size_t packetPayload(uint16_t mtu) {
  return mtu > kAttOverhead + kPacketHeader ? mtu - kAttOverhead - kPacketHeader : 0;
}

// Turns a stream of encoded records into packets (used by the sender).
// Feed it from a queue: it reads records with peek()/pop() callbacks so that a
// record is only removed once the packet carrying it was accepted by BLE.
class Packetizer {
 public:
  // Fills pkt (capacity = mtu - kAttOverhead) with as many records as fit.
  // src must expose
  //   size_t peek(uint8_t *buf, size_t cap, size_t skip)
  // which copies the record that starts `skip` bytes after the queue head into
  // buf and returns its size (0 = no record there).
  // Returns the packet length (0 = nothing to send). Call commit() after a
  // successful send, rollback() after a failed one.
  template <typename Source>
  size_t build(Source &src, uint8_t *pkt, uint16_t mtu, uint8_t dropped);
  // The packet from the last build() was sent. Returns how many bytes the
  // caller must now remove from the head of its queue (whole records only).
  // While a record is being fragmented, build() does not touch the queue.
  size_t commit();
  // The packet could not be sent: build() will rebuild the same content next time.
  void   rollback();
  bool   midFragment() const { return fragOffset_ != 0 || pendingFragOffset_ != 0; }

 private:
  uint8_t rec_[kMaxRecord];
  size_t  recLen_ = 0;             // size of the record being fragmented
  size_t  fragOffset_ = 0;         // bytes of it already sent
  size_t  pendingFragOffset_ = 0;  // offset after the packet waiting for commit
  size_t  pendingBytes_ = 0;      // queue bytes to pop on commit
  bool    pendingFrag_ = false;
};

// Reassembles packets into records (used by the receiver).
class Reassembler {
 public:
  using Sink = void (*)(const Record &r, void *ctx);
  // Returns the number of records delivered.
  size_t feed(const uint8_t *pkt, size_t len, Sink sink, void *ctx);
  void   reset() { have_ = 0; inFrag_ = false; }

  uint32_t records = 0;       // delivered
  uint32_t errors = 0;        // malformed packets / broken fragment chains
  uint32_t senderDropped = 0; // sum of the "dropped" header field
  uint8_t  lastVersion = 0;

 private:
  uint8_t buf_[kMaxRecord];
  size_t  have_ = 0;
  bool    inFrag_ = false;
};

// -------------------------------------------------------------------- filter
// Written by the client to kFilterUuid: a list of PGNs, 3 bytes LE each.
// An empty write means "every PGN". The newest write replaces the old filter.
constexpr size_t kMaxFilterPgns = 64;

size_t encodeFilter(const uint32_t *pgns, size_t n, uint8_t *out, size_t cap);
// Returns the number of PGNs read into pgns (at most maxPgns), or -1 if malformed.
int    decodeFilter(const uint8_t *in, size_t len, uint32_t *pgns, size_t maxPgns);

// ---------------------------------------------------------------------- info
// Read from kInfoUuid. Little endian.
struct Info {
  uint8_t  version = kProtocolVersion;
  uint8_t  features = kFeatureRx;
  uint8_t  n2kSource = 0xff;   // the bridge's own N2K address (0xff = listen only)
  uint8_t  clients = 0;        // connected clients
  uint32_t busMessages = 0;    // N2K messages seen on the bus since boot
  uint32_t dropped = 0;        // messages dropped because a client queue was full
};
constexpr size_t kInfoSize = 12;
size_t encodeInfo(const Info &i, uint8_t *out, size_t cap);
bool   decodeInfo(const uint8_t *in, size_t len, Info &out);

// ============================================================ implementation
template <typename Source>
size_t Packetizer::build(Source &src, uint8_t *pkt, uint16_t mtu, uint8_t dropped) {
  const size_t cap = packetPayload(mtu);
  if (cap < kRecordHeader) return 0;
  pendingBytes_ = 0;
  pendingFrag_ = false;
  pkt[0] = kProtocolVersion;
  pkt[2] = dropped;
  uint8_t *p = pkt + kPacketHeader;

  // Continue a record that is being sent in fragments
  if (fragOffset_ != 0) {
    size_t left = recLen_ - fragOffset_;
    size_t n = left < cap ? left : cap;
    pkt[1] = (n == left) ? kPktFragLast : kPktFragMore;
    for (size_t i = 0; i < n; i++) p[i] = rec_[fragOffset_ + i];
    pendingFrag_ = true;
    pendingFragOffset_ = (n == left) ? 0 : fragOffset_ + n;
    return kPacketHeader + n;  // the record left the queue with FRAG_FIRST
  }

  // Whole records, as many as fit; the caller pops them on commit.
  size_t used = 0;
  size_t len = src.peek(rec_, sizeof(rec_), 0);
  if (len == 0) return 0;
  if (len > cap) {  // too big for one packet: start fragmenting
    // rec_ keeps the whole record, so it leaves the queue as soon as the first
    // fragment is sent and the queue may drop old records freely meanwhile.
    recLen_ = len;
    pkt[1] = kPktFragFirst;
    for (size_t i = 0; i < cap; i++) p[i] = rec_[i];
    pendingFrag_ = true;
    pendingFragOffset_ = cap;
    pendingBytes_ = len;
    return kPacketHeader + cap;
  }
  pkt[1] = kPktRecords;
  while (len != 0 && used + len <= cap) {
    for (size_t i = 0; i < len; i++) p[used + i] = rec_[i];
    used += len;
    len = src.peek(rec_, sizeof(rec_), used);
  }
  pendingBytes_ = used;
  return kPacketHeader + used;
}

}  // namespace n2kble
