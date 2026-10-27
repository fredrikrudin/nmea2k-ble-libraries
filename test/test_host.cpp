// nmea2k-ble-libraries - host unit tests (no ESP32 needed)
// Build: see test/run_tests.sh
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "N2kBleDispatcher.h"
#include "N2kBleProtocol.h"
#include "N2kBleQueue.h"

using namespace n2kble;

static int failures = 0;
#define CHECK(c)                                                    \
  do {                                                              \
    if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } \
  } while (0)

static Record makeRecord(uint32_t pgn, uint8_t len, uint8_t seed) {
  Record r;
  r.pgn = pgn;
  r.priority = 3;
  r.source = seed;
  r.destination = 0xff;
  r.length = len;
  for (int i = 0; i < len; i++) r.data[i] = (uint8_t)(seed + i);
  return r;
}

static bool same(const Record &a, const Record &b) {
  return a.pgn == b.pgn && a.priority == b.priority && a.source == b.source && a.destination == b.destination &&
         a.length == b.length && memcmp(a.data, b.data, a.length) == 0;
}

// Simulates bridge -> BLE -> client for one connection, with notify failures.
struct Link {
  RecordQueue q;
  Packetizer pz;
  Reassembler rx;
  std::vector<Record> got;
  uint16_t mtu;
  uint8_t pkt[600];
  int failEvery = 0, sends = 0;

  explicit Link(uint16_t m, size_t qbytes = 4096) : mtu(m) { q.begin(qbytes); }
  void push(const Record &r) {
    uint8_t buf[kMaxRecord];
    size_t n = encodeRecord(r, buf, sizeof(buf));
    q.push(buf, n);
  }
  static void sink(const Record &r, void *ctx) { static_cast<Link *>(ctx)->got.push_back(r); }
  void pump(int maxPackets = 1000) {
    for (int i = 0; i < maxPackets; i++) {
      uint8_t d = q.droppedForHeader();
      size_t len = pz.build(q, pkt, mtu, d);
      if (len == 0) return;
      CHECK(len <= (size_t)(mtu - kAttOverhead));
      sends++;
      if (failEvery && sends % failEvery == 0) { pz.rollback(); continue; }  // BLE busy
      q.pop(pz.commit());
      q.ackDropped(d);
      rx.feed(pkt, len, sink, this);
    }
  }
};

static void testRecordRoundTrip() {
  Record r = makeRecord(129029, 43, 7), out;
  uint8_t buf[kMaxRecord];
  size_t n = encodeRecord(r, buf, sizeof(buf));
  CHECK(n == kRecordHeader + 43);
  CHECK(decodeRecord(buf, n, out) == n);
  CHECK(same(r, out));
  CHECK(decodeRecord(buf, n - 1, out) == 0);  // truncated
  Record big = makeRecord(126996, 223, 1);
  CHECK(encodeRecord(big, buf, sizeof(buf)) == kMaxRecord);
  Record bad = makeRecord(0x40000, 1, 1);
  CHECK(encodeRecord(bad, buf, sizeof(buf)) == 0);
}

static void testManySmallRecordsShareAPacket() {
  Link l(247);
  for (int i = 0; i < 30; i++) l.push(makeRecord(128267, 8, i));
  l.pump();
  CHECK(l.got.size() == 30);
  CHECK(l.sends <= 2);  // 30 * 15 bytes = 450 -> two 241-byte payloads
  for (int i = 0; i < 30; i++) CHECK(same(l.got[i], makeRecord(128267, 8, i)));
}

static void testFragmentationAtSmallMtu() {
  Link l(23);  // default MTU: 17 payload bytes per packet
  l.push(makeRecord(126996, 134, 9));   // product info, fast-packet
  l.push(makeRecord(127250, 8, 1));     // heading
  l.push(makeRecord(129540, 223, 3));   // GNSS sats, max size
  l.pump();
  CHECK(l.got.size() == 3);
  CHECK(same(l.got[0], makeRecord(126996, 134, 9)));
  CHECK(same(l.got[1], makeRecord(127250, 8, 1)));
  CHECK(same(l.got[2], makeRecord(129540, 223, 3)));
  CHECK(l.rx.errors == 0);
}

static void testNotifyFailuresLoseNothing() {
  for (uint16_t mtu : {23, 64, 247}) {
    Link l(mtu, 16384);
    l.failEvery = 3;
    std::vector<Record> sent;
    for (int i = 0; i < 60; i++) {
      Record r = makeRecord(127000 + i, (uint8_t)((i * 37) % 224), i);
      sent.push_back(r);
      l.push(r);
    }
    l.pump(100000);
    CHECK(l.got.size() == sent.size());
    for (size_t i = 0; i < sent.size() && i < l.got.size(); i++) CHECK(same(l.got[i], sent[i]));
    CHECK(l.rx.errors == 0);
  }
}

static void testFullQueueDropsOldestAndReports() {
  Link l(247, 200);  // tiny queue
  for (int i = 0; i < 40; i++) l.push(makeRecord(128267, 8, i));  // 15 bytes each, 13 fit
  l.pump();
  CHECK(l.got.size() == 13);
  CHECK(l.got.back().source == 39);  // the newest survive
  CHECK(l.rx.senderDropped == 27);
}

static void testPushDuringFragmentation() {
  // New messages arrive while a big record is half sent and the queue overflows.
  Link l(23, 300);
  l.push(makeRecord(129540, 223, 3));
  l.pump(3);  // a few fragments out
  for (int i = 0; i < 40; i++) l.push(makeRecord(128267, 8, i));
  l.pump();
  CHECK(l.got.size() >= 2);
  CHECK(same(l.got[0], makeRecord(129540, 223, 3)));
  CHECK(l.rx.errors == 0);
}

static void testReassemblerRejectsBrokenChains() {
  Reassembler rx;
  int count = 0;
  auto sink = [](const Record &, void *c) { (*(int *)c)++; };
  uint8_t more[] = {kProtocolVersion, kPktFragMore, 0, 1, 2, 3};
  rx.feed(more, sizeof(more), sink, &count);  // MORE without FIRST
  CHECK(rx.errors == 1 && count == 0);
  uint8_t wrongVersion[] = {99, kPktRecords, 0};
  rx.feed(wrongVersion, sizeof(wrongVersion), sink, &count);
  CHECK(rx.errors == 2);
  uint8_t garbage[] = {kProtocolVersion, kPktRecords, 0, 0xff, 0xff, 0xff, 1, 2, 3, 250};
  rx.feed(garbage, sizeof(garbage), sink, &count);
  CHECK(rx.errors == 3 && count == 0);
}

static void testFilterAndInfo() {
  uint32_t in[] = {128267, 127488, 130310}, out[8];
  uint8_t buf[64];
  size_t n = encodeFilter(in, 3, buf, sizeof(buf));
  CHECK(n == 9);
  CHECK(decodeFilter(buf, n, out, 8) == 3);
  CHECK(out[0] == 128267 && out[1] == 127488 && out[2] == 130310);
  CHECK(decodeFilter(buf, 0, out, 8) == 0);   // empty = all
  CHECK(decodeFilter(buf, 4, out, 8) == -1);  // malformed

  Info i, o;
  i.features = kFeatureRx | kFeatureTx;
  i.busMessages = 123456789;
  i.dropped = 42;
  CHECK(encodeInfo(i, buf, sizeof(buf)) == kInfoSize);
  CHECK(decodeInfo(buf, kInfoSize, o));
  CHECK(o.features == 3 && o.busMessages == 123456789 && o.dropped == 42);
}

static void testPacketQueue() {
  PacketQueue q;
  q.begin(60);
  uint8_t a[30], b[30], out[64];
  memset(a, 1, sizeof(a));
  memset(b, 2, sizeof(b));
  CHECK(q.push(a, 30));
  CHECK(!q.push(b, 30));  // 32 + 32 > 60
  CHECK(q.dropped == 1);
  CHECK(q.pop(out, sizeof(out)) == 30 && out[0] == 1);
  CHECK(q.push(b, 30));   // wraps
  CHECK(q.pop(out, sizeof(out)) == 30 && out[29] == 2);
  CHECK(q.pop(out, sizeof(out)) == 0);
}

static void testDispatcher() {
  N2kBleDispatcher d;
  int depth = 0, any = 0, fromSrc = 0;
  d.onPGN(128267, [&](const tN2kMsg &m) { depth++; CHECK(m.DataLen == 8 && m.Data[0] == 5); });
  d.onPGN(127488, 23, [&](const tN2kMsg &) { fromSrc++; });
  std::vector<uint32_t> pgns;
  CHECK(d.wantedPgns(pgns) && pgns.size() == 2);

  d.dispatch(makeRecord(128267, 8, 5), 1000);
  Record e = makeRecord(127488, 8, 22);
  d.dispatch(e, 1000);  // source 22: not for the handler
  e.source = 23;
  d.dispatch(e, 1000);
  CHECK(depth == 1 && fromSrc == 1);
  CHECK(d.ageMs(128267, 1500) == 500);
  CHECK(d.ageMs(130310, 1500) == UINT32_MAX);

  uint32_t rev = d.revision();
  d.onAny([&](const tN2kMsg &) { any++; });
  CHECK(d.revision() != rev);
  CHECK(!d.wantedPgns(pgns));  // onAny -> everything
  d.dispatch(makeRecord(999999 & 0x3ffff, 1, 1), 2000);
  CHECK(any == 1);
}

// tN2kMsg's real parsers on a dispatched message: the point of the design.
#include <N2kMessages.h>
static void testRealParserThroughBle() {
  tN2kMsg src;
  SetN2kWaterDepth(src, 1, 12.34, 0.5, 100);
  src.Source = 35;
  Record r;
  r.pgn = src.PGN; r.priority = src.Priority; r.source = src.Source; r.destination = src.Destination;
  r.length = src.DataLen;
  memcpy(r.data, src.Data, src.DataLen);

  Link l(23);
  l.push(r);
  l.pump();
  CHECK(l.got.size() == 1);

  N2kBleDispatcher d;
  double gotDepth = -1;
  d.onPGN(128267, [&](const tN2kMsg &m) {
    unsigned char sid; double depth, offset, range;
    CHECK(ParseN2kWaterDepth(m, sid, depth, offset, range));
    CHECK(m.Source == 35);
    gotDepth = depth;
  });
  d.dispatch(l.got[0], 0);
  CHECK(gotDepth > 12.33 && gotDepth < 12.35);
}

int main() {
  testRecordRoundTrip();
  testManySmallRecordsShareAPacket();
  testFragmentationAtSmallMtu();
  testNotifyFailuresLoseNothing();
  testFullQueueDropsOldestAndReports();
  testPushDuringFragmentation();
  testReassemblerRejectsBrokenChains();
  testFilterAndInfo();
  testPacketQueue();
  testDispatcher();
  testRealParserThroughBle();
  printf(failures ? "%d FAILED\n" : "all tests passed\n", failures);
  return failures ? 1 : 0;
}
