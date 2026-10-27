// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleDispatcher.cpp
#include "N2kBleDispatcher.h"

#include <string.h>

void N2kBleDispatcher::track(uint32_t pgn) {
  for (auto &s : seen_) if (s.pgn == pgn) return;
  seen_.push_back({pgn, 0, false});
}

void N2kBleDispatcher::onPGN(uint32_t pgn, Handler h) {
  entries_.push_back({pgn, -1, std::move(h)});
  track(pgn);
  revision_++;
}

void N2kBleDispatcher::onPGN(uint32_t pgn, uint8_t source, Handler h) {
  entries_.push_back({pgn, source, std::move(h)});
  track(pgn);
  revision_++;
}

void N2kBleDispatcher::onAny(Handler h) {
  any_.push_back(std::move(h));
  revision_++;
}

void N2kBleDispatcher::clear() {
  entries_.clear();
  any_.clear();
  seen_.clear();
  revision_++;
}

size_t N2kBleDispatcher::dispatch(const tN2kMsg &msg) {
  size_t ran = 0;
  for (auto &s : seen_) {
    if (s.pgn == msg.PGN) { s.lastMs = msg.MsgTime; s.ever = true; break; }
  }
  for (auto &h : any_) { h(msg); ran++; }
  for (auto &e : entries_) {
    if (e.pgn != msg.PGN) continue;
    if (e.source >= 0 && e.source != msg.Source) continue;
    e.fn(msg);
    ran++;
  }
  return ran;
}

void N2kBleDispatcher::toN2kMsg(const n2kble::Record &r, uint32_t rxTimeMs, tN2kMsg &out) {
  out.Init(r.priority, r.pgn, r.source, r.destination);
  memcpy(out.Data, r.data, r.length);
  out.DataLen = r.length;
  out.MsgTime = rxTimeMs;  // when it reached us, not when it was on the bus
}

size_t N2kBleDispatcher::dispatch(const n2kble::Record &r, uint32_t rxTimeMs) {
  tN2kMsg msg;
  toN2kMsg(r, rxTimeMs, msg);
  return dispatch(msg);
}

uint32_t N2kBleDispatcher::ageMs(uint32_t pgn, uint32_t nowMs) const {
  for (auto &s : seen_) {
    if (s.pgn == pgn) return s.ever ? nowMs - s.lastMs : UINT32_MAX;
  }
  return UINT32_MAX;
}

bool N2kBleDispatcher::wantedPgns(std::vector<uint32_t> &out) const {
  out.clear();
  if (!any_.empty()) return false;
  for (auto &s : seen_) out.push_back(s.pgn);
  return true;
}
