// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBleDispatcher.h - maps received PGNs to your functions.
//
// Handlers receive a normal tN2kMsg, so every ParseN2k...() function of the
// NMEA2000 library works on it, and a handler written for a wired CAN bus
// (tNMEA2000::SetMsgHandler) can be reused unchanged with onAny().
#pragma once
#include <N2kMsg.h>
#include <stdint.h>

#include <functional>
#include <vector>

#include "N2kBleProtocol.h"

class N2kBleDispatcher {
 public:
  using Handler = std::function<void(const tN2kMsg &msg)>;

  // Called for every message with this PGN. Several handlers per PGN are fine;
  // they run in the order they were added.
  void onPGN(uint32_t pgn, Handler h);
  // Same, but only for messages from one N2K source address.
  void onPGN(uint32_t pgn, uint8_t source, Handler h);
  // Called for every message, before the PGN handlers.
  void onAny(Handler h);
  void clear();

  // Delivers one message to the matching handlers. Returns how many ran.
  size_t dispatch(const tN2kMsg &msg);
  size_t dispatch(const n2kble::Record &r, uint32_t rxTimeMs);

  // Milliseconds since a PGN was last received (any source), or UINT32_MAX if
  // never. Tracked for PGNs that have a handler; use it to blank stale values.
  uint32_t ageMs(uint32_t pgn, uint32_t nowMs) const;

  // The PGNs the bridge has to send: the handler list, or an empty list
  // (= everything) when onAny() is used. Returns false when "everything".
  bool wantedPgns(std::vector<uint32_t> &out) const;
  // Bumped whenever handlers change, so the client knows to resend its filter.
  uint32_t revision() const { return revision_; }

  static void toN2kMsg(const n2kble::Record &r, uint32_t rxTimeMs, tN2kMsg &out);

 private:
  struct Entry {
    uint32_t pgn;
    int16_t  source;  // -1 = any
    Handler  fn;
  };
  struct Seen {
    uint32_t pgn;
    uint32_t lastMs;
    bool     ever;
  };
  std::vector<Entry>   entries_;
  std::vector<Handler> any_;
  std::vector<Seen>    seen_;
  uint32_t revision_ = 0;
  void track(uint32_t pgn);
};
