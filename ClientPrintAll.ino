// nmea2k-ble-libraries - ClientPrintAll example
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
//
// Connects to the first bridge it finds and prints every NMEA 2000 message.
// Good for seeing what your bus actually sends before writing handlers.

#include <N2kBle.h>

N2kBleClient n2k;

void setup() {
  Serial.begin(115200);

  // onAny(): every message, and it tells the bridge to send every PGN.
  n2k.onAny([](const tN2kMsg &m) {
    Serial.printf("%8lu PGN %6lu src %3u dst %3u prio %u len %3d :", (unsigned long)m.MsgTime,
                  (unsigned long)m.PGN, m.Source, m.Destination, m.Priority, m.DataLen);
    for (int i = 0; i < m.DataLen && i < 16; i++) Serial.printf(" %02X", m.Data[i]);
    Serial.println(m.DataLen > 16 ? " ..." : "");
  });

  n2k.onConnection([](bool up) {
    Serial.printf(up ? "Connected to bridge %s\n" : "Bridge lost%s\n", up ? n2k.bridgeAddress().c_str() : "");
  });

  n2k.begin();  // scans, connects, reconnects - all in the background
}

void loop() {
  n2k.loop();  // the handler above runs here

  static uint32_t last = 0;
  if (millis() - last > 10000) {
    last = millis();
    N2kBleClient::Stats s = n2k.stats();
    Serial.printf("-- %lu msgs, %lu lost in bridge, %lu lost here, %lu errors\n", (unsigned long)s.messages,
                  (unsigned long)s.bridgeDropped, (unsigned long)s.queueDropped, (unsigned long)s.errors);
  }
}
