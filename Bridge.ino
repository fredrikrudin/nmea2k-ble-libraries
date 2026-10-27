// nmea2k-ble-libraries - Bridge example
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
//
// An ESP32 on the NMEA 2000 bus. It listens to every message and streams them
// over BLE to the displays that connect to it (2 by default).
//
// Hardware: any ESP32 / ESP32-S3 with a CAN transceiver. The pins below are the
// Waveshare ESP32-S3-Touch-LCD-4 V4 (TWAI TX = GPIO6, RX = GPIO0).
//
// Libraries: NMEA2000 (Timo Lappalainen), NMEA2000_esp32_twai (Sergei P.),
//            NimBLE-Arduino 2.x, nmea2k-ble-libraries.

#define ESP32_CAN_TX_PIN GPIO_NUM_6
#define ESP32_CAN_RX_PIN GPIO_NUM_0
#include <NMEA2000_esp32_twai.h>
#include <N2kBle.h>

NMEA2000_esp32_twai n2k(ESP32_CAN_TX_PIN, ESP32_CAN_RX_PIN);
N2kBleBridge bridge;

void setup() {
  Serial.begin(115200);

  // Listen only: the bridge never writes to the bus. (A future TX module will
  // need N2km_ListenAndNode so it has its own address on the bus.)
  n2k.SetMode(tNMEA2000::N2km_ListenOnly);
  n2k.SetN2kCANMsgBufSize(32);
  n2k.SetN2kCANReceiveFrameBufSize(150);
  // Fast-packet PGNs the library does not know by default (e.g. proprietary
  // ones) must be listed, or they arrive as separate 8-byte frames:
  // static const unsigned long extraFast[] = { 130820L, 0 };
  // n2k.ExtendFastPacketMessages(extraFast);

  bridge.attachTo(n2k);  // every message the library receives goes to BLE
  n2k.Open();

  N2kBleBridge::Config cfg;
  cfg.deviceName = "N2K-BLE";
  cfg.maxClients = 2;
  if (!bridge.begin(cfg)) Serial.println("BLE start failed");
  Serial.printf("Bridge up, address %s\n", NimBLEDevice::getAddress().toString().c_str());
}

void loop() {
  n2k.ParseMessages();
  bridge.loop();

  static uint32_t last = 0;
  if (millis() - last > 5000) {
    last = millis();
    N2kBleBridge::Stats s = bridge.stats();
    Serial.printf("bus %lu msgs, sent %lu, dropped %lu, BLE busy %lu, clients %u\n",
                  (unsigned long)s.busMessages, (unsigned long)s.sent, (unsigned long)s.dropped,
                  (unsigned long)s.notifyBusy, s.clients);
  }
}
