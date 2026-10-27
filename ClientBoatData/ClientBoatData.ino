// nmea2k-ble-libraries - ClientBoatData example
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
//
// Maps the PGNs a boat display needs to functions, using the NMEA2000 library's
// own parsers. Only these PGNs are sent by the bridge (automatic filter).
// Values older than 5 s are shown as "--" (ageMs()).

#include <N2kBle.h>
#include <N2kMessages.h>

N2kBleClient n2k;

struct {
  double depth = NAN, stw = NAN, seaTemp = NAN, rpm = NAN, coolant = NAN, oilPressure = NAN, volts = NAN;
  double tank[4] = {NAN, NAN, NAN, NAN};  // by instance
} boat;

static double k2c(double k) { return k - 273.15; }

void setup() {
  Serial.begin(115200);

  n2k.onPGN(128267, [](const tN2kMsg &m) {  // water depth
    unsigned char sid;
    double depth, offset, range;
    if (ParseN2kWaterDepth(m, sid, depth, offset, range) && !N2kIsNA(depth)) boat.depth = depth;
  });

  n2k.onPGN(128259, [](const tN2kMsg &m) {  // speed through water
    unsigned char sid;
    double stw, sog;
    tN2kSpeedWaterReferenceType type;
    if (ParseN2kBoatSpeed(m, sid, stw, sog, type) && !N2kIsNA(stw)) boat.stw = stw * 1.9438445;  // kn
  });

  n2k.onPGN(130310, [](const tN2kMsg &m) {  // environmental: sea temperature
    unsigned char sid;
    double water, air, pressure;
    if (ParseN2kOutsideEnvironmentalParameters(m, sid, water, air, pressure) && !N2kIsNA(water)) boat.seaTemp = k2c(water);
  });

  n2k.onPGN(127488, [](const tN2kMsg &m) {  // engine rapid
    unsigned char inst;
    double rpm, boost;
    int8_t trim;
    if (ParseN2kEngineParamRapid(m, inst, rpm, boost, trim) && inst == 0 && !N2kIsNA(rpm)) boat.rpm = rpm;
  });

  n2k.onPGN(127489, [](const tN2kMsg &m) {  // engine dynamic
    unsigned char inst;
    double oilP, oilT, cool, alt, fuel, hours, coolP, fuelP;
    int8_t load, torque;
    tN2kEngineDiscreteStatus1 s1;
    tN2kEngineDiscreteStatus2 s2;
    if (!ParseN2kEngineDynamicParam(m, inst, oilP, oilT, cool, alt, fuel, hours, coolP, fuelP, load, torque, s1, s2)) return;
    if (inst != 0) return;
    if (!N2kIsNA(cool)) boat.coolant = k2c(cool);
    if (!N2kIsNA(oilP)) boat.oilPressure = oilP / 100000.0;  // bar
  });

  n2k.onPGN(127505, [](const tN2kMsg &m) {  // fluid level, every tank
    unsigned char inst;
    tN2kFluidType type;
    double level, capacity;
    if (ParseN2kFluidLevel(m, inst, type, level, capacity) && inst < 4 && !N2kIsNA(level)) boat.tank[inst] = level;
  });

  n2k.onPGN(127508, [](const tN2kMsg &m) {  // battery status
    unsigned char inst, sid;
    double v, a, t;
    if (ParseN2kDCBatStatus(m, inst, v, a, t, sid) && inst == 0 && !N2kIsNA(v)) boat.volts = v;
  });

  // To take one PGN from one device only (e.g. two engine gateways):
  // n2k.onPGN(127488, 23, handler);   // source address 23

  n2k.begin();
}

static void show(const char *name, double v, uint32_t pgn, const char *unit) {
  if (isnan(v) || n2k.ageMs(pgn) > 5000) Serial.printf("%-10s --\n", name);
  else Serial.printf("%-10s %.1f %s\n", name, v, unit);
}

void loop() {
  n2k.loop();

  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  Serial.println(n2k.connected() ? "---- bridge connected" : "---- no bridge");
  show("Depth", boat.depth, 128267, "m");
  show("STW", boat.stw, 128259, "kn");
  show("Sea temp", boat.seaTemp, 130310, "C");
  show("RPM", boat.rpm, 127488, "");
  show("Coolant", boat.coolant, 127489, "C");
  show("Oil", boat.oilPressure, 127489, "bar");
  show("Battery", boat.volts, 127508, "V");
  for (int i = 0; i < 4; i++) {
    char n[12];
    snprintf(n, sizeof(n), "Tank %d", i);
    show(n, boat.tank[i], 127505, "%");
  }
}
