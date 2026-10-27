// The NMEA2000 library needs a millisecond clock off-target.
#include <stdint.h>
#include <chrono>
uint64_t N2kMillis64() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
uint32_t N2kMillis() { return (uint32_t)N2kMillis64(); }
