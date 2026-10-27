#!/bin/sh
# Runs the host unit tests. Needs g++ and a checkout of the NMEA2000 library:
#   NMEA2000_SRC=/path/to/NMEA2000/src ./test/run_tests.sh
set -e
cd "$(dirname "$0")/.."
: "${NMEA2000_SRC:?set NMEA2000_SRC to the NMEA2000 library src folder}"
OUT=$(mktemp -d)
g++ -std=c++17 -Wall -Wextra -O1 -g -fsanitize=address,undefined \
  -Isrc -I"$NMEA2000_SRC" \
  test/test_host.cpp test/host_shims.cpp src/N2kBleProtocol.cpp src/N2kBleDispatcher.cpp \
  "$NMEA2000_SRC/N2kMsg.cpp" "$NMEA2000_SRC/N2kMessages.cpp" "$NMEA2000_SRC/N2kStream.cpp" \
  -o "$OUT/test_host"
"$OUT/test_host"
