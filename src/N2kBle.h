// nmea2k-ble-libraries - NMEA 2000 over Bluetooth Low Energy
// Copyright (c) 2026 Fredrik Rudin. Licensed CC BY-NC 4.0, see LICENSE.md
// N2kBle.h - include this one. Pulls in the bridge and the client; the linker
// drops whichever side your sketch doesn't use.
#pragma once
#include "N2kBleProtocol.h"
#include "N2kBleDispatcher.h"
#include "N2kBleModule.h"
#include "N2kBleBridge.h"
#include "N2kBleClient.h"

#define N2KBLE_VERSION "0.1.0"
