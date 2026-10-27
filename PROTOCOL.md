# N2kBle wire protocol, version 1

Everything is little endian.

## GATT

One primary service, `b4060001-e746-4cf5-a400-0d257fc6e7ba`, advertised in the advertising data (so passive scanners see it). The device name is in the scan response.

| UUID | Properties | Direction | Purpose |
|---|---|---|---|
| `b4060002-…` | notify | bridge → client | RX stream: packets of N2K messages |
| `b4060003-…` | write, write without response | client → bridge | PGN filter |
| `b4060004-…` | read | bridge → client | Info: version, features, counters |
| `b4060005-…` | write | client → bridge | TX stream (reserved for the TX module, feature bit `0x02`) |

All UUIDs share the suffix `-e746-4cf5-a400-0d257fc6e7ba`.

A client should request a large MTU (the reference client asks for 247). The protocol works at the default MTU of 23, with more fragmentation.

## Record (one N2K message)

| Offset | Size | Field |
|---|---|---|
| 0 | 3 | PGN (18 bits used) |
| 3 | 1 | priority (0–7) |
| 4 | 1 | source address |
| 5 | 1 | destination address (255 = global) |
| 6 | 1 | data length N (0–223) |
| 7 | N | data, as on the bus after fast-packet / multi-packet reassembly |

## Packet (one notification or write)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | protocol version (1) |
| 1 | 1 | type |
| 2 | 1 | messages the sender dropped since its previous packet (saturates at 255) |
| 3 | … | payload |

| Type | Payload |
|---|---|
| 1 `RECORDS` | one or more complete records, back to back |
| 2 `FRAG_FIRST` | first part of one record |
| 3 `FRAG_MORE` | next part |
| 4 `FRAG_LAST` | last part; the record is complete |

A sender only fragments a record that does not fit in one packet. BLE delivers notifications in order, so fragments carry no sequence number. A receiver drops a partial record when a fragment chain is broken (a `FRAG_MORE` without `FRAG_FIRST`, or a `RECORDS` before `FRAG_LAST`).

A receiver ignores packets with another protocol version or an unknown type.

## Filter

A write of 3 bytes per PGN (at most 64 PGNs). The bridge then sends only those PGNs to this client. An empty write means every PGN, which is also the state after connecting. Each write replaces the previous filter.

## Info

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | protocol version |
| 1 | 1 | features: `0x01` RX, `0x02` TX |
| 2 | 1 | the bridge's own N2K source address (255 if unknown) |
| 3 | 1 | connected clients |
| 4 | 4 | N2K messages seen on the bus since boot |
| 8 | 4 | messages dropped because a client queue was full (all clients) |

Later versions may append fields; readers must accept a longer value.
