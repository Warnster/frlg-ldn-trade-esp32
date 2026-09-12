# gba_link — Gen 3 link-CABLE cart side (ESP32-S3 port of Celio)

The cart-facing half of the link-cable pivot (gba-switch-bridge `docs/25`). Emulates a Gen 3 link
**cable** peer to a real FireRed/LeafGreen cartridge, so trade blocks can be bridged to the Switch
over LDN on core0. **Everything on the one ESP32-S3** — Celio-Firmware (RP2040/Zephyr) is the
*reference*, not a deployed chip. The RFU wireless-adapter path it replaces is preserved at git tag
`wireless-rfu-wake-proven`.

## Layering (bottom → top)

| Layer | File | HW-dependent? | Status |
|---|---|---|---|
| Protocol vocabulary (LINKCMD/LINKTYPE, Gen 3 structs) | `gba_trade_proto.h` | no | **done, size-verified** |
| Physical link (16-bit UART-framed multiplayer transfer) | `gba_link.h` + `gba_link_*.c` | **yes (pins/timing)** | interface done; impl TODO (task #9) |
| Packet engine (handshake → CRC → 8-word command framing) | `gba_packet.{h,c}` | no | **done, host-tested** |
| Trade sections (setup → connection → lounge → disconnect) | `gba_trade_*.{h,c}` | no | TODO (task #7) |
| Bridge to core0/LDN (blocks ↔ RFU slots, session sync) | (core0 side) | no | TODO (task #10) |

## Celio → this port (file map)

| Celio-Firmware | here | notes |
|---|---|---|
| `src/link_defines.h`, `src/payloads/{pokemon,linkPlayer}.h` | `gba_trade_proto.h` | byte-exact structs, `_Static_assert`-guarded |
| `src/layers/linkLayer.{h,c}` + `pio_*.pio` | `gba_link.h` + `gba_link_*.c` | **PIO → bare-metal bit-bang** (S3 has no PIO) |
| `src/layers/packetLayer.{hpp,cpp}` | `gba_packet.{h,c}` | **ISR+semaphore → synchronous polling** |
| `src/sections/trade*.{hpp,cpp}` | `gba_trade_*.{h,c}` | the trade state machine |

## The one control-flow adaptation that matters

Celio drives its PIO from **interrupts** and hands finished packets to a **thread via semaphores**.
Our core1 is a **bare, non-preemptible polling loop** (no RTOS, no ISRs — see `gba_spi.c`). So the
packet engine is restructured as a synchronous state machine: the driver loop does
`next_tx → gba_link_transceive16 → on_transceive` and checks the return for "command packet
complete". Identical protocol and CRC (host-test: `pkt_test` — handshake→CRC→8-word command
completes, CRC = word sum); only the control flow is inverted.

## Why master at 115200 is tractable on the S3

The multiplayer link is **115200 baud ≈ 8.68µs/bit** — ~2000 core1 cycles per bit. Driving SC as
master bit-bangs with comfortable margin, unlike the 2MHz RFU wake. Celio's proven trade config is
device-as-master; slave mode (force-cart-master) is the fallback.

## OPEN before the physical layer (task #8, needs hardware to confirm)

Celio's devicetree maps GP0–GP4 = SC/SI/SO/SD(GBA)/SD(GBC), but its master PIO shifts the 16-bit
data on the pin it calls **SD** while standard GBA multiplayer uses **SI/SO** for data. Our cut
cable (`gba_spi.c`) is wired SC=GPIO12, SO=GPIO11 (in), SI=GPIO13 (out), SD=GPIO14 — which exposes
the SI/SO data lines, so the existing wiring is *expected* to suffice. The exact data-line/role
mapping is confirmed on the bench (logic-analyzer on the four lines) before trusting the physical
layer. Getting this wrong is the one unrecoverable mistake, so the timing code waits on it.
