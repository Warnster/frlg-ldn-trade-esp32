# ESP32-S3 as the GBA Wireless-Adapter front-end (drop the RP2040)

**Branch:** `s3-spi-slave-gba` (frlg-ldn-trade-esp32). **Goal:** wire a real GBA link port straight
into the ESP32-S3 and have the S3 *emulate the RFU wireless adapter* using the S3's **hardware SPI
slave** peripheral, so one chip does both the GBA link AND the LDN Wi-Fi — eliminating the RP2040 and
the whole ESP↔Pico UART bridge (the source of this week's grief: the GPIO21/WS2812 TX bug + the 32-byte
RX-FIFO overflow). See memory `esp32-s3-direct-gba-spi` for the feasibility research this builds on.

This doc is the design + the go/no-go test plan. Written for autonomous overnight progress; the
timing-critical parts CANNOT be validated without the physical GBA, so they are implemented as a
testable skeleton with the risky steps clearly marked.

---

## 1. Why hardware SPI *slave* (not bit-bang)

The RP2040 exists because the GBA link was **software bit-banged**, and software bit-banging cannot
survive Wi-Fi ISR jitter (`docs/01-architecture.md`: "Linux cannot promise 800µs"; one extra
instruction shifted every read one bit). But in a GBA↔adapter session **the GBA is the SPI master and
drives the clock (SC)**. If the S3's SPI *slave* peripheral shifts the 32-bit words, the **silicon**
clocks the bits — Wi-Fi jitter can only affect the gaps *between* words, never the bits. The GBA's
2 MHz clock is far below the S3 slave's ~10 MHz reliable ceiling. This is the whole unlock.

## 2. GBA link electrical facts (sourced)

- GBA "Normal Mode" serial, **32-bit words, MSB-first**. GBA drives **SC** at **2 MHz** (command
  mode) / **256 kHz** (init). Idle placeholder word = **`0x80000000`**.
- Link-port pins (from `gba-switch-bridge/docs/03-hardware.md`), **GBA's perspective**:
  - pin 2 **SO** = GBA serial OUT (GBA→adapter data). Adapter *reads* it  → S3 **MOSI in**.
  - pin 3 **SI** = GBA serial IN  (adapter→GBA data). Adapter *drives* it → S3 **MISO out**.
  - pin 4 **SD** = reset/ready line (NOT a per-word chip-select).
  - pin 5 **SC** = clock, GBA-driven                                 → S3 **SCLK in**.
  - pin 6 GND. pin 1 VCC 3.3V (leave unconnected; S3 powers itself). No level shifters (both 3.3V).
- **SPI mode is TBD-by-test.** Sources conflict (davidgf: "clock idle low" → Mode 0/1; the survey said
  Mode 3). The empirical truth is in the *working* Pico bit-bang (`GBA-Global-Link .../spi.rs
  transfer_bit`): it drives TX while SC is **low** and samples RX on the **low→high (rising)** edge.
  Data-out-on-falling + sample-on-rising ⇒ start with **CPOL=1, CPHA=1 (Mode 3)** and, if login bytes
  come out bit-shifted/garbled, try Mode 0. This is the FIRST go/no-go dial.

## 3. The between-word handshake (the hard part)

From afska `docs/wireless_adapter.md`. Between every 32-bit word there is an out-of-band handshake on
the **SI/SO lines themselves** (same wires as the data), so those pins must alternate between GPIO
(handshake) and the SPI peripheral (data burst).

**GBA-initiated transfer (normal case, GBA has a command for us):**
1. GBA drives **SO low** (as soon as it can).
2. Adapter drives **SI high**.
3. GBA drives **SO high**.
4. Adapter drives **SI low** *when ready*.
5. GBA drives **SO low** when ready.
6. GBA pulses **SC** ×32; both exchange one 32-bit word.

**Adapter-initiated transfer (we have data to push to the GBA — the "interrupt"):**
1. Adapter drives **SI low** (as soon as it can).
2. GBA drives **SO high**.
3. Adapter drives **SI high**.
4. GBA drives **SO low** when ready — **but waits ≥40µs**.
5. Adapter drives **SI low** when ready.
6. Adapter... clock still comes from the GBA (master role-switch is logical, not electrical) ×32.

**Deadlines:** adapter must complete the ack within **~800µs** or the GBA gives up; the ≥**40µs**
wait in the inverted direction; the session drops after **500ms** of the adapter having nothing to say.

**PROVEN handshake (use this — from the working RP2040 `gpi.rs`, simpler than the afska theory above):**
the RP2040 adapter does, per post-login word: (a) drive **SI low** = "ready to send", (b) the 32-bit
transfer, (c) `ack_recv`: SI low → ~2µs → SI high → **wait for SO to go high** (the GBA's ack), with a
give-up timeout. LOGIN is different: **bare back-to-back 32-bit transfers, NO handshake** (`login.rs`),
running the fixed challenge/response `0x00000000`→…→`0xB0BB8001`. `gba_spi.c` implements exactly this;
the afska 5-step is retained here only as cross-reference if the proven path needs debugging.

**Consequence for the S3:** SC stays wired to the SPI-slave SCLK the whole time (idle = no edges, so
the peripheral does nothing between words). SO (MOSI-in) can be read as a raw GPIO level *without*
detaching from the peripheral (ESP allows reading a pin routed to a peripheral input). **Only SI (the
adapter's MISO output) must switch**: GPIO-driven during the handshake, then handed to the SPI
peripheral just before step 6 so it outputs the data word. That GPIO⇄peripheral handoff on SI, done
inside the ~800µs window under live Wi-Fi, is THE risk to validate.

## 4. Chip-select framing (no per-word CS on the GBA)

The ESP `spi_slave` driver frames a transaction by CS. The GBA has none. Two strategies:

- **Strategy A (try first): CS held asserted, fixed 32-bit transactions.** Tie the slave CS to a GPIO
  we hold LOW (or to GND). Post 4-byte transactions; each should complete after 32 SC edges. Pre-queue
  several so the DMA auto-advances word-to-word without per-word CPU re-arm (relaxes the deadline a
  lot). RISK: must confirm the driver completes a transaction on *length* (32 clocks) and not only on
  a CS edge. If it only completes on CS-deassert, fall back to B.
- **Strategy B (fallback): drive CS ourselves per word.** Use the handshake completion (step 5→6) to
  assert CS low, and a **PCNT** unit counting SC edges (or an SC-idle timeout) to deassert after 32.
  More moving parts, tighter timing.

## 5. Pin plan (S3, avoiding known conflicts)

Avoid: **GPIO21** (WS2812 LED — this week's TX bug), **GPIO26–37** (octal PSRAM on the N8R8),
GPIO43/44 (USB-serial console), GPIO0/3/45/46 (strapping). Use **IO_MUX**-capable pins for the SPI
signals (avoids GPIO-matrix setup-time violations at speed). Candidate (confirm against FSPI IO_MUX):

| GBA | dir (S3) | S3 signal | GPIO (S3 FSPI IO_MUX) |
|-----|----------|-----------|----------------|
| SC (pin5) | in  | FSPICLK    | **GPIO12** |
| SO (pin2) | in  | FSPID (MOSI) | **GPIO11** |
| SI (pin3) | out | FSPIQ (MISO) | **GPIO13** |
| CS (internal) | out/tie | FSPICS0 | **GPIO10** (drive low / tie GND — see §4) |
| SD (pin4) | i/o | GPIO (handshake/reset) | **GPIO14** (free; not an SPI data pin) |
| GND | — | GND | GND |

FSPI = SPI2. GPIO10–13 are the S3's FSPI IO_MUX pins (fast path, no GPIO-matrix setup-time penalty):
CS0=10, D/MOSI=11, CLK=12, Q/MISO=13. All of 9–14 are free (PSRAM uses 26–37; LED=8; Pico-UART 7/47
is unused in this mode). Confirm exposure on the Waveshare S3-Zero edge before wiring.

## 6. Software architecture on the one S3

- **core1:** the `gba_spi` task — the GBA-facing adapter. Owns the SPI-slave peripheral + the handshake
  FSM. Runs the WAP/RFU adapter protocol (login, command routing) that TODAY lives in the Pico's
  `routes.rs`/`wap.rs`. Produces the GBA's outgoing trade slots and consumes the Switch's slots.
- **core0:** the existing LDN Wi-Fi/Pia/NI brain (`ldn_probe`/`ldn_brain`/`sim`) — unchanged. Pin the
  Wi-Fi/LwIP stack here (`CONFIG_ESP_WIFI_TASK_CORE_ID`, `CONFIG_LWIP_TCPIP_TASK_AFFINITY`).
- The `gba_spi` engine replaces `ldn_pico` as the slot source/sink: it feeds `sim`'s engine callbacks
  directly (in-process), so NO UART, NO AA55 frames, NO 32-byte FIFO. The two halves talk via a couple
  of lock-free ring buffers in RAM.

## 7. What to port from the RP2040 (the adapter protocol)

The S3 now IS the adapter, so it needs the Pico's `GBA-Global-Link/adapter/src/comms/` logic in C:
- **login** (`wap.rs::login`) — the adapter handshake that gets the GBA to `0xB0BB8001`.
- **command routing** (`routes.rs::local_respond`) — GetSomeValue(0x13), Unknown(0x11),
  BroadcastReadPoll(0x1d)→peer, Connect(0x1f), IsConnecting(0x20), ReceiveData(0x26/0x28)→slot,
  SendDataAndWait(0x25)→outgoing slot + async_ack, ReceiveDataAndWait(0x27) async_ack.
- **WAP framing** (`wap.rs`) — the `0x9966<<16 | size<<8 | cmd` header + word count.
This is a mechanical but sizable port; the Rust is the reference. NOTE the timing that `async_ack`
fakes (the ~800µs completion) is now enforced by the real handshake FSM, not faked.

## 8. GO / NO-GO TEST PLAN (run on hardware, in order)

Each gate is cheap and decisive; stop at the first hard failure.
1. **Clock capture:** GBA in wireless mode, S3 SPI-slave configured, log raw 32-bit words received.
   PASS = the login words appear (even if wrong mode → garbled but present). Confirms SC/SO wiring +
   that the slave shifts. Tune SPI mode (Mode 3↔0) until login reads match the Pico's known values.
2. **Handshake:** implement §3 GBA-initiated FSM; PASS = the GBA proceeds past the first word (doesn't
   give up after 800µs) → login completes to `0xB0BB8001`.
3. **Under Wi-Fi load:** repeat gate 2 with LDN Wi-Fi running on core0. PASS = login still completes.
   **THIS is the make-or-break gate for dropping the RP2040.**
4. **Full session:** union-room entry, BroadcastRead, the whole adapter protocol port, then the trade.

If gate 3 fails (SI GPIO⇄peripheral handoff can't be met inside ~800µs under Wi-Fi), the RP2040 stays
justified and we keep the two-chip design. That is an acceptable, well-scoped answer either way.

## 9. Status (autonomous session 2026-09-09 night)

DONE (compiles clean; host logic tested; NOT yet hardware-tested):
- [x] Branch `s3-spi-slave-gba`, this design doc.
- [x] `gba_spi` component (`components/gba_spi/`): SPI-slave init, the PROVEN per-word handshake
      (§3), raw login exchange, and `gba_spi_run_adapter()` (login + WAP command loop). Behind
      `CONFIG_GBA_SPI_DIRECT_SELFTEST` (default n) so the working LDN build is byte-identical.
      Verified: builds with the flag OFF (LDN binary unchanged) AND ON (selftest links).
- [x] WAP/adapter protocol port from RP2040 → `gba_wap.c/.h`: login challenge/response table
      (`login.rs`), WAP framing (`packet.rs`), command routing (`routes.rs::local_respond`).
- [x] Host unit test `components/gba_spi/test/test_gba_wap.c` — **PASSES** (login chain, framing,
      GetSomeValue/Unknown/BroadcastPoll/Connect→IsConnecting/SendData→async_ack/RecvData, no-peer).
      Build+run: `gcc -Icomponents/gba_spi components/gba_spi/gba_wap.c
      components/gba_spi/test/test_gba_wap.c -o /tmp/t && /tmp/t`

NOT DONE (need the physical GBA / next session):
- [ ] Wire `gba_spi_run_adapter`'s providers to the real LDN side (sim engine) instead of the mocks —
      i.e. replace `ldn_pico`/`ldn_brain`'s UART engine with in-process ring buffers. (Design §6.)
- [ ] Hardware gates 1–4 (§8): the whole timing story — SPI mode, the handshake, the SI
      GPIO⇄peripheral handoff, and Strategy-A CS framing — is UNVALIDATED without a real GBA.
- [ ] CS framing (§4-A): confirm the slave completes a transaction on length (32 clocks) with CS
      tied low; if not, implement Strategy B (PCNT-driven CS).

HOW TO RUN THE FIRST HARDWARE TEST (when the user is back):
1. Wire GBA→S3 per §5 (SC=12, SO=11, SI=13, tie CS/GPIO10 to GND, SD=14, GND). GBA link cable only.
2. `idf.py menuconfig` → enable "Run the GBA SPI-slave self-test at boot", set SPI mode (start 3).
   (Or set `CONFIG_GBA_SPI_DIRECT_SELFTEST=y` in the build's sdkconfig.) Build + flash to the S3.
3. Put the GBA into wireless mode; watch the S3 console for `gba_spi` word logs (gate 1). Flip SPI
   mode 3↔0 until the login words match. Then build out gates 2–4.
