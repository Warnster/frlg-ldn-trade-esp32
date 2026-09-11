/* gba_wap — pure GBA Wireless-Adapter protocol logic (login + WAP framing + command routing),
 * ported from the RP2040 Rust (GBA-Global-Link adapter/src/comms/{login,packet,routes}.rs). NO
 * hardware dependencies, so it compiles + unit-tests on the host AND links into the S3 gba_spi
 * front-end. The transport (gba_spi.c on the S3, or a bit-bang elsewhere) feeds words in/out; this
 * module decides what the adapter says. See GBA_SPI_SLAVE_DESIGN.md §7. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Hot-path attribute: on the ESP32 the whole core-1 GBA protocol path must live in IRAM so a flash-cache
 * miss (e.g. core-0 logging / Wi-Fi) can't stall it mid-transaction and jitter the GBA link timing.
 * On the host unit-test build it's a no-op. */
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define GBA_HOT IRAM_ATTR
#else
#define GBA_HOT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---- login (adapter-detection) challenge/response, from login.rs ----
 * Bare 32-bit exchanges (no ready-handshake yet). Start by sending 0x00000000; feed each rx word
 * here and send the returned tx word next. *done=true once login completes (rx==0xB0BB8001). */
uint32_t gba_login_next(uint32_t rx, bool *done);

/* ---- WAP framing (packet.rs) ---- */
#define GBA_WAP_MAGIC 0x9966u
static inline uint32_t gba_wap_header(uint8_t cmd, uint8_t size) {
    return (GBA_WAP_MAGIC << 16) | ((uint32_t)size << 8) | cmd;
}
static inline bool    gba_wap_header_valid(uint32_t h) { return (h >> 16) == GBA_WAP_MAGIC; }
static inline uint8_t gba_wap_cmd(uint32_t h)          { return (uint8_t)h; }
static inline uint8_t gba_wap_size(uint32_t h)         { return (uint8_t)(h >> 8); }        /* data words */
static inline uint32_t gba_wap_response_header(uint32_t h) { return h | 0x80; }
static inline bool    gba_wap_is_response(uint8_t cmd) { return (cmd & 0x80) != 0; }

/* GBA command IDs (routes.rs) */
enum {
    GBA_CMD_UNKNOWN         = 0x11,
    GBA_CMD_GET_SOME_VALUE  = 0x13,
    GBA_CMD_BROADCAST       = 0x16,
    /* Setup's data[0] high-mid byte (bits 16-23) is the join/host ROLE byte: 0x3c=join, 0x3f=host
     * (same byte the union-room lobby uses — captured on real hardware, doc 15). Not dispatched
     * specially below (still a bare ack, matching real adapter behaviour) — `on_raw_command` in
     * `gba_wap_io` is how a backend observes it for room-selection purposes. */
    GBA_CMD_SETUP           = 0x17,
    GBA_CMD_BROADCAST_POLL  = 0x1d,
    GBA_CMD_BROADCAST_END   = 0x1e,
    GBA_CMD_CONNECT         = 0x1f,
    GBA_CMD_IS_CONNECTING   = 0x20,
    GBA_CMD_FINISH_CONNECT  = 0x21,
    GBA_CMD_SEND_DATA       = 0x24,
    GBA_CMD_SEND_DATA_WAIT  = 0x25,
    GBA_CMD_RECV_DATA       = 0x26,
    GBA_CMD_RECV_DATA_WAIT  = 0x27,
    GBA_CMD_RECV_DATA_WAIT_RESP = 0x28,
    /* Real Nintendo agbrfu low-level REQ IDs (librfu.h), the "_AND_CHANGE" family: an ack to any of
     * these triggers an SIO clock master/slave role swap, which our transport fakes via ASYNC_ACK
     * (gpi.rs/xfer_word's send-0xa8-then-one-more-idle-word pattern) rather than a plain immediate
     * reply. 0x25/0x27 were already handled; 0x35 (ID_UNK35_REQ, grouped with 0x25/0x27/0x37 in
     * librfu_stwi.c/librfu_intr.c) showed up on real hardware only after sustained Union Room time
     * (~590 commands in) and was previously falling through to the generic immediate-ack default,
     * which almost certainly caused the "Communication errors" disconnect. 0x36/0x37 are siblings in
     * the same family (CPR poll/resume-retransmit) — added defensively in case they appear too.
     */
    GBA_CMD_UNK35_AND_CHANGE = 0x35,
    GBA_CMD_UNK36_AND_CHANGE = 0x36,
    GBA_CMD_RESUME_RETRANSMIT_AND_CHANGE = 0x37,
};

/* Providers wire the adapter logic to the LDN/Switch side (the peer advert + trade slots). */
typedef struct gba_wap_io {
    int      (*get_peer)(uint32_t out[7], void *ctx);          /* 7 words [peer_id,beacon x6] or 0 */
    uint32_t (*peer_id)(void *ctx);                            /* nonzero peer_id, or 0 if none */
    int      (*take_slot)(uint32_t *out, int max, void *ctx);  /* latest Switch slot, word count */
    void     (*put_slot)(const uint32_t *data, int len, void *ctx); /* the GBA's outgoing slot */
    /* Optional: called for EVERY command before dispatch (mirrors routes.rs::handle_req's
     * "mirror every outgoing command" philosophy). NULL = no-op. Lets a backend observe raw
     * command/data content — e.g. the Setup role byte / Broadcast activity byte that distinguish
     * the trade counter from the union-room lobby — without gba_wap.c needing to know why. */
    void     (*on_raw_command)(uint8_t cmd, const uint32_t *data, uint8_t len, void *ctx);
    void *ctx;
    /* internal router state — zero-initialise */
    bool    some_value_high;
    uint8_t connecting_polls;
    bool    connected;
} gba_wap_io;

/* Result flags from gba_wap_respond so the transport knows how to finish the command. */
typedef enum {
    GBA_WAP_REPLY = 0,     /* send `out[0..n]` back as a normal reply (response header) */
    GBA_WAP_ASYNC_ACK,     /* SendDataAndWait/ReceiveDataAndWait: transport must fake async_ack */
} gba_wap_action;

/* Given a received command header + its data words, produce the reply. Returns the reply word count
 * (into out, capacity >=8) and sets *action. Mirrors routes.rs handle_req/local_respond. */
int gba_wap_respond(gba_wap_io *io, uint8_t command, const uint32_t *data, int len,
                    uint32_t *out, gba_wap_action *action);

#ifdef __cplusplus
}
#endif
