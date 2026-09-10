/* gba_wap — pure GBA Wireless-Adapter protocol logic (login + WAP framing + command routing),
 * ported from the RP2040 Rust (GBA-Global-Link adapter/src/comms/{login,packet,routes}.rs). NO
 * hardware dependencies, so it compiles + unit-tests on the host AND links into the S3 gba_spi
 * front-end. The transport (gba_spi.c on the S3, or a bit-bang elsewhere) feeds words in/out; this
 * module decides what the adapter says. See GBA_SPI_SLAVE_DESIGN.md §7. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

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
};

/* Providers wire the adapter logic to the LDN/Switch side (the peer advert + trade slots). */
typedef struct gba_wap_io {
    int      (*get_peer)(uint32_t out[7], void *ctx);          /* 7 words [peer_id,beacon x6] or 0 */
    uint32_t (*peer_id)(void *ctx);                            /* nonzero peer_id, or 0 if none */
    int      (*take_slot)(uint32_t *out, int max, void *ctx);  /* latest Switch slot, word count */
    void     (*put_slot)(const uint32_t *data, int len, void *ctx); /* the GBA's outgoing slot */
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
