/* gba_relay.c — see gba_relay.h for the full design rationale. */
#include "gba_relay.h"
#include <string.h>
#include <stdatomic.h>
#include "esp_attr.h"

typedef struct {
    /* core0 -> core1: Switch peer presence + beacon */
    _Atomic uint32_t peer_present;
    uint32_t peer_id;                  /* written by core0 BEFORE the release store to peer_present */
    uint32_t beacon[6];

    /* core1 -> core0: the GBA's outgoing slot (SendDataAndWait / put_slot) */
    uint32_t gba_slot[GBA_RELAY_SLOT_WORDS];
    _Atomic uint32_t gba_slot_len;
    _Atomic uint32_t gba_slot_seq;
    uint32_t gba_slot_seq_seen;        /* core0-owned only — no cross-core race on this field */

    /* core0 -> core1: the latest Switch-received slot (ReceiveData / take_slot) */
    uint32_t switch_slot[GBA_RELAY_SLOT_WORDS];
    _Atomic uint32_t switch_slot_len;
    _Atomic uint32_t switch_slot_seq;
    uint32_t switch_slot_seq_seen;     /* core1-owned only */

    /* diagnostics: decoded room-select bytes, captured passively via on_raw_command */
    _Atomic uint8_t  setup_role;
    uint32_t         setup_raw0;       /* plain — protected by setup_seen's release/acquire pair */
    _Atomic uint32_t setup_seen;
    _Atomic uint8_t  broadcast_activity;
    uint32_t         broadcast_raw[6]; /* plain — protected by broadcast_seen's release/acquire pair */
    _Atomic uint32_t broadcast_seen;

    /* activity counters for the core0 LDN join-driving gate (2026-09-11, docs/16 Phase 2). The
     * two-chip build derived these from the raw WAP UART stream in ldn_pico.c; on the single chip
     * the equivalent signals are observed here via on_raw_command. All monotonic, plain atomics —
     * core1 bumps, core0 reads. `wap_seen` = any command at all (the "GBA is alive in wireless
     * mode" signal the gate uses); `connect_seen` = Connect(0x1f) count (the player SELECTED our
     * advertised peer = the trigger to actually join the Switch); `send_seen` = SendDataWait(0x25)
     * count (trade slots flowing). */
    _Atomic uint32_t wap_seen;
    _Atomic uint32_t connect_seen;
    _Atomic uint32_t send_seen;

    /* The GBA cartridge's own in-game identity, decoded from its Broadcast(0x16) RfuGameData
     * (2026-09-11). The 0x16 payload words map 1:1 to the beacon6 words: data[1] low16 = TID,
     * data[4]/data[5] = uname (8-byte trainer name, FRLG charmap). Presented to the Switch as the
     * joining player instead of the hardcoded "EMU". */
    uint16_t         gba_id_tid;
    uint8_t          gba_id_name[8];
    _Atomic uint32_t gba_id_seen;
    _Atomic uint32_t cmd_hist[256];   /* per-command-byte count — full visibility into what flow the
                                       * GBA is in (host 0x19/0x1a vs browse 0x1c/1d/1e vs trade 0x25). */
} gba_relay_state_t;

/* One static instance — the relay is a singleton (matches gba_spi's own single-adapter design;
 * there's only ever one GBA link and one Switch peer at a time). Zero-initialised at load time;
 * gba_relay_init() re-zeroes explicitly for a clean re-arm across selftest re-runs. */
static gba_relay_state_t s_relay;

void gba_relay_init(void)
{
    memset(&s_relay, 0, sizeof(s_relay));
}

/* ---- core1-side callbacks (IRAM, non-blocking, called only from gba_spi_core1_entry) ---- */

static IRAM_ATTR int gba_relay_get_peer_cb(uint32_t out[7], void *ctx)
{
    (void)ctx;
    if (!atomic_load_explicit(&s_relay.peer_present, memory_order_acquire)) return 0;
    out[0] = s_relay.peer_id;
    for (int i = 0; i < 6; i++) out[1 + i] = s_relay.beacon[i];
    return 7;
}

static IRAM_ATTR uint32_t gba_relay_peer_id_cb(void *ctx)
{
    (void)ctx;
    if (!atomic_load_explicit(&s_relay.peer_present, memory_order_acquire)) return 0;
    return s_relay.peer_id;
}

static IRAM_ATTR int gba_relay_take_slot_cb(uint32_t *out, int max, void *ctx)
{
    (void)ctx;
    uint32_t seq = atomic_load_explicit(&s_relay.switch_slot_seq, memory_order_acquire);
    if (seq == s_relay.switch_slot_seq_seen) return 0;   /* nothing new — cheap, never blocks */
    s_relay.switch_slot_seq_seen = seq;
    int len = (int)atomic_load_explicit(&s_relay.switch_slot_len, memory_order_relaxed);
    if (len > max) len = max;
    for (int i = 0; i < len; i++) out[i] = s_relay.switch_slot[i];
    return len;
}

static IRAM_ATTR void gba_relay_put_slot_cb(const uint32_t *data, int len, void *ctx)
{
    (void)ctx;
    if (len > GBA_RELAY_SLOT_WORDS) len = GBA_RELAY_SLOT_WORDS;
    if (len < 0) len = 0;
    for (int i = 0; i < len; i++) s_relay.gba_slot[i] = data[i];
    atomic_store_explicit(&s_relay.gba_slot_len, (uint32_t)len, memory_order_relaxed);
    atomic_fetch_add_explicit(&s_relay.gba_slot_seq, 1, memory_order_release);
}

static IRAM_ATTR void gba_relay_on_raw_command_cb(uint8_t cmd, const uint32_t *data, uint8_t len, void *ctx)
{
    (void)ctx;
    /* Activity signals for the core0 join gate — bumped for EVERY command, before the per-command
     * decode below. Relaxed order is fine: these are monotonic counters core0 only samples for
     * edge-detection ("did it change?"), never for ordering against payload data. */
    atomic_fetch_add_explicit(&s_relay.wap_seen, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&s_relay.cmd_hist[cmd], 1, memory_order_relaxed);
    if (cmd == GBA_CMD_CONNECT)   atomic_fetch_add_explicit(&s_relay.connect_seen, 1, memory_order_relaxed);
    if (cmd == GBA_CMD_SEND_DATA_WAIT) atomic_fetch_add_explicit(&s_relay.send_seen, 1, memory_order_relaxed);
    if (cmd == GBA_CMD_SETUP && len >= 1) {
        uint8_t role = (uint8_t)(data[0] >> 16);
        s_relay.setup_raw0 = data[0];
        atomic_store_explicit(&s_relay.setup_role, role, memory_order_relaxed);
        atomic_fetch_add_explicit(&s_relay.setup_seen, 1, memory_order_release);
    } else if (cmd == GBA_CMD_BROADCAST && len >= 4) {
        uint8_t act = (uint8_t)(data[3] & 0xFFu);
        int n = len < 6 ? len : 6;
        for (int i = 0; i < n; i++) s_relay.broadcast_raw[i] = data[i];
        for (int i = n; i < 6; i++) s_relay.broadcast_raw[i] = 0;
        atomic_store_explicit(&s_relay.broadcast_activity, act, memory_order_relaxed);
        atomic_fetch_add_explicit(&s_relay.broadcast_seen, 1, memory_order_release);
        /* Capture the GBA's own in-game identity (TID + trainer name) from the same RfuGameData. */
        if (len >= 6) {
            s_relay.gba_id_tid = (uint16_t)(data[1] & 0xFFFFu);
            s_relay.gba_id_name[0] = (uint8_t)(data[4]);       s_relay.gba_id_name[1] = (uint8_t)(data[4] >> 8);
            s_relay.gba_id_name[2] = (uint8_t)(data[4] >> 16); s_relay.gba_id_name[3] = (uint8_t)(data[4] >> 24);
            s_relay.gba_id_name[4] = (uint8_t)(data[5]);       s_relay.gba_id_name[5] = (uint8_t)(data[5] >> 8);
            s_relay.gba_id_name[6] = (uint8_t)(data[5] >> 16); s_relay.gba_id_name[7] = (uint8_t)(data[5] >> 24);
            atomic_fetch_add_explicit(&s_relay.gba_id_seen, 1, memory_order_release);
        }
    }
}

void gba_relay_fill_io(gba_wap_io *io)
{
    memset(io, 0, sizeof(*io));
    io->get_peer       = gba_relay_get_peer_cb;
    io->peer_id        = gba_relay_peer_id_cb;
    io->take_slot      = gba_relay_take_slot_cb;
    io->put_slot       = gba_relay_put_slot_cb;
    io->on_raw_command = gba_relay_on_raw_command_cb;
    io->ctx            = NULL;
}

/* ---- core0-side API (plain FreeRTOS-task code, no IRAM/blocking constraints) ---- */

void gba_relay_set_peer(uint32_t peer_id, const uint32_t beacon[6])
{
    s_relay.peer_id = peer_id;
    for (int i = 0; i < 6; i++) s_relay.beacon[i] = beacon[i];
    atomic_store_explicit(&s_relay.peer_present, 1, memory_order_release);
}

void gba_relay_clear_peer(void)
{
    atomic_store_explicit(&s_relay.peer_present, 0, memory_order_release);
}

int gba_relay_take_gba_slot(uint32_t *out, int max)
{
    uint32_t seq = atomic_load_explicit(&s_relay.gba_slot_seq, memory_order_acquire);
    if (seq == s_relay.gba_slot_seq_seen) return 0;
    s_relay.gba_slot_seq_seen = seq;
    int len = (int)atomic_load_explicit(&s_relay.gba_slot_len, memory_order_relaxed);
    if (len > max) len = max;
    for (int i = 0; i < len; i++) out[i] = s_relay.gba_slot[i];
    return len;
}

void gba_relay_put_switch_slot(const uint32_t *data, int len)
{
    if (len > GBA_RELAY_SLOT_WORDS) len = GBA_RELAY_SLOT_WORDS;
    if (len < 0) len = 0;
    for (int i = 0; i < len; i++) s_relay.switch_slot[i] = data[i];
    atomic_store_explicit(&s_relay.switch_slot_len, (uint32_t)len, memory_order_relaxed);
    atomic_fetch_add_explicit(&s_relay.switch_slot_seq, 1, memory_order_release);
}

uint32_t gba_relay_wap_seen(void)     { return atomic_load_explicit(&s_relay.wap_seen,     memory_order_relaxed); }
uint32_t gba_relay_connect_count(void){ return atomic_load_explicit(&s_relay.connect_seen, memory_order_relaxed); }
uint32_t gba_relay_send_count(void)   { return atomic_load_explicit(&s_relay.send_seen,     memory_order_relaxed); }
uint32_t gba_relay_cmd_count(uint8_t cmd) { return atomic_load_explicit(&s_relay.cmd_hist[cmd], memory_order_relaxed); }

bool gba_relay_get_gba_identity(uint16_t *tid, uint8_t name8[8])
{
    if (!atomic_load_explicit(&s_relay.gba_id_seen, memory_order_acquire)) return false;
    if (tid) *tid = s_relay.gba_id_tid;
    if (name8) for (int i = 0; i < 8; i++) name8[i] = s_relay.gba_id_name[i];
    return true;
}

void gba_relay_get_room_info(gba_relay_room_info_t *out)
{
    out->setup_seen         = atomic_load_explicit(&s_relay.setup_seen, memory_order_acquire);
    out->setup_role         = atomic_load_explicit(&s_relay.setup_role, memory_order_relaxed);
    out->setup_raw0         = s_relay.setup_raw0;
    out->broadcast_seen     = atomic_load_explicit(&s_relay.broadcast_seen, memory_order_acquire);
    out->broadcast_activity = atomic_load_explicit(&s_relay.broadcast_activity, memory_order_relaxed);
    for (int i = 0; i < 6; i++) out->broadcast_raw[i] = s_relay.broadcast_raw[i];
}
