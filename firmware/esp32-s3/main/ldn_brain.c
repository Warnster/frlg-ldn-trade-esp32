/* ldn_brain.c — cport/sim.c (the FRLG<->LDN trade brain) driven by the ESP's own LDN I/O.
 * Inbound LDN datagrams are fed to sim_feed_datagram(); a ~60 Hz tick drives sim_tick(); sim's
 * send callback goes straight to ldn_udp_send(). The GBA-facing engine is a stub here (idle slots)
 * so the brain completes the Pia handshake + NI registration with the Switch on its own; Task 4
 * replaces the engine with the Pico-UART RFU relay of the real cartridge. */
#include "sdkconfig.h"
#if CONFIG_LDN_PROBE_CONTROL_PORT
#include <string.h>
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "pia_zstd.h"
#include "ldn_brain.h"
#include "ldn_control.h"
#include "ldn_pico.h"
#include "ldn_udp.h"
#include "ldn_wire.h"
#include "sim.h"
#include "ni.h"   /* NI_LANGUAGE_JAPANESE for the cartridge identity below */
#include "crypto.h"
#define printf ldn_wire_printf

/* Real captured JP-FireRed Union-Room beacon + peer id (host/pico_host.py DEFAULT_BEACON): the GBA
 * discovered+connected to this without crashing (Phase 3). Advertised so the cartridge joins. */
#define BRAIN_OUR_MPID   1
static const uint32_t BRAIN_PEER_ID = 0x00002abc;
static const uint32_t BRAIN_BEACON[6] = {   /* activity byte 0x40->0x04 = ACTIVITY_TRADE (see ldn_pico.c) */
    0x0f820002, 0x0000c979, 0x00000000, 0xe3000004, 0xffc2cdbb, 0x00000000 };
static int s_peer_tick;

static sim_t      s_sim;
static pc_conn_t  s_conn;
static sim_engine s_engine;
static pia_crypto s_pc;
static bool       s_active;

/* progress diagnostics (printed on change over the ESP's own USB, never the GBA) */
static int s_seen_accepted, s_seen_ni_done, s_seen_uni, s_seen_conn;

/* ---- engine: the real GBA over the Pico UART ---- */
static void eng_tick(void *ctx, uint16_t words[7]) {
    (void)ctx;
    if (ldn_pico_take_gba_slot(words)) return;  /* the cartridge's next outgoing slot */
    for (int i = 0; i < 7; i++) words[i] = 0;   /* idle when the GBA hasn't produced one */
}
static void eng_feed(void *ctx, const gba_in *rec) {
    (void)ctx;
    /* DIAGNOSTIC: log the host's NI sub-frames (what the Switch sends us at the RFU/NI layer). */
    static int nidbg;
    if (rec->has_ni && nidbg < 40) { nidbg++;
        printf("HOSTNI st=%d ack=%d n=%d ph=%d sz=%d p0=%02x p1=%02x\n",
               rec->llsf_state, rec->ni_ack, rec->ni_n, rec->ni_phase, rec->ni_size,
               rec->ni_payload_len > 0 ? rec->ni_payload[0] : 0,
               rec->ni_payload_len > 1 ? rec->ni_payload[1] : 0);
    }
    /* Relay each non-self participant slot (the Switch host is mpId 0) down to the GBA. */
    for (int i = 0; i < rec->nslots; i++) {
        if (i == BRAIN_OUR_MPID) continue;
        ldn_pico_send_slot(rec->slots[i]);
    }
}

/* ---- transport send: sim -> LDN UDP ---- */
static void brain_send(void *ctx, const uint8_t *dg, size_t len, const uint8_t dst_ip[4]) {
    (void)ctx;
    ldn_udp_send(dst_ip, dg, (int)len);
}

bool ldn_brain_active(void) { return s_active; }

int ldn_brain_start(const uint8_t ssid[16], const uint8_t our_ip[4], const uint8_t host_ip[4],
                    const uint8_t our_mac[6], const uint8_t host_mac[6], const char *joiner_name) {
    pia_crypto_init(&s_pc, ssid, FRLG_GAME_KEY);

    const char *jn = (joiner_name && joiner_name[0]) ? joiner_name : "EMU";
    uint8_t random4[4];
    esp_fill_random(random4, sizeof(random4));
    pc_conn_init(&s_conn, our_mac, host_mac, our_ip, host_ip, PC_DEFAULT_OUR_VAR,
                 jn, random4, NULL);

    memset(&s_engine, 0, sizeof(s_engine));
    s_engine.tick = eng_tick;
    s_engine.feed_in_frame = eng_feed;
    /* Identity we present to the Switch as the joining player: the REAL cartridge's trainer id +
     * name, taken from its own Broadcast(0x16) as RAW FRLG-charmap bytes. Raw (not ASCII) because
     * this cart is Japanese — its name is kana (としあき = 14 0c 01 07), which the ASCII charmap
     * cannot encode. Falls back to the built-in default if the GBA hasn't broadcast yet. */
    {
        uint16_t gtid = 0; uint8_t gname8[8];
        if (ldn_pico_get_gba_identity(&gtid, gname8)) {
            s_engine.lp_trainer_id = gtid;
            memcpy(s_engine.lp_name_raw, gname8, 8);
            s_engine.lp_name_raw_valid = 1;
            s_engine.lp_language = NI_LANGUAGE_JAPANESE;
            printf("LDN_BRAIN identity tid=0x%04x name_raw=%02x%02x%02x%02x%02x%02x%02x%02x (JP, raw)\n",
                   gtid, gname8[0], gname8[1], gname8[2], gname8[3],
                   gname8[4], gname8[5], gname8[6], gname8[7]);
        }
        /* Advertise the activity the cartridge is ACTUALLY doing, rather than hardcoding trade.
         * FRLG's Direct-Corner entry path for a Colosseum battle is nearly identical to trade
         * (same warp/cable-club machinery, different destination map), and in this relay the two
         * real games supply all the game logic — so carrying the cart's own activity byte through
         * is what lets anything other than a trade be advertised correctly to the Switch.
         * Falls back to sim.c's NI_ACTIVITY_TRADE default when the GBA hasn't broadcast yet. */
        uint8_t gact = 0, gstarted = 0; uint16_t gtrade = 0;
        if (ldn_pico_get_gba_activity(&gact, &gstarted, &gtrade) && gact) {
            s_engine.ni_activity = gact;
            s_engine.ni_started  = gstarted;
            printf("LDN_BRAIN activity=0x%02x started=%u trade_word=0x%04x (%s)\n",
                   gact, gstarted, gtrade,
                   gact == 0x04 ? "TRADE" : gact == 0x01 ? "BATTLE_SINGLE" :
                   gact == 0x02 ? "BATTLE_DOUBLE" : gact == 0x05 ? "CHAT" : "other");
        }
    }
    s_engine.in_seat_phase = 0;
    s_engine.established = 0;
    /* Default only — the cartridge's own broadcast (above) overrides it when available. Must not
     * clobber that, so only set it if the activity plumbing didn't already decide. */
    if (!s_engine.ni_activity) s_engine.ni_started = 1;

    uint8_t connect_id[2];
    esp_fill_random(connect_id, sizeof(connect_id));
    if (!connect_id[0] && !connect_id[1]) connect_id[0] = 1;

    uint64_t nonce = 0;
    esp_fill_random(&nonce, sizeof(nonce));
    if (nonce == 0) nonce = 1;

    sim_init(&s_sim, &s_pc, &s_engine, brain_send, NULL, our_ip, host_ip, &s_conn,
             connect_id, 2, nonce);

    s_seen_accepted = s_seen_ni_done = s_seen_uni = s_seen_conn = 0;
    s_peer_tick = 0;
    s_active = true;
    ldn_pico_send_peer(BRAIN_PEER_ID, BRAIN_BEACON);   /* let the GBA discover + connect */
    printf("LDN_BRAIN_STARTED us=%u.%u.%u.%u host=%u.%u.%u.%u\n",
           our_ip[0], our_ip[1], our_ip[2], our_ip[3],
           host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
    return 0;
}

void ldn_brain_stop(void) {
    if (!s_active) return;
    s_active = false;
    printf("LDN_BRAIN_STOPPED rx=%d tx=%d\n", s_sim.rx_count, s_sim.tx_count);
}

void ldn_brain_on_datagram(const uint8_t src_ip[4], const uint8_t *data, int len) {
    if (!s_active) return;
    static int dbg;
    if (dbg < 2) { dbg++;
        static const char *H = "0123456789abcdef";
        char hx[256]; int n = len < 120 ? len : 120;
        for (int i = 0; i < n; i++) { hx[2*i] = H[data[i] >> 4]; hx[2*i+1] = H[data[i] & 15]; }
        hx[2*n] = 0;
        printf("LDN_BRAIN_DG len=%d dg=%s\n", len, hx);
    }
    sim_feed_datagram(&s_sim, data, len, src_ip);
}

void ldn_brain_tick(void) {
    if (!s_active) return;
    ldn_pico_poll();                       /* drain the GBA's outgoing WAP slots */
    /* Keep advertising the peer so the cartridge stays discoverable/connected (~0.5s cadence). */
    if (++s_peer_tick >= 30) { s_peer_tick = 0; ldn_pico_send_peer(BRAIN_PEER_ID, BRAIN_BEACON); }
    sim_tick(&s_sim);

    /* Surface milestones (once each) so progress is visible over the ESP USB. */
    if (!s_seen_conn && sim_connected(&s_sim)) { s_seen_conn = 1; printf("LDN_BRAIN connected\n"); }
    if (!s_seen_accepted && s_sim.gba_accepted) { s_seen_accepted = 1; printf("LDN_BRAIN host_accepted\n"); }
    if (!s_seen_ni_done && s_sim.ni_done) { s_seen_ni_done = 1; printf("LDN_BRAIN ni_done\n"); }
    if (!s_seen_uni && s_sim.host_uni_seen) { s_seen_uni = 1; printf("LDN_BRAIN host_uni (trade ready)\n"); }
    if (s_sim.ni_rejected) { printf("LDN_BRAIN ni_REJECTED\n"); ldn_brain_stop(); }
    if (s_sim.host_disconnected) { printf("LDN_BRAIN host_disconnect\n"); ldn_brain_stop(); }

    /* ~1s heartbeat so we can see whether the Switch's Pia datagrams are arriving. */
    static int hb;
    if (++hb % 60 == 0 || (s_sim.rx_count && s_sim.rx_count < 4)) {
        uint32_t air[4]; ldn_control_air(air);
        uint32_t p[5]; ldn_pico_stats(p);
        printf("LDN_BRAIN rx=%d tx=%d pia=%d acc=%d ni=%d uni=%d | GBA wap=%u slots=%u taken=%u recv=%u\n",
               s_sim.rx_count, s_sim.tx_count, s_conn.state,
               s_sim.gba_accepted, s_sim.ni_done, s_sim.host_uni_seen,
               (unsigned)p[0], (unsigned)p[1], (unsigned)p[2], (unsigned)p[3]);
        (void)air;
    }
}
#endif
