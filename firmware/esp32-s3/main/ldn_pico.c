/* ldn_pico.c — UART link to the Pico (GBA adapter). See ldn_pico.h. */
#include "sdkconfig.h"
#if CONFIG_LDN_PROBE_CONTROL_PORT
#include <string.h>
#include "driver/uart.h"
#include "ldn_pico.h"

/* Wiring — adjust to the board. UART1 (the console is USB Serial/JTAG, so UART0/1 pins are free). */
#define PICO_UART      UART_NUM_1
#define PICO_UART_TX   7     /* ESP TX -> Pico GP1 (RX). Moved off GPIO21 (S3-Zero WS2812 LED pin,
                                which loaded the line and broke TX while RX on 47 still worked). */
#define PICO_UART_RX   47    /* ESP RX <- Pico GP0 (TX) */
#define PICO_BAUD      115200

#define WAP_MAGIC      0x9966
#define CMD_SEND_DATA  0x25
#define CMD_BROADCAST  0x16
#define CMD_CONNECT    0x1f
#define T_NO_PEER      0x00
#define T_PEER         0x01
#define T_RECV_SLOT    0x02
#define T_BOOTSEL      0x7f   /* reboots the Pico to USB bootloader — used as a decisive TX-wire test */

/* GBA outgoing-slot ring (7 u16 each) */
#define SLOTQ 8
static uint16_t s_slotq[SLOTQ][7];
static int s_slot_head, s_slot_n;
static uint32_t s_wap_seen, s_slots_seen, s_slots_taken, s_recv_sent, s_peer_sent;   /* diagnostics */
/* Count of Connect (0x1f) commands the GBA issued (forwarded by the Pico, routes.rs). Each increment
 * means the player SELECTED our advertised peer — the trigger to actually join the Switch. */
static uint32_t s_connect_seen;
static uint32_t s_cmd_hist[256];   /* count of each RFU command byte the GBA has issued (diagnostic) */
static uint32_t s_pico_diag[3];    /* {rx_bytes, peer_commits, peer_present} reported by the Pico */

/* Real captured JP-FireRed Union-Room beacon + peer id — the GBA discovered+connected to this in
 * Phase 3. Advertised to the Pico so the cartridge lists it as a joinable group. */
static const uint32_t PEER_ID = 0x00002abc;
/* Captured JP-FireRed beacon, but with the RFU gname ACTIVITY byte (gname[10], the low byte of word3)
 * changed 0x40 -> 0x04: 0x40 = IN_UNION_ROOM|ACTIVITY_NONE (filtered out on the Trade counter);
 * 0x04 = ACTIVITY_TRADE, which is what LINK_GROUP_TRADE's sAcceptedActivityIds requires (pokefirered
 * union_room.c). The checksum byte (0xE3, word3 high byte) is unchanged — it only covers gname[0..7]
 * + uname[0..7], NOT gname[10] (librfu_rfu.c rfu_REQ_configGameData), so it stays valid. */
static const uint32_t PEER_BEACON[6] = {
    0x0f820002, 0x0000c979, 0x00000000, 0xe3000004, 0xffc2cdbb, 0x00000000 };

void ldn_pico_stats(uint32_t out[5]) {
    out[0]=s_wap_seen; out[1]=s_slots_seen; out[2]=s_slots_taken; out[3]=s_recv_sent; out[4]=s_peer_sent;
}

/* Host Switch's real in-game identity (from the LDN advert); when valid, replaces the placeholder. */
static uint16_t s_peer_tid; static uint8_t s_peer_name[8]; static bool s_peer_id_valid;
void ldn_pico_set_peer_identity(uint16_t tid, const uint8_t name8[8])
{
    s_peer_tid = tid; memcpy(s_peer_name, name8, 8); s_peer_id_valid = true;
}
/* Two-chip path: the GBA's 0x16 identity isn't decoded from the UART stream yet — fall back to the
 * default join name. (Single-chip ldn_gba.c decodes it via gba_relay.) */
bool ldn_pico_get_gba_identity(uint16_t *tid, uint8_t name8[8]) { (void)tid; (void)name8; return false; }
void ldn_pico_gba_bcast_dbg(uint32_t out6[6], uint32_t *seen) { for (int i=0;i<6;i++) out6[i]=0; if (seen) *seen=0; }
/* Two-chip path: the 0x16 RfuGameData isn't decoded from the UART stream — fall back to the
 * built-in default activity. (Single-chip ldn_gba.c decodes it via gba_relay.) */
bool ldn_pico_get_gba_activity(uint8_t *a, uint8_t *s, uint16_t *t) { (void)a; (void)s; (void)t; return false; }
/* Same GBA beacon build + checksum as the single-chip path (see ldn_gba.c build_gba_beacon). */
static void build_gba_beacon(uint16_t tid, const uint8_t name8[8], uint32_t beacon6[6])
{
    uint8_t b[24] = {0};
    b[0] = 0x02; b[1] = 0x00; b[2] = 0x82; b[3] = 0x0f;
    b[4] = tid & 0xFF; b[5] = (tid >> 8) & 0xFF;
    b[12] = 0x04;
    for (int i = 0; i < 8; i++) b[16 + i] = name8[i];
    uint8_t cs = 0;
    for (int i = 2; i < 10; i++) cs += b[i];
    for (int i = 16; i < 24; i++) cs += b[i];
    b[15] = (uint8_t)~cs;
    for (int k = 0; k < 6; k++)
        beacon6[k] = (uint32_t)b[4*k] | ((uint32_t)b[4*k+1] << 8) |
                     ((uint32_t)b[4*k+2] << 16) | ((uint32_t)b[4*k+3] << 24);
}
void ldn_pico_advertise_peer(void)
{
    uint32_t beacon6[6];
    if (s_peer_id_valid) build_gba_beacon(s_peer_tid, s_peer_name, beacon6);
    else memcpy(beacon6, PEER_BEACON, sizeof(beacon6));
    ldn_pico_send_peer(PEER_ID, beacon6); s_peer_sent++;
}

uint32_t ldn_pico_connect_count(void) { return s_connect_seen; }
uint32_t ldn_pico_cmd_count(uint8_t cmd) { return s_cmd_hist[cmd]; }
void ldn_pico_diag(uint32_t out[3]) { out[0]=s_pico_diag[0]; out[1]=s_pico_diag[1]; out[2]=s_pico_diag[2]; }

/* WAP byte accumulator */
#define ACC 512
static uint8_t s_acc[ACC];
static int s_accn;

void ldn_pico_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = PICO_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(PICO_UART, 2048, 2048, 0, NULL, 0);
    uart_param_config(PICO_UART, &cfg);
    uart_set_pin(PICO_UART, PICO_UART_TX, PICO_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void slot_push(const uint16_t words[7])
{
    int idx = (s_slot_head + s_slot_n) % SLOTQ;
    if (s_slot_n >= SLOTQ) {                /* full: drop oldest */
        s_slot_head = (s_slot_head + 1) % SLOTQ; s_slot_n--;
        idx = (s_slot_head + s_slot_n) % SLOTQ;
    }
    memcpy(s_slotq[idx], words, 7 * sizeof(uint16_t));
    s_slot_n++;
}

/* Parse complete WAP packets out of s_acc, byte-resyncing on the 0x9966 magic. */
static void parse_wap(void)
{
    int off = 0;
    while (s_accn - off >= 4) {
        uint32_t hdr = le32(s_acc + off);
        if (((hdr >> 16) & 0xFFFF) != WAP_MAGIC) { off++; continue; }   /* slide to resync */
        int cmd = hdr & 0xFF;
        int size = (hdr >> 8) & 0xFF;                                    /* data words */
        if (size > 32) { off++; continue; }                             /* implausible -> desync */
        int need = 4 + size * 4;
        if (s_accn - off < need) break;                                 /* wait for the rest */
        s_wap_seen++;
        s_cmd_hist[cmd & 0xFF]++;   /* which RFU commands is the GBA actually issuing? */
        if (cmd == CMD_SEND_DATA) {
            s_slots_seen++;
            uint8_t raw[16] = {0};
            int nb = size * 4; if (nb > 14) nb = 14;
            memcpy(raw, s_acc + off + 4, nb);                           /* first 14 bytes = slot */
            uint16_t words[7];
            for (int i = 0; i < 7; i++) words[i] = (uint16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
            slot_push(words);
        } else if (cmd == CMD_CONNECT) {
            s_connect_seen++;   /* GBA selected our peer -> trigger the real Switch join */
        } else if (cmd == 0xFE && size == 3) {   /* Pico->ESP relay diagnostic report */
            s_pico_diag[0] = le32(s_acc + off + 4);   /* rx bytes core0 pulled from the ESP */
            s_pico_diag[1] = le32(s_acc + off + 8);   /* T_PEER frames the relay committed */
            s_pico_diag[2] = le32(s_acc + off + 12);  /* peer_present (0/1) */
        }
        /* CMD_BROADCAST (0x16) game-data is ignored: the brain synthesizes its own NI. */
        off += need;
    }
    if (off > 0) {
        s_accn -= off;
        memmove(s_acc, s_acc + off, s_accn);
    }
}

void ldn_pico_poll(void)
{
    uint8_t tmp[256];
    for (;;) {
        int n = uart_read_bytes(PICO_UART, tmp, sizeof(tmp), 0);
        if (n <= 0) break;
        int space = ACC - s_accn;
        if (n > space) {                    /* overflow: keep the newest bytes */
            int drop = n - space;
            if (drop >= s_accn) s_accn = 0;
            else { s_accn -= drop; memmove(s_acc, s_acc + drop, s_accn); }
        }
        int take = n; if (take > ACC - s_accn) take = ACC - s_accn;
        memcpy(s_acc + s_accn, tmp, take); s_accn += take;
        parse_wap();
        if (n < (int)sizeof(tmp)) break;
    }
}

int ldn_pico_take_gba_slot(uint16_t words[7])
{
    if (s_slot_n <= 0) return 0;
    memcpy(words, s_slotq[s_slot_head], 7 * sizeof(uint16_t));
    s_slot_head = (s_slot_head + 1) % SLOTQ; s_slot_n--;
    s_slots_taken++;
    return 1;
}

static void aa55(uint8_t type, const uint8_t *payload, int len)
{
    uint8_t frame[64];
    if (len > 58) return;
    frame[0] = 0xAA; frame[1] = 0x55; frame[2] = type; frame[3] = (uint8_t)len;
    uint8_t csum = type ^ (uint8_t)len;
    for (int i = 0; i < len; i++) { frame[4 + i] = payload[i]; csum ^= payload[i]; }
    frame[4 + len] = csum;
    uart_write_bytes(PICO_UART, (const char *)frame, 5 + len);
}

void ldn_pico_send_slot(const uint8_t slot14[14])
{
    uint8_t payload[16] = {0};
    memcpy(payload, slot14, 14);           /* pad to 16 (4 u32 words), matching pico_link */
    aa55(T_RECV_SLOT, payload, 16);
    s_recv_sent++;
}

void ldn_pico_send_peer(uint32_t peer_id, const uint32_t beacon6[6])
{
    uint8_t payload[28];
    payload[0] = peer_id; payload[1] = peer_id >> 8; payload[2] = peer_id >> 16; payload[3] = peer_id >> 24;
    for (int k = 0; k < 6; k++) {
        uint32_t w = beacon6[k];
        payload[4 + k * 4] = w; payload[5 + k * 4] = w >> 8;
        payload[6 + k * 4] = w >> 16; payload[7 + k * 4] = w >> 24;
    }
    aa55(T_PEER, payload, 28);
}

void ldn_pico_send_no_peer(void) { aa55(T_NO_PEER, NULL, 0); }

/* DIAGNOSTIC: reboot the Pico into USB bootloader via the relay's T_BOOTSEL. If this lands, the
 * Pico enumerates as an RPI-RP2 drive — decisive proof the ESP->Pico UART TX wire works. */
void ldn_pico_tx_bootsel_test(void) { aa55(T_BOOTSEL, NULL, 0); }
#endif
