/* Host unit test for gba_wap.c (pure logic). Build+run:
 *   gcc -I.. -std=c11 -Wall -Werror ../gba_wap.c test_gba_wap.c -o /tmp/test_gba_wap && /tmp/test_gba_wap
 * No hardware needed — validates the login table, WAP framing, and command routing. */
#include "gba_wap.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

/* ---- mock LDN/Switch side ---- */
static uint32_t mock_beacon[6] = {0x0f820002, 0x0000c979, 0, 0xe3000040, 0xffc2cdbb, 0};
static const uint32_t MOCK_PEER = 0x00002abc;
static int mock_have_peer = 1;
static uint32_t last_put_slot[8]; static int last_put_len = -1;

static int m_get_peer(uint32_t out[7], void *ctx) {
    (void)ctx; if (!mock_have_peer) return 0;
    out[0] = MOCK_PEER; for (int i = 0; i < 6; i++) out[1+i] = mock_beacon[i]; return 7;
}
static uint32_t m_peer_id(void *ctx) { (void)ctx; return mock_have_peer ? MOCK_PEER : 0; }
static int m_take_slot(uint32_t *out, int max, void *ctx) {
    (void)ctx; (void)max; out[0] = 0xdeadbeef; return 1; }
static void m_put_slot(const uint32_t *d, int len, void *ctx) {
    (void)ctx; last_put_len = len; for (int i = 0; i < len && i < 8; i++) last_put_slot[i] = d[i]; }

int main(void)
{
    /* ---- 1. login walk (login.rs chain) ---- */
    struct { uint32_t rx, tx; int done; } chain[] = {
        {0x0000494E, 0x494EB6B1, 0}, {0xB6B1494E, 0x544EB6B1, 0}, {0xB6B1544E, 0x544EABB1, 0},
        {0xABB1544E, 0x4E45ABB1, 0}, {0xABB14E45, 0x4E45B1BA, 0}, {0xB1BA4E45, 0x4F44B1BA, 0},
        {0xB1BA4F44, 0x4F44B0BB, 0}, {0xB0BB4F44, 0x8001B0BB, 0}, {0xB0BB8001, 0x8001B0BB, 1},
    };
    for (unsigned i = 0; i < sizeof(chain)/sizeof(chain[0]); i++) {
        bool done = false;
        uint32_t tx = gba_login_next(chain[i].rx, &done);
        CHECK(tx == chain[i].tx, "login tx mismatch");
        CHECK(done == (bool)chain[i].done, "login done flag mismatch");
    }
    /* resync default for an unexpected rx: 0x494e0000 | (~(rx>>16)&0xffff) */
    { bool d; uint32_t tx = gba_login_next(0x12340000, &d);
      CHECK(tx == (0x494E0000u | (~0x1234u & 0xFFFFu)), "login resync default"); }

    /* ---- 2. WAP framing round-trip ---- */
    uint32_t h = gba_wap_header(0x1d, 0);
    CHECK(gba_wap_header_valid(h), "header magic");
    CHECK(gba_wap_cmd(h) == 0x1d, "header cmd");
    CHECK(gba_wap_size(h) == 0, "header size");
    CHECK(gba_wap_response_header(h) == (h | 0x80), "response header bit");
    { uint32_t h2 = gba_wap_header(0x25, 4);
      CHECK(gba_wap_cmd(h2) == 0x25 && gba_wap_size(h2) == 4, "header cmd/size 2"); }

    /* ---- 3. routing ---- */
    gba_wap_io io; memset(&io, 0, sizeof(io));
    io.get_peer = m_get_peer; io.peer_id = m_peer_id; io.take_slot = m_take_slot; io.put_slot = m_put_slot;
    uint32_t out[8]; gba_wap_action act; int n;

    n = gba_wap_respond(&io, GBA_CMD_GET_SOME_VALUE, NULL, 0, out, &act);
    CHECK(n == 1 && out[0] == 0x0200abcd, "GetSomeValue high");
    n = gba_wap_respond(&io, GBA_CMD_GET_SOME_VALUE, NULL, 0, out, &act);
    CHECK(n == 1 && out[0] == 0x00000000, "GetSomeValue low");

    n = gba_wap_respond(&io, GBA_CMD_UNKNOWN, NULL, 0, out, &act);
    CHECK(n == 1 && out[0] == 0x000000ff, "Unknown 0x11");

    n = gba_wap_respond(&io, GBA_CMD_BROADCAST_POLL, NULL, 0, out, &act);
    CHECK(n == 7 && out[0] == MOCK_PEER && out[1] == mock_beacon[0], "BroadcastPoll lists peer");

    /* connect flow: Connect -> IsConnecting x3 = connecting, then peer_id */
    n = gba_wap_respond(&io, GBA_CMD_CONNECT, NULL, 0, out, &act);
    CHECK(n == 0, "Connect returns nothing");
    for (int i = 0; i < 3; i++) {
        n = gba_wap_respond(&io, GBA_CMD_IS_CONNECTING, NULL, 0, out, &act);
        CHECK(n == 1 && out[0] == 0x01000000, "IsConnecting -> connecting");
    }
    n = gba_wap_respond(&io, GBA_CMD_IS_CONNECTING, NULL, 0, out, &act);
    CHECK(n == 1 && out[0] == MOCK_PEER, "IsConnecting -> connected(peer_id)");

    /* SendDataAndWait: put_slot called + async_ack action */
    uint32_t slot[4] = {0x11111111, 0x22222222, 0x33333333, 0x44444444};
    n = gba_wap_respond(&io, GBA_CMD_SEND_DATA_WAIT, slot, 4, out, &act);
    CHECK(n == 0 && act == GBA_WAP_ASYNC_ACK, "SendDataWait -> async_ack");
    CHECK(last_put_len == 4 && last_put_slot[0] == 0x11111111, "SendDataWait forwarded slot");

    /* ReceiveData -> latest Switch slot */
    n = gba_wap_respond(&io, GBA_CMD_RECV_DATA, NULL, 0, out, &act);
    CHECK(n == 1 && out[0] == 0xdeadbeef, "RecvData -> slot");

    /* no-peer case: BroadcastPoll empty */
    mock_have_peer = 0;
    n = gba_wap_respond(&io, GBA_CMD_BROADCAST_POLL, NULL, 0, out, &act);
    CHECK(n == 0, "BroadcastPoll empty when no peer");

    if (failures == 0) { printf("ALL GBA-WAP TESTS PASSED\n"); return 0; }
    printf("%d FAILURE(S)\n", failures); return 1;
}
