/* gba_wap.c — see gba_wap.h. Faithful C port of the RP2040 adapter logic. Pure logic, no hardware. */
#include "gba_wap.h"
#include <string.h>

/* login.rs challenge/response table (verbatim). */
GBA_HOT uint32_t gba_login_next(uint32_t rx, bool *done)
{
    if (done) *done = false;
    switch (rx) {
        case 0x0000494E: return 0x494EB6B1;
        case 0xFFFF494E: return 0x494EB6B1;
        case 0x7FFF494E: return 0x494EB6B1;
        case 0xB6B1494E: return 0x544EB6B1;
        case 0xB6B1544E: return 0x544EABB1;
        case 0xABB1544E: return 0x4E45ABB1;
        case 0xABB14E45: return 0x4E45B1BA;
        case 0xB1BA4E45: return 0x4F44B1BA;
        case 0xB1BA4F44: return 0x4F44B0BB;
        case 0xB0BB4F44: return 0x8001B0BB;
        case 0xB0BB8001: if (done) *done = true; return 0x8001B0BB;
        /* resync default (login.rs): 0x494e0000 | !(rx>>16) */
        default: return 0x494E0000u | (~(rx >> 16) & 0xFFFFu);
    }
}

/* routes.rs::local_respond + the handle_req SendData/ReceiveData special-casing. */
GBA_HOT int gba_wap_respond(gba_wap_io *io, uint8_t command, const uint32_t *data, int len,
                    uint32_t *out, gba_wap_action *action)
{
    *action = GBA_WAP_REPLY;
    if (io->on_raw_command) io->on_raw_command(command, data, (uint8_t)len, io->ctx);

    switch (command) {
        case GBA_CMD_GET_SOME_VALUE:                 /* 0x13: alternate 0x0200abcd / 0 */
            io->some_value_high = !io->some_value_high;
            out[0] = io->some_value_high ? 0x0200abcd : 0x00000000;
            return 1;

        case GBA_CMD_UNKNOWN:                         /* 0x11 -> 0x000000ff */
            out[0] = 0x000000ff;
            return 1;

        case GBA_CMD_BROADCAST_POLL:                  /* 0x1d / 0x1e -> list the relayed peer */
        case GBA_CMD_BROADCAST_END:
            return io->get_peer ? io->get_peer(out, io->ctx) : 0;

        case GBA_CMD_CONNECT:                         /* 0x1f -> begin connecting */
            io->connecting_polls = 0;
            io->connected = false;
            return 0;

        case GBA_CMD_IS_CONNECTING:                   /* 0x20 CP_POLL / 0x21 CP_END */
        case GBA_CMD_FINISH_CONNECT: {
            /* 2026-09-11 (decomp research, librfu_rfu.c rfu_getConnectParentStatus /
             * rfu_CB_pollConnectParent + librfu.h status constants): this reply must ALWAYS be
             * exactly ONE data word, laid out as
             *     status<<24 | slot<<16 | (id & 0xFFFF)
             * with status 0x00=done, 0x01=connecting, 0x02=slot closed, 0x03=disconnected.
             * Returning 0 words (the old no-peer path) makes librfu read STALE buffer bytes as the
             * status/id, which can strand the connect state machine. Mask the id to 16 bits rather
             * than relying on the peer id happening to be 16-bit clean. */
            uint32_t id = io->peer_id ? io->peer_id(io->ctx) : 0;
            if (id == 0) { out[0] = 0x02000000; return 1; }        /* no peer -> slot closed */
            if (!io->connected && io->connecting_polls < 3) {
                io->connecting_polls++;
                out[0] = 0x01000000;                               /* still connecting */
                return 1;
            }
            io->connected = true;
            out[0] = (id & 0xFFFFu);                               /* status 0x00 = connected, slot 0 */
            return 1;
        }

        case GBA_CMD_SLOT_STATUS:                     /* 0x14 — see gba_wap.h: a 0-word ack makes
                                                       * librfu's watchLink underflow its packet
                                                       * count to 255 and over-read its rx buffer.
                                                       * One word keeps that arithmetic sane. */
            out[0] = 0x00000000;
            return 1;

        case GBA_CMD_DISCONNECT:                      /* 0x30 -> the player backed out / link torn down */
            /* Clear our connect state so a subsequent Connect starts clean instead of inheriting
             * "already connected" (which would skip the connecting-poll fakeout and desync the
             * GBA's link manager). Bare ack is correct for the disconnect itself. */
            io->connected = false;
            io->connecting_polls = 0;
            return 0;

        case GBA_CMD_CPR_START:                       /* 0x32/0x33/0x34 connection RECOVERY family */
        case GBA_CMD_CPR_POLL:
        case GBA_CMD_CPR_END: {
            /* CPR does NOT share the CP status layout — corrected 2026-09-11 after a decomp audit.
             * rfu_getConnectRecoveryStatus reads the status from data[4], i.e. the LOW byte of data
             * word 0 (librfu_rfu.c:1198,1221-1223) — not bits 24-31 like CP_POLL/CP_END. The link
             * manager completes a POLL on status < 2 and treats END as success only on status == 0
             * (AgbRfu_LinkManager.c:813,825).
             * The first version of this arm used the CP layout, which was actively harmful: the
             * "no peer" value 0x03000000 decoded as status 0 = "recovery SUCCEEDED", telling the GBA
             * a lost link had been restored when it hadn't.
             * Status values (librfu.h RC_STATUS_*): 0 = OK/recovered, 1 = failed,
             * 2 = still searching for the parent. */
            uint32_t id = io->peer_id ? io->peer_id(io->ctx) : 0;
            out[0] = (id != 0) ? 0x00000000u    /* recovered */
                               : 0x00000001u;   /* failed — no peer to recover to */
            return 1;
        }

        case GBA_CMD_RECV_DATA:                       /* 0x26 / 0x28 -> count header + parent frame */
        case GBA_CMD_RECV_DATA_WAIT_RESP:
            /* docs/22 wake-word model: the adapter's clock-master window is only a 2-word wake-up
             * notification (0x99660028) — the GBA then pulls the actual data HERE, as clock master
             * again. take_frame answers with the received-byte-count header word + the parent LLSF
             * frame (afska wireless_adapter.md, ReceiveData). take_slot remains the legacy raw-slot
             * reply for backends that never grew a frame path (selftest stubs, 2-chip build). */
            if (io->take_frame) return io->take_frame(out, 24, io->ctx);
            return io->take_slot ? io->take_slot(out, 8, io->ctx) : 0;

        /* ---- ASYNC_ACK family: 0x25/0x27/0x35/0x36/0x37 ------------------------------------
         * Per pokefirered librfu_intr.c the GBA expects the ack for the command it sent —
         * 0x25->0xA5, 0x27->0xA7, 0x35->0xB5, 0x37->0xB7 (cmd|0x80, no trailing word) — and that
         * ack flips the GBA's msMode to AGB_CLK_SLAVE: it stops driving SC and waits for the
         * ADAPTER to send a wake-up notification as clock master. The CONFIG_GBA_SPI_CLOCK_MASTER
         * transport implements that (correct ack + wait-then-wake, docs/22); the legacy transport
         * path still fakes it with the historical 0xA8+idle pattern. */
        case GBA_CMD_SEND_DATA:                       /* 0x24 ID_DATA_TX_REQ — send WITHOUT a clock
                                                       * change. Was falling through to the default
                                                       * bare ack, which replies correctly but never
                                                       * called put_slot: the GBA's data was silently
                                                       * DROPPED. librfu parses no reply for 0x24
                                                       * (librfu_rfu.c:1723-1747), so a bare ack is
                                                       * right — we just have to keep the payload. */
            if (io->put_slot) io->put_slot(data, len, io->ctx);
            return 0;

        case GBA_CMD_SEND_DATA_WAIT:                  /* 0x25 -> the GBA's outgoing 14B slot */
            if (io->put_slot) io->put_slot(data, len, io->ctx);
            *action = GBA_WAP_ASYNC_ACK;              /* transport fakes the completion + clock change */
            return 0;

        case GBA_CMD_RECV_DATA_WAIT:                  /* 0x27 -> payload-less clock master/slave change */
            *action = GBA_WAP_ASYNC_ACK;
            return 0;

        /* 0x35: decomp audit (2026-09-11) found pokefirered's librfu has NO SENDER for
         * ID_UNK35_REQ at all — it appears only in the retry/ack-recognition lists. 0x25 and 0x35
         * differ by a single bit, and this link has documented single-bit misreads, so the
         * "0x35 appeared after ~590 commands" sighting is far better explained as a CORRUPTED 0x25
         * than as a real command. Kept in the clock-change family (harmless, and it matches 0x25's
         * handling if it really is a mangled 0x25), but it is counted separately so a future
         * capture can settle it. Do NOT treat its presence as protocol evidence. */
        case GBA_CMD_UNK35_AND_CHANGE:
        case GBA_CMD_RESUME_RETRANSMIT_AND_CHANGE:    /* 0x37 (parent-only; unreachable as a child) */
            *action = GBA_WAP_ASYNC_ACK;
            return 0;

        /* 0x36 is ADAPTER->GBA only (librfu_intr.c:185,233 — consumed as reqCommandId 0x0136 with a
         * link-loss bitmap in data[5]). It can never arrive as a command from the GBA, so the old
         * clock-change arm here was dead code that would have mis-acked if framing ever slipped.
         * Falls through to the plain bare ack now. */
        case GBA_CMD_UNK36_AND_CHANGE:
            return 0;

        default:                                      /* Init/Setup/Broadcast/StartHost/...: bare ack */
            return 0;
    }
}
