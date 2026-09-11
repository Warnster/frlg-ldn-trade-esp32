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

        case GBA_CMD_RECV_DATA:                       /* 0x26 / 0x28 -> latest Switch slot */
        case GBA_CMD_RECV_DATA_WAIT_RESP:
            return io->take_slot ? io->take_slot(out, 8, io->ctx) : 0;

        /* ---- ASYNC_ACK family: 0x25/0x27/0x35/0x36/0x37 ------------------------------------
         * KNOWN-WRONG, DELIBERATELY NOT YET CHANGED (2026-09-11 decomp research). The transport
         * currently answers all of these with a hardcoded header 0x996600A8 plus one extra idle
         * word. Per pokefirered librfu_intr.c the GBA actually expects the ack for the command it
         * sent — 0x25->0xA5, 0x27->0xA7, 0x35->0xB5, 0x37->0xB7 (i.e. just cmd|0x80, which
         * gba_wap_response_header already computes) — and NO trailing word (with ackLength=0 the
         * GBA stops clocking after the header, so our extra word is consumed as the next command's
         * header). 0xA8 is the GBA's OWN ack to an adapter-originated 0x28 in slave mode: the wrong
         * side's word.
         * WHY IT IS STILL HERE: those four correct acks are exactly the ones that set the GBA's
         * msMode = AGB_CLK_SLAVE — after them the GBA STOPS DRIVING SC and waits for the ADAPTER to
         * clock the link. We have no clock-master path at all (SC is input-only, gba_spi.c), so
         * sending the "correct" ack today would permanently silence the link (librfu_stwi.c: once
         * clock-slave, STWI_init returns ERR_REQ_CMD_CLOCK_SLAVE without touching SIOCNT).
         * Fixing this ack and implementing SC clock-master drive are ONE task and must land
         * together — that is the next architectural step for the trade DATA phase. */
        case GBA_CMD_SEND_DATA_WAIT:                  /* 0x25 -> the GBA's outgoing 14B slot */
            if (io->put_slot) io->put_slot(data, len, io->ctx);
            *action = GBA_WAP_ASYNC_ACK;              /* transport fakes the completion + clock change */
            return 0;

        case GBA_CMD_RECV_DATA_WAIT:                  /* 0x27 -> payload-less clock master/slave change */
            *action = GBA_WAP_ASYNC_ACK;
            return 0;

        case GBA_CMD_UNK35_AND_CHANGE:                /* 0x35/0x36/0x37 -> real agbrfu "_AND_CHANGE"
                                                          family (see gba_wap.h) — same clock-role-swap
                                                          fakeout as 0x25/0x27, 0 data words either way */
        case GBA_CMD_UNK36_AND_CHANGE:
        case GBA_CMD_RESUME_RETRANSMIT_AND_CHANGE:
            *action = GBA_WAP_ASYNC_ACK;
            return 0;

        default:                                      /* Init/Setup/Broadcast/StartHost/...: bare ack */
            return 0;
    }
}
