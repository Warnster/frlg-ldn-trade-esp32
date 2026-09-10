/* gba_wap.c — see gba_wap.h. Faithful C port of the RP2040 adapter logic. Pure logic, no hardware. */
#include "gba_wap.h"
#include <string.h>

/* login.rs challenge/response table (verbatim). */
uint32_t gba_login_next(uint32_t rx, bool *done)
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
int gba_wap_respond(gba_wap_io *io, uint8_t command, const uint32_t *data, int len,
                    uint32_t *out, gba_wap_action *action)
{
    *action = GBA_WAP_REPLY;

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

        case GBA_CMD_IS_CONNECTING:                   /* 0x20 / 0x21 */
        case GBA_CMD_FINISH_CONNECT: {
            uint32_t id = io->peer_id ? io->peer_id(io->ctx) : 0;
            if (id == 0) return 0;
            if (!io->connected && io->connecting_polls < 3) {
                io->connecting_polls++;
                out[0] = 0x01000000;                  /* still connecting */
                return 1;
            }
            io->connected = true;
            out[0] = id;                              /* connected: report the peer id */
            return 1;
        }

        case GBA_CMD_RECV_DATA:                       /* 0x26 / 0x28 -> latest Switch slot */
        case GBA_CMD_RECV_DATA_WAIT_RESP:
            return io->take_slot ? io->take_slot(out, 8, io->ctx) : 0;

        case GBA_CMD_SEND_DATA_WAIT:                  /* 0x25 -> the GBA's outgoing 14B slot */
            if (io->put_slot) io->put_slot(data, len, io->ctx);
            *action = GBA_WAP_ASYNC_ACK;              /* transport fakes the completion + clock change */
            return 0;

        case GBA_CMD_RECV_DATA_WAIT:                  /* 0x27 -> payload-less clock master/slave change */
            *action = GBA_WAP_ASYNC_ACK;
            return 0;

        default:                                      /* Init/Setup/Broadcast/StartHost/...: bare ack */
            return 0;
    }
}
