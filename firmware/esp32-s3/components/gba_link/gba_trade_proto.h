/* gba_trade_proto — the Gen 3 (FireRed/LeafGreen) link-trade protocol vocabulary: the LINKCMD/
 * LINKTYPE control words, the handshake magics, and the byte-exact Pokémon / LinkPlayer block
 * structs the trade exchanges.
 *
 * PORTED 2026-09-12 from Celio-Firmware (src/link_defines.h, src/payloads/pokemon.hpp,
 * src/payloads/linkPlayer.h), which in turn mirrors the pokefirered decomp. This is the
 * hardware-INDEPENDENT layer of the link-cable cart side (docs/25): it is identical whether the
 * blocks travel over the physical link cable (cart side, this firmware) or over RFU-in-LDN (Switch
 * side, pokeldn). That sameness is exactly why the bridge is re-framing, not translation — so this
 * header is the shared "blocks" vocabulary both sides speak.
 *
 * Plain C (matches gba_spi's style); structs are byte-exact for the wire — _Static_assert guards
 * the sizes so a layout drift is a compile error, not a silent trade corruption. */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- LINKCMD control words (link.c) — the 16-bit commands that sequence a trade ---------------
 * The cart and the Switch drive the SAME set; only the ones on the trade path are commented. */
#define LINKCMD_EMPTY                   0x0000
#define LINKCMD_SEND_LINK_TYPE          0x2222
#define LINKCMD_READY_EXIT_STANDBY      0x2FFE
#define LINKCMD_SEND_PACKET             0x2FFF
#define LINKCMD_READY_CLOSE_LINK        0x5FFF
#define LINKCMD_SEND_EMPTY              0x6666
#define LINKCMD_SEND_0xEE               0x7777
#define LINKCMD_CONT_BLOCK              0x8888   /* continue a multi-chunk block */
#define LINKCMD_READY_TO_TRADE          0xAABB   /* both-ready gate before the exchange */
#define LINKCMD_READY_FINISH_TRADE      0xABCD
#define LINKCMD_INIT_BLOCK              0xBBBB   /* begin a block transfer (link player / party) */
#define LINKCMD_READY_CANCEL_TRADE      0xBBCC
#define LINKCMD_SEND_HELD_KEYS          0xCAFE   /* held-key slot (idle filler in the link loop) */
#define LINKCMD_SEND_BLOCK_REQ          0xCCCC   /* request the partner send a block */
#define LINKCMD_START_TRADE             0xCCDD
#define LINKCMD_CONFIRM_FINISH_TRADE    0xDCBA
#define LINKCMD_SET_MONS_TO_TRADE       0xDDDD   /* carries the chosen party slot(s) */
#define LINKCMD_PLAYER_CANCEL_TRADE     0xDDEE
#define LINKCMD_REQUEST_CANCEL          0xEEAA
#define LINKCMD_BOTH_CANCEL_TRADE       0xEEBB
#define LINKCMD_PARTNER_CANCEL_TRADE    0xEECC
#define LINKCMD_NONE                    0xEFFF

/* ---- LINKTYPE values (the session kind announced at connect) ---- */
#define LINKTYPE_TRADE               0x1111
#define LINKTYPE_TRADE_CONNECTING    0x1122
#define LINKTYPE_TRADE_SETUP         0x1133
#define LINKTYPE_TRADE_DISCONNECTED  0x1144

/* ---- link-layer handshake magics (multiplayer connect negotiation) ---- */
#define LINK_PLAYER_ID              0x81
#define LINK_MASTER_HANDSHAKE       0x8FFF
#define LINK_SLAVE_HANDSHAKE        0xB9A0   /* the word a child presents; master waits for this */
#define LINK_HANDSHAKE_DISABLE      0xD15E

/* ---- Gen 3 Pokémon structure (80-byte box mon, 100-byte party mon) ----------------------------
 * The 48-byte "secure" substruct block is, on the wire, encrypted + permuted per the mon's
 * personality; these structs describe the DECRYPTED, canonical-order layout (decryption/permutation
 * live in a separate step, same as pokefirered's BoxPokemon <-> plaintext). */
#define POKEMON_NAME_LENGTH 10
#define PLAYER_NAME_LENGTH   7

typedef struct {
    uint16_t species;
    uint16_t heldItem;
    uint32_t experience;
    uint8_t  ppBonuses;
    uint8_t  friendship;
    uint16_t filler;
} PokemonSubstruct0;              /* growth */

typedef struct {
    uint16_t moves[4];
    uint8_t  pp[4];
} PokemonSubstruct1;             /* attacks */

typedef struct {
    uint8_t hpEV, attackEV, defenseEV, speedEV, spAttackEV, spDefenseEV;
    uint8_t cool, beauty, cute, smart, tough, sheen;
} PokemonSubstruct2;             /* EVs + contest condition */

typedef struct {
    uint8_t  pokerus;
    uint8_t  metLocation;
    uint16_t metLevel:7;
    uint16_t metGame:4;
    uint16_t pokeball:4;
    uint16_t otGender:1;
    uint32_t hpIV:5;
    uint32_t attackIV:5;
    uint32_t defenseIV:5;
    uint32_t speedIV:5;
    uint32_t spAttackIV:5;
    uint32_t spDefenseIV:5;
    uint32_t isEgg:1;
    uint32_t abilityNum:1;
    uint32_t coolRibbon:3;
    uint32_t beautyRibbon:3;
    uint32_t cuteRibbon:3;
    uint32_t smartRibbon:3;
    uint32_t toughRibbon:3;
    uint32_t championRibbon:1;
    uint32_t winningRibbon:1;
    uint32_t victoryRibbon:1;
    uint32_t artistRibbon:1;
    uint32_t effortRibbon:1;
    uint32_t marineRibbon:1;
    uint32_t landRibbon:1;
    uint32_t skyRibbon:1;
    uint32_t countryRibbon:1;
    uint32_t nationalRibbon:1;
    uint32_t earthRibbon:1;
    uint32_t worldRibbon:1;
    uint32_t unusedRibbons:4;
    uint32_t modernFatefulEncounter:1;
} PokemonSubstruct3;             /* misc: met data, IVs, ribbons */

typedef struct {
    PokemonSubstruct0 type0;
    PokemonSubstruct1 type1;
    PokemonSubstruct2 type2;
    PokemonSubstruct3 type3;
} PokemonSubstruct;              /* 48 bytes, canonical order */

typedef struct {
    uint32_t personality;
    uint32_t otId;
    uint8_t  nickname[POKEMON_NAME_LENGTH];
    uint8_t  language;
    uint8_t  isBadEgg:1;
    uint8_t  hasSpecies:1;
    uint8_t  isEgg:1;
    uint8_t  blockBoxRS:1;
    uint8_t  unused:4;
    uint8_t  otName[PLAYER_NAME_LENGTH];
    uint8_t  markings;
    uint16_t checksum;
    uint16_t unknown;
    PokemonSubstruct secure;
} BoxPokemon;                    /* 80 bytes */

typedef struct {
    BoxPokemon box;
    uint32_t status;
    uint8_t  level;
    uint8_t  mail;
    uint16_t hp;
    uint16_t maxHP;
    uint16_t attack;
    uint16_t defense;
    uint16_t speed;
    uint16_t spAttack;
    uint16_t spDefense;
} Pokemon;                       /* 100 bytes */

/* ---- the 28-byte link-player record + its block wrapper (cable_club.c) ---- */
typedef struct {
    uint16_t version;
    uint16_t lp_field_2;
    uint16_t trainerId;
    uint16_t secretId;
    uint8_t  name[8];
    uint8_t  progressFlags;      /* low nibble = hasNationalDex, high nibble = clearedGame */
    uint8_t  neverRead;
    uint8_t  progressFlagsCopy;
    uint8_t  gender;
    uint32_t linkType;
    uint16_t id;
    uint16_t language;
} LinkPlayer;                    /* 28 bytes */

typedef struct {
    char       magic1[16];       /* GameFreak magic, framing the entry block */
    LinkPlayer linkPlayer;
    char       magic2[16];
} LinkPlayerBlock;

/* Wire-size guards: a layout drift here would silently corrupt a trade. */
_Static_assert(sizeof(PokemonSubstruct) == 48, "Gen3 secure substruct must be 48 bytes");
_Static_assert(sizeof(BoxPokemon)       == 80, "Gen3 BoxPokemon must be 80 bytes");
_Static_assert(sizeof(Pokemon)          == 100, "Gen3 party Pokemon must be 100 bytes");
_Static_assert(sizeof(LinkPlayer)       == 28, "Gen3 LinkPlayer must be 28 bytes");

#ifdef __cplusplus
}
#endif
