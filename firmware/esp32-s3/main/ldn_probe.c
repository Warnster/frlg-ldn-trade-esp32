#include <ctype.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "ldn_led.h"
#if CONFIG_LDN_PROBE_CONTROL_PORT
#include "ldn_brain.h"
#include "ldn_control.h"
#include "ldn_pico.h"
#include "gba_spi.h"   /* experimental single-chip GBA-direct SPI-slave front-end (CONFIG_GBA_SPI_*) */
#if CONFIG_LDN_GBA_SINGLE_CHIP
#include "gba_relay.h" /* single-chip: the core1<->core0 relay backing ldn_gba.c (docs/16 Phase 2) */
#endif
#include "ldn_session.h"
#include "ldn_udp.h"
#include "ldn_wire.h"
#define printf ldn_wire_printf
static esp_netif_t *s_station_netif;
#endif

#if !CONFIG_IDF_TARGET_ESP32C6 && !CONFIG_IDF_TARGET_ESP32S3
#error "The LDN firmware supports only ESP32-C6 / ESP32-S3"
#endif

#if CONFIG_LDN_PROBE_PRIVATE_JOIN
#include "ldn_private_wifi.h"
#include "ldn_advert.h"
#include "ldn_auth.h"
#include "ldn_keys.h"

/* FRLG's built-in LDN passphrase (pokeldn GBA_APP_PASSPHRASE), 64 bytes. */
static const uint8_t GBA_APP_PASSPHRASE[64] = {
    0xfc,0xb6,0xf6,0xad,0xb9,0xdf,0xea,0x66,0xac,0xa9,0xc3,0x26,0x14,0x9d,0x2b,0x3b,
    0x08,0xa7,0x81,0x89,0x5c,0xbf,0x78,0xf7,0x20,0xd7,0x8b,0x85,0xa5,0x75,0x84,0xa9,
    0x96,0x65,0xd2,0x37,0x79,0x7b,0x2a,0x41,0xdd,0xef,0x14,0x06,0x3e,0xc2,0x8d,0x25,
    0x91,0x43,0xaf,0x78,0x32,0xfb,0x3c,0xbc,0xf2,0x75,0x9c,0xbf,0xbd,0xc8,0x1d,0x8c };

#if !CONFIG_LDN_PROBE_CONTROL_PORT || !CONFIG_LDN_PROBE_EXPORT_ADVERTISEMENTS
#error "Private join requires the dynamic serial profile; build with probe.ps1 -Mode serial"
#endif

#if ESP_IDF_VERSION_MAJOR != 6 || ESP_IDF_VERSION_MINOR != 1
#error "The private CCMP probe is ABI-pinned to ESP-IDF v6.1"
#endif

#endif

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

static const char *TAG = "ldn_probe";
typedef struct {
    uint32_t promisc_action_count;
    uint32_t roc_action_count;
    uint32_t promisc_ldn_count;
    uint32_t roc_ldn_count;
    uint32_t eapol_dropped;
    uint8_t last_category;
    uint8_t last_source[6];
    uint8_t last_bssid[6];
} probe_stats_t;

static probe_stats_t s_stats;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_probe_task;
#if CONFIG_LDN_PROBE_PRIVATE_JOIN
static void remember_association(const uint8_t *frame, size_t length);
#endif

#if CONFIG_LDN_PROBE_EXPORT_ADVERTISEMENTS
static struct {
    uint8_t body[1536];
    uint8_t source[6];
    uint8_t channel;
    size_t length;
} s_advertisement;

static void export_advertisement(void)
{
    static uint8_t body[1536];
    static char hex[3073];
    static const char digits[] = "0123456789abcdef";
    uint8_t source[6];
    portENTER_CRITICAL(&s_stats_lock);
    const size_t length = s_advertisement.length;
    const uint8_t channel = s_advertisement.channel;
    memcpy(body, s_advertisement.body, length);
    memcpy(source, s_advertisement.source, sizeof(source));
    s_advertisement.length = 0;
    portEXIT_CRITICAL(&s_stats_lock);
    if (length == 0) {
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        hex[2 * i] = digits[body[i] >> 4];
        hex[2 * i + 1] = digits[body[i] & 15];
    }
    hex[2 * length] = '\0';
    printf("LDN_ADV " MACSTR " %u %s\n", MAC2STR(source), channel, hex);
    fflush(stdout);
}
#endif

static void log_action_stats(const char *phase)
{
    portENTER_CRITICAL(&s_stats_lock);
    const probe_stats_t stats = s_stats;
    portEXIT_CRITICAL(&s_stats_lock);
    ESP_LOGI(TAG,
             "%s action frames: roc=%" PRIu32 " promiscuous=%" PRIu32
             " ldn-prefix-broadcast: roc=%" PRIu32 " promiscuous=%" PRIu32
             " last src=" MACSTR " bssid=" MACSTR " category=0x%02x",
             phase, stats.roc_action_count, stats.promisc_action_count,
             stats.roc_ldn_count, stats.promisc_ldn_count,
             MAC2STR(stats.last_source), MAC2STR(stats.last_bssid),
             stats.last_category);
#if CONFIG_LDN_PROBE_EXPORT_ADVERTISEMENTS
    export_advertisement();
#endif
}

static void remember_action(const uint8_t *header, const uint8_t *body,
                            size_t body_len, bool from_roc, uint8_t channel)
{
    if (header == NULL || body == NULL || body_len == 0) {
        return;
    }
    static const uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static const uint8_t ldn_prefix[] = {0x7f, 0x00, 0x22, 0xaa, 0x04};
    const bool ldn = body_len >= sizeof(ldn_prefix) &&
                     memcmp(header + 4, broadcast, sizeof(broadcast)) == 0 &&
                     memcmp(body, ldn_prefix, sizeof(ldn_prefix)) == 0;
    portENTER_CRITICAL(&s_stats_lock);
#if CONFIG_LDN_PROBE_EXPORT_ADVERTISEMENTS
    if (ldn && body_len <= sizeof(s_advertisement.body)) {
        memcpy(s_advertisement.body, body, body_len);
        memcpy(s_advertisement.source, header + 10, 6);
        s_advertisement.channel = channel;
        s_advertisement.length = body_len;
    }
#else
    (void)channel;
#endif
    memcpy(s_stats.last_source, header + 10, sizeof(s_stats.last_source));
    memcpy(s_stats.last_bssid, header + 16, sizeof(s_stats.last_bssid));
    s_stats.last_category = body[0];
    if (from_roc) {
        ++s_stats.roc_action_count;
        s_stats.roc_ldn_count += ldn;
    } else {
        ++s_stats.promisc_action_count;
        s_stats.promisc_ldn_count += ldn;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

static void promiscuous_rx(void *buffer, wifi_promiscuous_pkt_type_t type)
{
    if (buffer == NULL) {
        return;
    }

    const wifi_promiscuous_pkt_t *packet = buffer;
    const uint8_t *frame = packet->payload;
    const size_t length = packet->rx_ctrl.sig_len;
#if CONFIG_LDN_PROBE_CONTROL_PORT
    if (type == WIFI_PKT_DATA || type == WIFI_PKT_CTRL || type == WIFI_PKT_MGMT) {
        ldn_control_sniff(frame, length);
    }
#endif
    if (type != WIFI_PKT_MGMT) return;
#if CONFIG_LDN_PROBE_PRIVATE_JOIN
    remember_association(frame, length);
#endif

    /* Management type 0, action subtype 13, little-endian frame control. */
    /* sig_len includes the four-byte FCS, which is not action payload. */
    if (length >= 29 && (frame[0] & 0xfcU) == 0xd0U) {
        remember_action(frame, frame + 24, length - 28, false, packet->rx_ctrl.channel);
    }
}

static void enable_management_sniffer(void)
{
    const wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
#if CONFIG_LDN_PROBE_CONTROL_PORT
            | WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_CTRL
#endif
        ,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
#if CONFIG_LDN_PROBE_CONTROL_PORT
    const wifi_promiscuous_filter_t control = {.filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ACK};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&control));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promiscuous_rx));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
}

#if !CONFIG_LDN_PROBE_PRIVATE_JOIN
static int roc_action_rx(uint8_t *header, uint8_t *payload, size_t length,
                         uint8_t channel)
{
    remember_action(header, payload, length, true, channel);
    return 0;
}

static void roc_done(uint32_t context, uint8_t op_id,
                     wifi_roc_done_status_t status)
{
    (void)context;
    (void)op_id;
    (void)status;
    if (s_probe_task != NULL) {
        xTaskNotifyGive(s_probe_task);
    }
}

#if CONFIG_LDN_PROBE_SEND_TEST_ACTION
static void send_test_action(void)
{
    static const uint8_t body[] = {
        0x7f,             /* Vendor-specific action category. */
        0x18, 0xfe, 0x34, /* Espressif OUI. */
        'L', 'D', 'N', 'P', 'R', 'O', 'B', 'E',
    };
    const size_t request_size = sizeof(wifi_action_tx_req_t) + sizeof(body);
    wifi_action_tx_req_t *request = calloc(1, request_size);
    if (request == NULL) {
        ESP_LOGE(TAG, "cannot allocate action request");
        return;
    }

    request->ifx = WIFI_IF_STA;
    memset(request->dest_mac, 0xff, sizeof(request->dest_mac));
    request->type = WIFI_OFFCHAN_TX_REQ;
    request->channel = CONFIG_LDN_PROBE_CHANNEL;
    request->sec_channel = WIFI_SECOND_CHAN_NONE;
    request->wait_time_ms = 100;
    request->no_ack = true;
    request->rx_cb = roc_action_rx;
    request->op_id = 0x41;
    memset(request->bssid, 0xff, sizeof(request->bssid));
    request->data_len = sizeof(body);
    memcpy(request->data, body, sizeof(body));

    const esp_err_t error = esp_wifi_action_tx_req(request);
    ESP_LOGI(TAG, "esp_wifi_action_tx_req: %s", esp_err_to_name(error));
    free(request);
}
#endif

static void run_public_probe(void)
{
    s_probe_task = xTaskGetCurrentTaskHandle();
    ldn_led_set(LDN_LED_IDLE);
    ESP_LOGI(TAG, "public probe on 2.4 GHz channel %d",
             CONFIG_LDN_PROBE_CHANNEL);

#if CONFIG_LDN_PROBE_SEND_TEST_ACTION
    send_test_action();
#endif

    for (uint8_t operation = 1;; ++operation) {
        wifi_roc_req_t request = {
            .ifx = WIFI_IF_STA,
            .type = WIFI_ROC_REQ,
            .channel = CONFIG_LDN_PROBE_CHANNEL,
            .sec_channel = WIFI_SECOND_CHAN_NONE,
            .wait_time_ms = 5000,
            .rx_cb = roc_action_rx,
            .op_id = operation,
            .done_cb = roc_done,
            .allow_broadcast = true,
        };

        const esp_err_t error = esp_wifi_remain_on_channel(&request);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_remain_on_channel: %s",
                     esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(6000));
        log_action_stats("public");
    }
}
#endif

#if CONFIG_LDN_PROBE_PRIVATE_JOIN
static int (*s_original_sta_connect)(uint8_t *bssid);
static uint8_t s_target_bssid[6];
static uint8_t s_ccmp_key[16];
static uint8_t s_station_mac[6];
static bool s_association_seen;
static char s_ssid[33];
static uint8_t s_ssid_bytes[16];
static bool s_ssid_bytes_valid;
static bool s_keys_installed;
/* On-chip auto-config: scan -> derive (embedded prod.keys) -> join -> resolve IPs -> start brain. */
/* AUTO_ARMED sits between "confirmed the Switch is here" and "actually joined it": the peer is
 * advertised to the GBA and we WAIT for the player to select it (GBA Connect 0x1f) before EMU joins
 * the real Switch. This matches the real union-room flow (list -> select -> join) and stops EMU from
 * knocking on the Switch before the GBA has picked the room. */
typedef enum { AUTO_OFF, AUTO_SCAN, AUTO_ARMED, AUTO_JOINING, AUTO_AUTH, AUTO_GETIP, AUTO_RUN } auto_state_t;
static auto_state_t s_auto;
/* Derived join config, captured in AUTO_SCAN and applied in AUTO_ARMED once the GBA selects us. */
static char s_armed_ssid[33], s_armed_ccmp[33], s_armed_bssid[18];
static unsigned s_armed_ch;
static bool s_target_confirmed;          /* a real Switch advert has been derived -> advertise the peer */
static uint32_t s_connect_baseline;      /* ldn_pico_connect_count() when we entered AUTO_ARMED */
static int64_t s_auth_time;
static int s_auth_attempts;
static volatile bool s_reset_requested;
static int64_t s_join_started;
static bool s_joining;
/* Channel-retry for the GBA-committed join (2026-09-11): the Switch's advert is heard on different
 * channels across runs, so the arm-time channel can be wrong by the time the player selects. Once
 * committed (Connect fired), if an association attempt fails (STA_DISCONNECTED before keys install),
 * cycle 1/6/11 and retry until one lands or the deadline passes — instead of the wrong channel
 * silently never reaching the Switch. */
static bool s_join_committed;
static volatile bool s_join_retry;
static int64_t s_join_deadline;
static int s_join_ch_idx;
static const uint8_t s_join_chans[3] = {1, 6, 11};

static void remember_association(const uint8_t *frame, size_t length)
{
    if (length < 34) {
        return;
    }
    const uint8_t subtype = frame[0] & 0xfcU;
    if ((subtype != 0x10 && subtype != 0x30) ||
        memcmp(frame + 4, s_station_mac, 6) != 0 ||
        memcmp(frame + 10, s_target_bssid, 6) != 0 ||
        memcmp(frame + 16, s_target_bssid, 6) != 0 ||
        frame[26] != 0 || frame[27] != 0) {
        return;
    }
    portENTER_CRITICAL(&s_stats_lock);
    const bool notify = !s_association_seen;
    s_association_seen = true;
    portEXIT_CRITICAL(&s_stats_lock);
    if (notify && s_probe_task != NULL) {
        xTaskNotifyGive(s_probe_task);
    }
#if CONFIG_LDN_PROBE_CONTROL_PORT
    if (notify) printf("LDN_DIAG assoc_resp_seen subtype=0x%02x\n", subtype);
#endif
}

/* Exact RSN element emitted by kinnay/LDN: CCMP, PSK, capabilities 0x000c. */
static uint8_t s_ldn_rsn_ie[] = {
    0x30, 0x14, 0x01, 0x00,
    0x00, 0x0f, 0xac, 0x04,
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
    0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,
    0x0c, 0x00,
};

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    value = (char)tolower((unsigned char)value);
    return value >= 'a' && value <= 'f' ? value - 'a' + 10 : -1;
}

static bool parse_hex(const char *text, uint8_t *output, size_t output_len,
                      char separator)
{
    if (text == NULL || output == NULL || output_len == 0 ||
        output_len > SIZE_MAX / 3) {
        return false;
    }
    const size_t expected_len = output_len * 2 +
        (separator != '\0' ? output_len - 1 : 0);
    if (strlen(text) != expected_len) {
        return false;
    }
    for (size_t i = 0; i < output_len; ++i) {
        const int high = hex_nibble(*text++);
        const int low = hex_nibble(*text++);
        if (high < 0 || low < 0) {
            return false;
        }
        output[i] = (uint8_t)((high << 4) | low);
        if (separator != '\0' && i + 1 < output_len && *text++ != separator) {
            return false;
        }
    }
    return *text == '\0';
}

static int probe_sta_connect(uint8_t *bssid)
{
    /* Copy mode takes raw IE bytes. Reference mode requires a two-byte length
       prefix and overwrites it; passing the IE directly corrupts its tag/size. */
    int result = esp_wifi_set_appie_internal(
        LDN_WIFI_APPIE_RSN, s_ldn_rsn_ie, sizeof(s_ldn_rsn_ie), 0);
    if (result != 0) {
        ESP_LOGE(TAG, "pre-connect RSN IE install failed: %d", result);
        return result;
    }

    result = s_original_sta_connect != NULL ? s_original_sta_connect(bssid) : 0;
    if (result != 0) {
        return result;
    }

    /* The stock callback rebuilds the RSN IE before starting association. */
    result = esp_wifi_set_appie_internal(
        LDN_WIFI_APPIE_RSN, s_ldn_rsn_ie, sizeof(s_ldn_rsn_ie), 0);
    if (result != 0) {
        ESP_LOGE(TAG, "post-connect RSN IE install failed: %d", result);
    }
#if CONFIG_LDN_PROBE_CONTROL_PORT
    printf("LDN_DIAG sta_connect result=%d\n", result);
#endif
    return result;
}

static void probe_sta_connected(uint8_t *bssid)
{
    if (bssid == NULL || memcmp(bssid, s_target_bssid, sizeof(s_target_bssid)) != 0) {
        ESP_LOGE(TAG, "unexpected association BSSID; ignoring callback");
        return;
    }
    ESP_LOGI(TAG, "WPA connected callback from " MACSTR,
             MAC2STR(s_target_bssid));
}

static int probe_rx_eapol(uint8_t *source, uint8_t *buffer, uint32_t length)
{
    (void)source;
    (void)buffer;
    (void)length;
    portENTER_CRITICAL(&s_stats_lock);
    ++s_stats.eapol_dropped;
    portEXIT_CRITICAL(&s_stats_lock);
    return 0;
}

static bool probe_in_4way_handshake(void)
{
    return false;
}

static void install_wpa_hook(void)
{
    ESP_ERROR_CHECK(wpa_cb == NULL ? ESP_ERR_INVALID_STATE : ESP_OK);
    struct ldn_wpa_funcs *probe = malloc(sizeof(*probe));
    ESP_ERROR_CHECK(probe == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    memcpy(probe, wpa_cb, sizeof(*probe));

    s_original_sta_connect = probe->wpa_sta_connect;
    probe->wpa_sta_connect = probe_sta_connect;
    probe->wpa_sta_connected_cb = probe_sta_connected;
    probe->wpa_sta_rx_eapol = probe_rx_eapol;
    probe->wpa_sta_in_4way_handshake = probe_in_4way_handshake;

    const int result = esp_wifi_register_wpa_cb_internal(probe);
    if (result != 0) {
        free(probe);
        ESP_ERROR_CHECK(result);
    }
    /* register_wpa_cb frees the old table; keep the supplicant owner in sync. */
    wpa_cb = probe;
    ESP_LOGW(TAG, "private ESP-IDF v6.1 WPA hook installed");
}

static bool read_key_back(int expected_index, enum ldn_key_flag flag,
                          const uint8_t *expected)
{
    uint8_t interface = WIFI_IF_STA;
    int algorithm = LDN_WIFI_WPA_ALG_NONE;
    int index = expected_index;
    uint8_t address[6];
    uint8_t key[16];
    memcpy(address, s_target_bssid, sizeof(address));
    memset(key, 0, sizeof(key));

    const int result = esp_wifi_get_sta_key_internal(
        &interface, &algorithm, address, &index, key, sizeof(key), flag);
    const bool matches = result == 0 && algorithm == LDN_WIFI_WPA_ALG_CCMP &&
                         index == expected_index &&
                         memcmp(key, expected, sizeof(key)) == 0;
    ESP_LOGI(TAG, "read key index %d: result=%d alg=%d match=%s",
             expected_index, result, algorithm, matches ? "yes" : "no");
    return matches;
}

static void install_ldn_keys(void)
{
    /* S3 ORDER (from easyworld's board-verified S3 trade): the S3 driver only finalizes the
       station context / CAM key slots once the handshake is reported done, so authorize BEFORE
       injecting the keys. The C6 used the opposite order — inheriting it left the group key in a
       slot that could not decrypt the host's (broadcast) Pia frames, so RX stayed empty. */
    const bool authorized = esp_wifi_auth_done_internal();
    ESP_LOGI(TAG, "esp_wifi_auth_done_internal: %s", authorized ? "true" : "false");
#if CONFIG_LDN_PROBE_CONTROL_PORT
    printf("LDN_DIAG auth_done=%d\n", authorized);
#endif

    /* The pinned blob copies eight RSC bytes even for a six-byte CCMP PN. */
    uint8_t sequence[8] = {0};
    const enum ldn_key_flag pairwise_flags =
        LDN_KEY_FLAG_PAIRWISE | LDN_KEY_FLAG_RX | LDN_KEY_FLAG_TX;
    const enum ldn_key_flag group_flags = LDN_KEY_FLAG_GROUP | LDN_KEY_FLAG_RX;

    const int pairwise = esp_wifi_set_sta_key_internal(
        LDN_WIFI_WPA_ALG_CCMP, s_target_bssid, 0, 1, sequence,
        sizeof(sequence), s_ccmp_key, sizeof(s_ccmp_key), pairwise_flags);
    const int group = esp_wifi_set_sta_key_internal(
        LDN_WIFI_WPA_ALG_CCMP, s_target_bssid, 1, 0, sequence,
        sizeof(sequence), s_ccmp_key, sizeof(s_ccmp_key), group_flags);

    ESP_LOGI(TAG, "key install: pairwise=%d group=%d", pairwise, group);
    /* Readback cannot verify keys on the S3 (the CAM stores them obfuscated); diagnostic only. */
    const bool group_ok = read_key_back(1, LDN_KEY_FLAG_GROUP, s_ccmp_key);
#if CONFIG_LDN_PROBE_CONTROL_PORT
    printf("LDN_DIAG key_install pairwise=%d group=%d group_readback=%d\n",
           pairwise, group, group_ok);
#endif
    (void)group_ok;
    if (pairwise != 0 || group != 0) {
        ESP_LOGE(TAG, "CCMP key injection failed; not authorizing port");
        ldn_led_set(LDN_LED_ERROR);
        return;
    }
    s_keys_installed = true;
    ldn_led_set(LDN_LED_LINKED);
}

static void wifi_event(void *argument, esp_event_base_t base, int32_t id,
                       void *data)
{
    (void)argument;
    (void)base;
    if (id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *event = data;
        portENTER_CRITICAL(&s_stats_lock);
        const uint32_t eapol_dropped = s_stats.eapol_dropped;
        portEXIT_CRITICAL(&s_stats_lock);
        ESP_LOGI(TAG, "STA_CONNECTED aid=%u channel=%u, eapol dropped=%" PRIu32,
                 event->aid, event->channel, eapol_dropped);
#if CONFIG_LDN_PROBE_CONTROL_PORT
        printf("LDN_DIAG sta_connected aid=%u channel=%u\n", event->aid, event->channel);
        ldn_control_link(true);
#endif
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event = data;
        ESP_LOGW(TAG, "STA_DISCONNECTED reason=%u", event->reason);
        ldn_led_set(LDN_LED_IDLE);
#if CONFIG_LDN_PROBE_CONTROL_PORT
        printf("LDN_DIAG sta_disconnected reason=%u\n", event->reason);
        ldn_control_link(false);
        /* Link dropped — stop the brain and re-scan from scratch (the room may reappear on a
         * different channel). The heavy reset (esp_wifi_stop/start) runs in the loop, not here. */
        if (s_join_committed && !s_keys_installed) {
            /* Association attempt failed on this channel (e.g. NO_AP_FOUND: the Switch is on another
             * of 1/6/11). Don't drop the GBA's commit — just retry the next channel (AUTO_JOINING). */
            s_join_retry = true;
        } else if (s_auto != AUTO_OFF) {
            ldn_brain_stop(); s_auto = AUTO_SCAN; s_reset_requested = true;
        }
#endif
    }
}

const char *ldn_session_ssid(void) { return s_ssid[0] ? s_ssid : "-"; }

int ldn_session_ssid_bytes(uint8_t out[16])
{
    if (!s_ssid_bytes_valid) return -1;
    memcpy(out, s_ssid_bytes, 16);
    return 0;
}

void ldn_session_identity(uint8_t our_mac[6], uint8_t host_mac[6])
{
    memcpy(our_mac, s_station_mac, 6);
    memcpy(host_mac, s_target_bssid, 6);
}

void ldn_session_stop(void)
{
    s_joining = false;
    s_join_committed = false; s_join_retry = false;   /* any teardown ends a committed join */
    ldn_udp_stop();
    esp_wifi_disconnect();
    /* Stop the driver before replacing key material or accepting another room. */
    esp_wifi_stop();
    s_association_seen = false;
    memset(s_ccmp_key, 0, sizeof(s_ccmp_key));
    memset(s_ssid, 0, sizeof(s_ssid));
    s_ssid_bytes_valid = false;
    s_keys_installed = false;
    memset(s_target_bssid, 0, sizeof(s_target_bssid));
    ldn_control_target(s_target_bssid);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    enable_management_sniffer();
    portENTER_CRITICAL(&s_stats_lock);
    s_advertisement.length = 0;
    portEXIT_CRITICAL(&s_stats_lock);
    ldn_led_set(LDN_LED_IDLE);
}

esp_err_t ldn_session_scan(unsigned channel)
{
    if (channel < 1 || channel > 11 || s_joining || s_ssid[0]) return ESP_ERR_INVALID_STATE;
    return esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

esp_err_t ldn_session_configure(const char *ssid, const char *bssid, const char *key, unsigned channel)
{
    uint8_t host[6], secret[16], ssid_bytes[16];
    if (channel < 1 || channel > 11 || !parse_hex(ssid, ssid_bytes, 16, '\0') ||
        !parse_hex(bssid, host, 6, ':') || (host[0] & 1) || !parse_hex(key, secret, 16, '\0'))
        return ESP_ERR_INVALID_ARG;
    ldn_session_stop();
    memcpy(s_ssid, ssid, 33);
    memcpy(s_ssid_bytes, ssid_bytes, 16); s_ssid_bytes_valid = true;
    memcpy(s_target_bssid, host, 6); memcpy(s_ccmp_key, secret, 16);
    memset(secret, 0, sizeof(secret));
    /* A new station identity avoids reusing a CCMP replay context on reconnect. */
    esp_wifi_stop();
    esp_fill_random(s_station_mac, 6); s_station_mac[0] = (s_station_mac[0] & 0xfc) | 2;
    esp_err_t result = esp_wifi_set_mac(WIFI_IF_STA, s_station_mac);
    if (result != ESP_OK) return result;
    esp_wifi_start(); esp_wifi_set_ps(WIFI_PS_NONE); enable_management_sniffer();
    ldn_control_target(s_target_bssid);

    wifi_config_t config = {0};
    memcpy(config.sta.ssid, s_ssid, 32);
    memcpy(config.sta.password, "00000000", 8);
    memcpy(config.sta.bssid, s_target_bssid, sizeof(s_target_bssid));
    config.sta.bssid_set = true;
    config.sta.channel = channel;
    config.sta.scan_method = WIFI_FAST_SCAN;
    config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = false;
    config.sta.pmf_cfg.required = false;

    result = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (result != ESP_OK) return result;
    s_association_seen = false; s_joining = true; s_join_started = esp_timer_get_time();
    result = esp_wifi_connect();
    if (result != ESP_OK) s_joining = false;
    else ldn_led_set(LDN_LED_JOINING);
    return result;
}

/* Snapshot the most recent captured LDN advertisement (thread-safe). Returns length or 0. */
static size_t auto_snapshot_advert(uint8_t *frame, size_t cap, uint8_t src[6], uint8_t *channel)
{
    portENTER_CRITICAL(&s_stats_lock);
    size_t len = s_advertisement.length;
    if (len > cap) len = 0;
    if (len) {
        memcpy(frame, s_advertisement.body, len);
        memcpy(src, s_advertisement.source, 6);
        *channel = s_advertisement.channel;
    }
    portEXIT_CRITICAL(&s_stats_lock);
    return len;
}

/* ---- trade-leader detection + real host identity (2026-09-11) --------------------------------
 * pokeldn ground truth (ldn/transport.py, ldn/beacon.py, vendor/LDN AdvertisementInfo.decode):
 *  - The decrypted advert plaintext `pt` is AdvertisementInfo V2 (big-endian): server_random[0:16],
 *    challenge[16:24], security_mode[24], accept_policy[25], app_version[26:28], pad[28:36],
 *    band/chan[36:38], max_participants[38], num_participants[39], then num*48-byte participant
 *    entries at [40], then a BE u16 app_data length, then application_data.
 *  - application_data = a 0x5C-byte Pia system header, then a custom base85 blob that decodes
 *    (5 chars -> 4 LE bytes; digit = c-0x23 for c<0x5C else c-0x24) to a 24-byte FRLG record:
 *    record[0:2]=in-game TID (LE), record[2:10]=OT name (FRLG charmap, 0xFF-padded — the SAME
 *    charmap the GBA uname uses, so it copies straight through), record[16:18]=search word (LE):
 *    activity = word & 0x7F (ACTIVITY_TRADE=4, ACTIVITY_SEARCH=12=just standing in the union room),
 *    started = word & 0x8000.
 *  This is the ONLY pre-join view of the host's game state — the real trainer name/TID are here,
 *  NOT in the "Lewis"-style LDN participant username (that's a different, console-level name). */
#define FRLG_HOST_COMM_ID   0x01006fa0233f8000ULL   /* FRLG title id, HOST role (joiner=0x0100610011000000) */
#define LDN_ACCEPT_NONE     1
#define GBA_ACTIVITY_TRADE  4
#define LDN_PIA_HDR_LEN     0x5C

/* Decode the 24-byte FRLG game-state record out of the decrypted advert plaintext. 0 on success. */
static int auto_decode_beacon(const uint8_t *pt, int ptl, uint16_t *tid, uint8_t name8[8],
                              uint8_t *activity, bool *started)
{
    if (ptl < 40) return -1;
    int num = pt[39];
    int off = 40 + 48 * num;
    if (off + 2 > ptl) return -1;
    int size = (pt[off] << 8) | pt[off + 1];      /* BE u16 app_data length */
    off += 2;
    if (size < LDN_PIA_HDR_LEN || off + size > ptl) return -1;
    const uint8_t *b85 = pt + off + LDN_PIA_HDR_LEN;   /* skip the Pia system header */
    int b85len = size - LDN_PIA_HDR_LEN;
    uint8_t rec[32]; int reclen = 0;
    for (int i = 0; i + 5 <= b85len && reclen + 4 <= (int)sizeof(rec); i += 5) {
        uint32_t v = 0;
        for (int k = 4; k >= 0; k--) {             /* reversed: first char = least-significant digit */
            uint8_t c = b85[i + k];
            v = v * 85u + ((c < 0x5C) ? (uint32_t)(c - 0x23) : (uint32_t)(c - 0x24));
        }
        rec[reclen++] = v & 0xFF; rec[reclen++] = (v >> 8) & 0xFF;
        rec[reclen++] = (v >> 16) & 0xFF; rec[reclen++] = (v >> 24) & 0xFF;
    }
    if (reclen < 24) return -1;
    *tid = (uint16_t)(rec[0] | (rec[1] << 8));
    memcpy(name8, rec + 2, 8);
    uint16_t word = (uint16_t)(rec[16] | (rec[17] << 8));
    *activity = (uint8_t)(word & 0x7F);
    *started = (word & 0x8000) != 0;
    return 0;
}

/* Render an 8-byte FRLG-charmap name to ASCII for logging only (mirrors pokeldn _frlg_name). */
static void frlg_name_ascii(const uint8_t name8[8], char out[9])
{
    int n = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t x = name8[i];
        if (x == 0xFF) break;
        if (x >= 0xBB && x <= 0xD4) out[n++] = 'A' + (x - 0xBB);
        else if (x >= 0xD5 && x <= 0xEE) out[n++] = 'a' + (x - 0xD5);
        else if (x >= 0xA1 && x <= 0xAA) out[n++] = '0' + (x - 0xA1);
        else out[n++] = (x == 0) ? ' ' : '?';
    }
    out[n] = 0;
}

/* The GBA player's real name (ASCII, decoded from its Broadcast 0x16) to present to the Switch as
 * the joining trainer — replaces the hardcoded "EMU". Falls back to "EMU" if the GBA hasn't
 * broadcast its identity yet (e.g. two-chip path, or before the first 0x16). */
static void gba_joiner_name(char out[10])
{
    uint16_t tid; uint8_t nm[8]; char a[9];
    if (ldn_pico_get_gba_identity(&tid, nm)) {
        frlg_name_ascii(nm, a);
        /* Only use it if it decoded to a real name — the GBA's 0x16 broadcast turned out NOT to
         * carry the trainer name (bytes there aren't FRLG-charmap letters -> '?'), so reject any
         * '?'/empty result and fall back to EMU rather than show garbage on the Switch. The real
         * GBA name has to come from the NI game-data instead (TODO). */
        int ok = a[0] != 0;
        for (const char *p = a; *p; ++p) if (*p == '?') { ok = 0; break; }
        if (ok) { strncpy(out, a, 9); out[9] = 0; return; }
    }
    strcpy(out, "EMU");
}

/* Evaluate a captured advert: derive the join config AND decide whether it's a TRADE LEADER (a FRLG
 * host with an open seat that has actually started a trade), extracting the host's real name/TID.
 * Returns 0 = trade leader (all outs incl. name8/tid filled); 1 = a valid advert but NOT a trade
 * leader (join config filled, identity not — do not advertise the peer); <0 = no/undecodable advert. */
static int auto_scan_evaluate(uint8_t ssid[16], uint8_t ccmp[16], uint8_t bssid[6], unsigned *channel,
                              uint16_t *tid, uint8_t name8[8], uint8_t *activity, bool *started)
{
    uint8_t frame[1536], src[6], chan, pt[512];
    size_t len = auto_snapshot_advert(frame, sizeof(frame), src, &chan);
    if (len == 0) return -1;
    long ptl = ldn_advert_decrypt(&LDN_KEYS, frame, len, pt);
    if (ptl < 40) return -1;
    if (ldn_advert_ssid(frame, len, ssid) != 0) return -1;
    if (ldn_derive_data_key(&LDN_KEYS, pt, 16, GBA_APP_PASSPHRASE, sizeof(GBA_APP_PASSPHRASE), ccmp) != 0)
        return -1;
    memcpy(bssid, src, 6);
    *channel = chan;
    /* Trade-leader gate. Proven checks first (host role + open seat), then the FRLG activity. */
    if (ldn_advert_local_comm_id(frame, len) != FRLG_HOST_COMM_ID) return 1;   /* not a FRLG host */
    uint8_t accept = pt[25], maxp = pt[38], nump = pt[39];
    if (accept == LDN_ACCEPT_NONE || nump >= maxp) return 1;                    /* no open seat */
    if (auto_decode_beacon(pt, (int)ptl, tid, name8, activity, started) != 0) return 1;  /* no game state */
    if (*activity != GBA_ACTIVITY_TRADE) return 1;                              /* union/search, not trade */
    return 0;
}

/* After join the host re-advertises with us listed; parse the V2 participant list for our IP + the
 * host's IP/MAC. 0 when both are resolved. */
static int auto_get_ips(const uint8_t our_mac[6], uint8_t our_ip[4], uint8_t host_ip[4],
                        uint8_t host_mac[6])
{
    uint8_t frame[1536], src[6], chan, pt[512];
    size_t len = auto_snapshot_advert(frame, sizeof(frame), src, &chan);
    if (len == 0) return -1;
    long ptl = ldn_advert_decrypt(&LDN_KEYS, frame, len, pt);
    if (ptl < 40) return -1;
    int num = pt[39];                       /* V2: num_participants at offset 39 */
    int found_us = 0, found_host = 0;
    for (int i = 0; i < num; ++i) {         /* each entry: ip(4) mac(6) index(1) plat(1) name(32) pad(4) */
        size_t off = 40 + 48 * (size_t)i;
        if (off + 48 > (size_t)ptl) break;
        const uint8_t *ip = pt + off, *mac = pt + off + 4;
        uint8_t index = pt[off + 10];
        int zero_ip = !(ip[0] | ip[1] | ip[2] | ip[3]);
        if (index == 0 && !zero_ip) { memcpy(host_ip, ip, 4); memcpy(host_mac, mac, 6); found_host = 1; }
        if (memcmp(mac, our_mac, 6) == 0 && !zero_ip) { memcpy(our_ip, ip, 4); found_us = 1; }
    }
    return (found_us && found_host) ? 0 : -1;
}

/* Build + send the LDN AuthenticationFrame that registers us as a participant (so the host assigns
 * our IP). Fields come from the room advertisement + the embedded prod.keys. 0 on TX ok. */
static int auto_send_auth(void)
{
    uint8_t frame[1536], src[6], chan, pt[512];
    size_t len = auto_snapshot_advert(frame, sizeof(frame), src, &chan);
    if (len < 45) return -1;
    long ptl = ldn_advert_decrypt(&LDN_KEYS, frame, len, pt);
    if (ptl < 24) return -1;
    uint8_t ssid[16];
    if (ldn_advert_ssid(frame, len, ssid) != 0) return -1;
    uint64_t comm_id = ldn_advert_local_comm_id(frame, len);
    uint16_t scene = (uint16_t)((frame[22] << 8) | frame[23]);   /* advert NetworkId is big-endian */
    int version = frame[44];
    uint8_t server_random[16]; memcpy(server_random, pt, 16);
    uint64_t token = 0; for (int i = 0; i < 8; i++) token = (token << 8) | pt[16 + i];  /* BE64 */
    uint8_t client_random[16]; esp_fill_random(client_random, sizeof(client_random));
    uint64_t nonce = 0; esp_fill_random(&nonce, sizeof(nonce));
    char jn[10]; gba_joiner_name(jn);   /* the GBA player's real name, not "EMU" */
    {   /* diagnostic: why is the joiner name what it is? show capture state + raw 0x16 layout */
        uint16_t dtid = 0; uint8_t dnm[8]; int dvalid = ldn_pico_get_gba_identity(&dtid, dnm);
        uint32_t bw[6], bseen = 0; ldn_pico_gba_bcast_dbg(bw, &bseen);
        printf("LDN_AUTO joiner='%s' gba_id_valid=%d tid=0x%04x bcast_seen=%u raw=%08x %08x %08x %08x %08x %08x\n",
               jn, dvalid, (unsigned)dtid, (unsigned)bseen,
               (unsigned)bw[0], (unsigned)bw[1], (unsigned)bw[2], (unsigned)bw[3], (unsigned)bw[4], (unsigned)bw[5]);
    }
    uint8_t authf[1024];
    int n = ldn_build_auth_frame(&LDN_KEYS, 3, version, comm_id, scene, ssid, server_random,
                                 client_random, token, nonce, jn, 88, authf, sizeof(authf));
    if (n < 0) return -1;
    int r = ldn_control_tx_ldn(authf, n);
    printf("LDN_AUTO auth_tx v=%d len=%d result=%d\n", version, n, r);
    return r == 0 ? 0 : -1;
}

void ldn_session_auto_start(void)
{
    if (s_auto == AUTO_OFF) { s_auto = AUTO_SCAN; printf("LDN_AUTO scanning\n"); }
}

/* One step of the standalone auto-config state machine. Called from run_private_join. */
static void auto_poll(void)
{
    if (s_auto == AUTO_SCAN) {
        uint8_t ssid[16], ccmp[16], bssid[6]; unsigned ch = 0;
        /* Hop 1/6/11 until the room's advertisement is heard (LDN rooms aren't always on ch1). */
        static const uint8_t chans[3] = {1, 6, 11};
        static int hop_i = 0; static int64_t hop_t = 0;
        int64_t hnow = esp_timer_get_time();
        if (hop_t == 0) { esp_wifi_set_channel(chans[0], WIFI_SECOND_CHAN_NONE); hop_t = hnow; }
        else if (hnow - hop_t > 1200000) {
            hop_i = (hop_i + 1) % 3;
            esp_wifi_set_channel(chans[hop_i], WIFI_SECOND_CHAN_NONE);
            hop_t = hnow;
            printf("LDN_AUTO scan ch=%u\n", chans[hop_i]);
        }
        uint16_t tid = 0; uint8_t name8[8] = {0}, activity = 0; bool started = false;
        int ev = auto_scan_evaluate(ssid, ccmp, bssid, &ch, &tid, name8, &activity, &started);
        /* Only ARM (and thus advertise the peer to the GBA) once the Switch is a TRADE LEADER —
         * a FRLG host with an open seat that has started a trade (ev==0). A Switch merely sitting in
         * a menu or standing in the union room (ev==1) must NOT be shown to the GBA. This is the fix
         * for "Ash appears even when the Switch isn't leading a trade". */
        if (ev == 0 && ch >= 1 && ch <= 11) {
            static const char d[] = "0123456789abcdef";
            for (int i = 0; i < 16; ++i) { s_armed_ssid[2*i]=d[ssid[i]>>4]; s_armed_ssid[2*i+1]=d[ssid[i]&15];
                                           s_armed_ccmp[2*i]=d[ccmp[i]>>4]; s_armed_ccmp[2*i+1]=d[ccmp[i]&15]; }
            s_armed_ssid[32] = s_armed_ccmp[32] = 0;
            snprintf(s_armed_bssid, sizeof(s_armed_bssid), MACSTR, MAC2STR(bssid));
            s_armed_ch = ch;
            /* Present the host's REAL in-game trainer name + TID to the GBA (decoded from the advert),
             * instead of the hardcoded "Ash"/0xc979. */
            ldn_pico_set_peer_identity(tid, name8);
            /* Lock onto the Switch's channel + start advertising the peer, but DON'T join yet.
             * Wait for the GBA to select us (Connect 0x1f) — the real "list -> pick" flow. */
            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            s_target_confirmed = true;
            s_connect_baseline = ldn_pico_connect_count();
            s_auto = AUTO_ARMED;
            char nm[9]; frlg_name_ascii(name8, nm);
            printf("LDN_AUTO armed ch=%u TRADE leader name='%s' tid=0x%04x started=%d (peer advertised, awaiting GBA select)\n",
                   ch, nm, tid, (int)started);
        } else if (ev == 1) {
            static int64_t last_skip; int64_t hn = esp_timer_get_time();
            if (hn - last_skip > 3000000) { last_skip = hn;
                printf("LDN_AUTO advert seen but NOT a trade leader (not host / no open seat / not trade) — not shown to GBA\n");
            }
        }
    } else if (s_auto == AUTO_ARMED) {
        /* The peer is in the GBA's Join-Group list (advertised from the main loop). Join the Switch
         * ONLY when the player picks it — a new Connect (0x1f) forwarded by the Pico. */
        if (ldn_pico_connect_count() != s_connect_baseline) {
            s_join_committed = true; s_join_retry = false;
            s_join_deadline = esp_timer_get_time() + 25000000;   /* 25s to land the association */
            /* try the armed channel first, then cycle the other two */
            s_join_ch_idx = (s_armed_ch == 6) ? 1 : (s_armed_ch == 11) ? 2 : 0;
            esp_err_t r = ldn_session_configure(s_armed_ssid, s_armed_bssid, s_armed_ccmp, s_armed_ch);
            printf("LDN_AUTO gba-selected -> configure ch=%u result=%d\n", s_armed_ch, (int)r);
            s_auto = AUTO_JOINING;                     /* stay committed; retry path owns channel cycling */
            if (r != ESP_OK) s_join_retry = true;
        }
    } else if (s_auto == AUTO_JOINING) {
        if (s_keys_installed) {
            s_join_committed = false;                  /* associated — stop cycling channels */
            s_auth_attempts = 0;
            s_auto = AUTO_AUTH;
            printf("LDN_AUTO joined; authenticating\n");
        } else if (s_join_retry) {
            s_join_retry = false;
            if (esp_timer_get_time() > s_join_deadline) {
                printf("LDN_AUTO join: no channel associated in time -> rescanning\n");
                s_join_committed = false; s_target_confirmed = false; s_auto = AUTO_SCAN;
            } else {
                unsigned ch = s_join_chans[s_join_ch_idx % 3]; s_join_ch_idx++;
                esp_err_t r = ldn_session_configure(s_armed_ssid, s_armed_bssid, s_armed_ccmp, ch);
                printf("LDN_AUTO join retry ch=%u result=%d\n", ch, (int)r);
                if (r != ESP_OK) s_join_retry = true;  /* couldn't start — advance again next tick */
            }
        }
    } else if (s_auto == AUTO_AUTH) {
        /* Register as an LDN participant so the host assigns our IP. */
        auto_send_auth();
        s_auth_time = esp_timer_get_time();
        s_auth_attempts++;
        s_auto = AUTO_GETIP;
    } else if (s_auto == AUTO_GETIP) {
        uint8_t our_ip[4], host_ip[4], host_mac[6], ssid[16];
        if (auto_get_ips(s_station_mac, our_ip, host_ip, host_mac) == 0 &&
            ldn_session_ssid_bytes(ssid) == 0) {
            char line[64];
            snprintf(line, sizeof(line), "LDN_NET %u.%u.%u.%u %u.%u.%u.%u",
                     our_ip[0], our_ip[1], our_ip[2], our_ip[3],
                     host_ip[0], host_ip[1], host_ip[2], host_ip[3]);
            ldn_udp_command(line, true);
            snprintf(line, sizeof(line), "LDN_NEIGH %u.%u.%u.%u %02x%02x%02x%02x%02x%02x",
                     host_ip[0], host_ip[1], host_ip[2], host_ip[3],
                     host_mac[0], host_mac[1], host_mac[2], host_mac[3], host_mac[4], host_mac[5]);
            ldn_udp_command(line, true);
            char jn[10]; gba_joiner_name(jn);   /* present the GBA player's real name to the Switch */
            ldn_brain_start(ssid, our_ip, host_ip, s_station_mac, s_target_bssid, jn);
            s_auto = AUTO_RUN;
        } else if (esp_timer_get_time() - s_auth_time > 2000000) {
            /* No participant IP yet — re-send the auth frame (bounded), else give up to SCAN. */
            if (s_auth_attempts < 12) s_auto = AUTO_AUTH;
            else { printf("LDN_AUTO auth gave up; rescanning\n"); s_auto = AUTO_SCAN; }
        }
    }
}

static void run_private_join(void)
{
    s_probe_task = xTaskGetCurrentTaskHandle();
    ldn_led_set(LDN_LED_IDLE);
    ldn_pico_init();
    ldn_control_init(s_station_netif, s_target_bssid);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    int64_t last_advertisement = 0;
    int64_t last_brain_tick = 0;
    bool diag_gate = false;
    for (;;) {
        ldn_control_poll();
        const int64_t now = esp_timer_get_time();
        /* Drive the standalone trade brain at ~60 Hz (VBlank cadence). */
        if (ldn_brain_active() && now - last_brain_tick >= 16667) {
            last_brain_tick = now;
            ldn_brain_tick();
        }
        if (s_reset_requested) {   /* link dropped: clean up the session, then AUTO_SCAN re-finds it */
            s_reset_requested = false;
            ldn_session_stop();
            printf("LDN_AUTO reset; re-scanning\n");
        }
        /* GBA-activity gate: only join the Switch while the real GBA is active in wireless mode
         * (the Pico forwards its RFU traffic to us). No GBA -> stay out of the Switch's room. */
        {
            static uint32_t s_last_wap; static int64_t s_last_wap_time;
            uint32_t st[5]; ldn_pico_stats(st);
            if (st[0] != s_last_wap) { s_last_wap = st[0]; s_last_wap_time = now; }
            bool gba_active = s_last_wap_time && (now - s_last_wap_time < 5000000);   /* 5s: pre-commit idle */
            bool gba_gone   = s_last_wap_time && (now - s_last_wap_time > 30000000); /* 30s: truly gone */
            if (gba_active && s_auto == AUTO_OFF) {
                printf("LDN_AUTO gba-active -> joining\n");
                ldn_session_auto_start();
            } else if (!gba_active && s_auto == AUTO_SCAN) {
                /* Only abandon on a brief idle while still SCANNING. Once ARMED (a trade partner is
                 * shown to the GBA) we stay sticky through the browse pauses — the player takes time
                 * to pick, and the old 5s rule kept dropping ARMED so the Connect landed in an
                 * un-armed window and the join never triggered. ARMED/JOINING+ are torn down only by
                 * the 30s "truly gone" path below (or a real link drop). */
                /* GBA went quiet BEFORE committing to a join — abandon and wait for it to come back. */
                printf("LDN_AUTO gba-idle -> leaving\n");
                ldn_brain_stop(); ldn_session_stop(); s_auto = AUTO_OFF;
                s_target_confirmed = false;
            } else if (gba_gone && s_auto != AUTO_OFF) {
                /* 2026-09-11 fix: once the GBA has SELECTED us and we're joining/authenticating/
                 * running (AUTO_JOINING+), a brief quiet is EXPECTED — the GBA waits for the
                 * connection + the Switch's trade data to flow back through the relay. Tearing the
                 * session down on the old 5s rule was killing a WORKING join one step past
                 * host_accepted (observed: NI accepted, then gba-idle -> STA_DISCONNECT reason=8).
                 * So in committed states only give up after a much longer silence (30s) that a real
                 * post-connect handshake never reaches; a genuine link loss still tears down
                 * immediately via STA_DISCONNECTED -> AUTO_SCAN. */
                printf("LDN_AUTO gba truly gone (30s) -> leaving\n");
                ldn_brain_stop(); ldn_session_stop(); s_auto = AUTO_OFF;
                s_target_confirmed = false;
            }
        }
        if (s_auto != AUTO_OFF && s_auto != AUTO_RUN) auto_poll();
        ldn_pico_poll();   /* drain the GBA's WAP stream continuously (FIFO relief) */
        {   /* Advertise the peer to the GBA only once we've CONFIRMED a real Switch is present
             * (a valid advert was derived). This puts the Switch in the GBA's Join-Group list as a
             * selectable partner without EMU having joined yet — the player's selection (Connect 0x1f)
             * is what drives the join. Advertising a phantom before confirmation would let the GBA
             * pick a peer with no Switch behind it. */
            static int64_t last_peer_adv;
            if (s_target_confirmed && now - last_peer_adv > 400000) {
                last_peer_adv = now;
                ldn_pico_advertise_peer();
            }
            /* standalone Pico-link status (visible even when EMU is not joined) */
            static int64_t last_pico_print;
            if (now - last_pico_print > 2000000) {
                last_pico_print = now;
                uint32_t st[5]; ldn_pico_stats(st);
                printf("PICO_LINK wap=%u slots=%u taken=%u recv=%u peer=%u connect=%u armed=%d auto=%d\n",
                       (unsigned)st[0], (unsigned)st[1], (unsigned)st[2], (unsigned)st[3],
                       (unsigned)st[4], (unsigned)ldn_pico_connect_count(),
                       (int)s_target_confirmed, s_auto);
                /* What flow is the GBA in? bcastRead(1c/1d/1e)=looking-for-rooms(join);
                 * startHost(19)/accept(1a)=hosting; connect(1f)=selected a room; send(25)=trade. */
                printf("CMD_HIST b16=%u host19=%u acc1a=%u bcS1c=%u bcP1d=%u bcE1e=%u conn1f=%u snd25=%u"
                       " | POST-CONNECT isConn20=%u finish21=%u recv26=%u recvW27=%u recvR28=%u chg35=%u last=0x%02x\n",
                       (unsigned)ldn_pico_cmd_count(0x16), (unsigned)ldn_pico_cmd_count(0x19),
                       (unsigned)ldn_pico_cmd_count(0x1a), (unsigned)ldn_pico_cmd_count(0x1c),
                       (unsigned)ldn_pico_cmd_count(0x1d), (unsigned)ldn_pico_cmd_count(0x1e),
                       (unsigned)ldn_pico_cmd_count(0x1f), (unsigned)ldn_pico_cmd_count(0x25),
                       (unsigned)ldn_pico_cmd_count(0x20), (unsigned)ldn_pico_cmd_count(0x21),
                       (unsigned)ldn_pico_cmd_count(0x26), (unsigned)ldn_pico_cmd_count(0x27),
                       (unsigned)ldn_pico_cmd_count(0x28), (unsigned)ldn_pico_cmd_count(0x35),
                       (unsigned)g_gba_last_cmd);
                /* what the Pico relay reports back: did our peer adverts arrive + commit there? */
                uint32_t pd[3]; ldn_pico_diag(pd);
                printf("PICO_RX rx_bytes=%u peer_commits=%u peer_present=%u\n",
                       (unsigned)pd[0], (unsigned)pd[1], (unsigned)pd[2]);
            }
        }
        if (s_joining && s_association_seen && !diag_gate) {
            diag_gate = true;
            printf("LDN_DIAG gate assoc_seen sta_running=%d\n",
                   esp_wifi_sta_is_running_internal());
        }
        if (s_joining && s_association_seen && esp_wifi_sta_is_running_internal()) {
            s_joining = false; install_ldn_keys();
        } else if (s_joining && !s_join_committed && now - s_join_started > 15000000) {
            /* Backstop only for a non-committed join; a GBA-committed join is governed by the
             * channel-retry + 25s deadline in AUTO_JOINING (don't let this fight it). */
            ldn_session_stop(); printf("LDN_ERROR ASSOCIATION_TIMEOUT\n");
            ldn_led_set(LDN_LED_ERROR);
            if (s_auto != AUTO_OFF) s_auto = AUTO_SCAN;   /* retry the whole auto sequence */
        }
        if (now - last_advertisement >= 250000) { export_advertisement(); last_advertisement = now; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
#endif

/* Standalone ESP-brain step 1: prove cport (the FRLG<->LDN trade brain) compiles, links, and
 * runs on the S3 — AES via mbedtls + zstd via the Xtensa component. Logged at boot (plaintext,
 * before the wire silences logs). */
#include "aes_backend.h"
#include "pia_zstd.h"
static void cport_selftest(void)
{
    /* AES-128-ECB round trip (mbedtls backend). NIST FIPS-197 test vector. */
    static const uint8_t key[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
                                    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    static const uint8_t pt[16] = {0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
                                   0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a};
    uint8_t ct[16], back[16];
    int e = aes128_ecb_encrypt_block(key, pt, ct);
    int d = aes128_ecb_decrypt_block(key, ct, back);
    bool aes_ok = (e == 0 && d == 0 && memcmp(back, pt, 16) == 0 && ct[0] == 0x3a && ct[1] == 0xd7);
    /* zstd round trip via pia_zstd (Xtensa zstd component). */
    static const uint8_t blob[80] = {0};
    uint8_t comp[160], dec[160];
    long c = pia_compress(blob, sizeof(blob), comp, sizeof(comp));
    long z = c > 0 ? pia_decompress(comp, (size_t)c, dec, sizeof(dec)) : -1;
    bool zstd_ok = (z == (long)sizeof(blob) && memcmp(dec, blob, sizeof(blob)) == 0);
    ESP_LOGW("cport", "SELFTEST aes=%s zstd=%s (c=%ld z=%ld)",
             aes_ok ? "OK" : "FAIL", zstd_ok ? "OK" : "FAIL", c, z);
}

void app_main(void)
{
    ldn_led_init();
    cport_selftest();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash_bytes = 0;
    ESP_ERROR_CHECK(esp_flash_get_size(NULL, &flash_bytes));
    ESP_LOGI(TAG, "target=%s revision=%u cores=%u flash=%" PRIu32 " MiB IDF=%s",
             CONFIG_IDF_TARGET, chip.revision, chip.cores,
             flash_bytes / (1024 * 1024), esp_get_idf_version());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_LOGI(TAG, "console=USB Serial/JTAG");
#else
    ESP_LOGI(TAG, "console=UART");
#endif
#if CONFIG_GBA_SPI_BUS_PROBE
    /* GPIO bus-stall probe (docs/17 §7 follow-up): measures worst-case core-1 GPIO_IN_REG read
     * stalls under staged core-0 bus load. Never returns; no GBA needed. Leave OFF normally. */
    gba_spi_bus_probe();
#endif
#if CONFIG_GBA_SPI_DIRECT_SELFTEST
    /* Experimental single-chip path: talk to the GBA directly via the S3 SPI slave (no RP2040).
     * Never returns — see GBA_SPI_SLAVE_DESIGN.md §8 gate 1/2. Leave the Kconfig OFF for LDN. */
    gba_spi_selftest();
#endif
#if CONFIG_GBA_SPI_TRADE_RELAY_SELFTEST
    /* docs/16-single-chip-trade-plan.md Phase 1: bare core1 GBA adapter + the real gba_relay
     * backend + a synthetic Switch peer, no real Wi-Fi yet. Never returns. */
    gba_spi_trade_relay_selftest();
#endif
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_LDN_PROBE_CONTROL_PORT
    /* LDN assigns a static address after authentication; it does not use DHCP. */
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    base.flags &= ~ESP_NETIF_DHCP_CLIENT;
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    netif_config.base = &base;
    s_station_netif = esp_netif_new(&netif_config);
    ESP_ERROR_CHECK(s_station_netif == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(s_station_netif));
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
    ESP_LOGI(TAG, "LDN static-address interface ready; automatic DHCP disabled");
#else
    esp_netif_create_default_wifi_sta();
#endif

    const wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#if CONFIG_LDN_PROBE_FRESH_MAC
    uint8_t temporary_mac[6];
    esp_fill_random(temporary_mac, sizeof(temporary_mac));
    temporary_mac[0] = (temporary_mac[0] & 0xfc) | 2;
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, temporary_mac));
#endif
#if CONFIG_LDN_PROBE_LEGACY_PHY
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G));
#endif
#if CONFIG_LDN_PROBE_PRIVATE_JOIN
    install_wpa_hook();
#endif
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    enable_management_sniffer();

#if CONFIG_LDN_GBA_SINGLE_CHIP
    /* Single-chip (docs/16 Phase 2): bring up the bare-metal GBA adapter on core 1 BEFORE the LDN
     * join loop, wired to the shared-SRAM relay that ldn_gba.c serves to the brain. Same bring-up
     * sequence proven tonight in gba_spi_trade_relay_selftest (9,278 clean Union-Room commands),
     * minus that function's own monitor loop — run_private_join IS the core-0 loop here. Wi-Fi is
     * already started above; UNICORE confines it (+ its ISRs) to core 0, leaving core 1 for the
     * jitter-free bit-bang. This is the first time both run together (the one Phase-2 unknown). */
    {
        static gba_wap_io s_gba_io;
        gba_spi_init();
        gba_relay_init();
        gba_relay_fill_io(&s_gba_io);
        gba_spi_set_core1_io(&s_gba_io);
        gba_core1_start();
        ESP_LOGI(TAG, "single-chip: core1 GBA adapter up (relay-backed); LDN join on core0");
    }
#endif

#if CONFIG_LDN_PROBE_PRIVATE_JOIN
    run_private_join();
#else
    run_public_probe();
#endif
}
