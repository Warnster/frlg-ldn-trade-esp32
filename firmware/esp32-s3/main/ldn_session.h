#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

const char *ldn_session_ssid(void);
int ldn_session_ssid_bytes(uint8_t out[16]);
void ldn_session_identity(uint8_t our_mac[6], uint8_t host_mac[6]);
esp_err_t ldn_session_configure(const char *ssid, const char *bssid, const char *key, unsigned channel);
esp_err_t ldn_session_scan(unsigned channel);
void ldn_session_stop(void);
void ldn_session_auto_start(void);
void ldn_control_target(const unsigned char host[6]);
