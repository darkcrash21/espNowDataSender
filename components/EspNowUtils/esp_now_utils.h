#pragma once

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <cstring>
#include <assert.h>

// ESP32 API Includes
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "esp_now_types.h"

namespace EspNowUtils
{
    void WifiInit(const char *tag);
    esp_err_t EspNowInit(void);
    void EspNowDeinit(EspNowSendParamType *send_param);
    void EspNowTask(void *pvParameter);
    void EspNowDataPrepare(EspNowSendParamType *send_param, void* data, size_t sizeOfDataInBytes);
    int EspNowDataParse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic);
    void EspNowSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);
    void EspNowRecvCallback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
} // EspNowUtils