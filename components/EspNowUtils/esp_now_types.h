#pragma once

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

namespace EspNowUtils
{
    /* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
    #if CONFIG_ESPNOW_WIFI_MODE_STATION
        #define ESPNOW_WIFI_MODE WIFI_MODE_STA
        #define ESPNOW_WIFI_IF WIFI_IF_STA
    #else
        #define ESPNOW_WIFI_MODE WIFI_MODE_AP
        #define ESPNOW_WIFI_IF ESP_IF_WIFI_AP
    #endif

    #define ESPNOW_QUEUE_SIZE 6

    #define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcastMac, ESP_NOW_ETH_ALEN) == 0)

    enum EspNowEventIdType
    {
        ESPNOW_SEND_CB,
        ESPNOW_RECV_CB,
    }; // EspNowEventIdType

    struct EspNowEventSendCallbackType
    {
        uint8_t mac_addr[ESP_NOW_ETH_ALEN];
        esp_now_send_status_t status;
    }; // EspNowEventSendCallbackType

    struct EspNowEventRecvCallbackType
    {
        uint8_t mac_addr[ESP_NOW_ETH_ALEN];
        uint8_t *data;
        int data_len;
    }; // EspNowEventRecvCallbackType

    union EspNowEventInfoType
    {
        EspNowEventSendCallbackType send_cb;
        EspNowEventRecvCallbackType recv_cb;
    }; // EspNowEventInfoType

    struct EspNowEventType
    {
        EspNowEventIdType id;
        EspNowEventInfoType info;
    }; // EspNowEventType

    enum
    {
        ESPNOW_DATA_BROADCAST,
        ESPNOW_DATA_UNICAST,
        ESPNOW_DATA_MAX,
    };

    struct __attribute__((packed)) EspNowDataType
    {
        uint8_t type;       // Broadcast or unicast ESPNOW data.
        uint8_t state;      // Indicate that if has received broadcast ESPNOW data or not.
        uint16_t seq_num;   // Sequence number of ESPNOW data.
        uint16_t crc;       // CRC16 value of ESPNOW data.
        uint32_t magic;     // Magic number which is used to determine which device to send unicast ESPNOW data.
        uint8_t payload[ESP_NOW_MAX_DATA_LEN - CONFIG_ESPNOW_SEND_LEN]; // Real payload of ESPNOW data.
    }; // EspNowDataType

    struct EspNowSendParamType
    {
        bool unicast;                       // Send unicast ESPNOW data.
        bool broadcast;                     // Send broadcast ESPNOW data.
        uint8_t state;                      // Indicate that if has received broadcast ESPNOW data or not.
        uint32_t magic;                     // Magic number which is used to determine which device to send unicast ESPNOW data.
        uint16_t count;                     // Total count of unicast ESPNOW data to be sent.
        uint16_t delay;                     // Delay between sending two ESPNOW data, unit: ms.
        int len;                            // Length of ESPNOW data to be sent, unit: byte.
        uint8_t *buffer;                    // Buffer pointing to ESPNOW data.
        uint8_t dest_mac[ESP_NOW_ETH_ALEN]; // MAC address of destination device.
    }; // EspNowSendParamType
} // EspNowUtils