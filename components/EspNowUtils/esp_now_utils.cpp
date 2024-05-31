#include "esp_now_utils.h"

namespace EspNowUtils
{
    #define ESPNOW_MAXDELAY 512
    static const char* TAG;
    static QueueHandle_t s_espNowQueue;
    static uint8_t s_broadcastMac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint16_t s_espNowSeq[ESPNOW_DATA_MAX] = {0, 0};

    //
    // WiFi should start before using ESPNOW
    //
    void WifiInit(const char *tag)
    {
        TAG = (char*)tag;
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
        ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    #endif
    } // WifiInit()

    //
    // ESP Now Init
    //
    esp_err_t EspNowInit(void)
    {
        EspNowSendParamType *send_param;

        s_espNowQueue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(EspNowEventType));
        if (s_espNowQueue == NULL)
        {
            ESP_LOGE(TAG, "Create mutex fail");
            return ESP_FAIL;
        }

        // Initialize ESPNOW and register sending and receiving callback function
        ESP_ERROR_CHECK(esp_now_init());
        ESP_ERROR_CHECK(esp_now_register_send_cb(EspNowSendCallback));
        ESP_ERROR_CHECK(esp_now_register_recv_cb(EspNowRecvCallback));
    #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
        ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
        ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
    #endif
        // Set primary master key
        ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

        // Add broadcast peer information to peer list
        esp_now_peer_info_t *peer = new esp_now_peer_info_t();
        if (peer == NULL)
        {
            ESP_LOGE(TAG, "Peer information memory allocation failed");
            vSemaphoreDelete(s_espNowQueue);
            esp_now_deinit();
            return ESP_FAIL;
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, s_broadcastMac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        delete peer;

        // Initialize sending parameters
        send_param = new EspNowSendParamType();
        if (send_param == NULL)
        {
            ESP_LOGE(TAG, "Send parameter memory allocation failed");
            vSemaphoreDelete(s_espNowQueue);
            esp_now_deinit();
            return ESP_FAIL;
        }
        memset(send_param, 0, sizeof(EspNowSendParamType));
        send_param->unicast = false;
        send_param->broadcast = true;
        send_param->state = 0;
        send_param->magic = esp_random();
        send_param->count = CONFIG_ESPNOW_SEND_COUNT;
        send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
        send_param->len = sizeof(EspNowDataType);
        send_param->buffer = new uint8_t[sizeof(EspNowDataType)];
        if (send_param->buffer == NULL)
        {
            ESP_LOGE(TAG, "Send buffer memory allocation failed");
            delete send_param;
            vSemaphoreDelete(s_espNowQueue);
            esp_now_deinit();
            return ESP_FAIL;
        }
        memcpy(send_param->dest_mac, s_broadcastMac, ESP_NOW_ETH_ALEN);
        std::string msg = "Hello, anyone out there?";
        EspNowDataPrepare(send_param, (void*)msg.c_str(), msg.size());

        xTaskCreate(EspNowTask, "EspNowTask", 2048, send_param, 4, NULL);

        return ESP_OK;
    } // EspNowInit()

    //
    // ESP Now Deinit
    //
    void EspNowDeinit(EspNowSendParamType *send_param)
    {
        delete[] send_param->buffer;
        delete send_param;
        vSemaphoreDelete(s_espNowQueue);
        esp_now_deinit();
    } // EspNowDeinit()

    //
    // ESP Now Task to send and receive
    //
    void EspNowTask(void *pvParameter)
    {
        EspNowEventType evt;
        uint8_t recv_state = 0;
        uint16_t recv_seq = 0;
        int recv_magic = 0;
        bool is_broadcast = false;
        int ret;

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Start sending broadcast data");

        // Start sending broadcast ESPNOW data
        EspNowSendParamType *send_param = (EspNowSendParamType *)pvParameter;
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error");
            EspNowDeinit(send_param);
            vTaskDelete(NULL);
        }

        while (xQueueReceive(s_espNowQueue, &evt, portMAX_DELAY) == pdTRUE)
        {
            switch (evt.id)
            {
            case ESPNOW_SEND_CB:
            {
                EspNowEventSendCallbackType *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to %02x:%02x:%02x:%02x:%02x:%02x, status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false))
                {
                    break;
                }

                if (!is_broadcast)
                {
                    send_param->count--;
                    if (send_param->count == 0)
                    {
                        ESP_LOGI(TAG, "Send done");
                        EspNowDeinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                // Delay a while before sending the next data
                if (send_param->delay > 0)
                {
                    vTaskDelay(send_param->delay / portTICK_PERIOD_MS);
                }

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                std::string msg = "Hello, anyone out there?";
                EspNowDataPrepare(send_param, (void*)msg.c_str(), msg.size());

                EspNowDataType *buf = (EspNowDataType *)send_param->buffer;
                ESP_LOGI(TAG, "send data to %02x:%02x:%02x:%02x:%02x:%02x, with msg: %s", MAC2STR(send_cb->mac_addr), buf->payload);

                // Send the next data after the previous data is sent
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Send error");
                    EspNowDeinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                EspNowEventRecvCallbackType *recv_cb = &evt.info.recv_cb;

                ret = EspNowDataParse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                if (ret == ESPNOW_DATA_BROADCAST)
                {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: %02x:%02x:%02x:%02x:%02x:%02x, len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    // If MAC address does not exist in peer list, add it to peer list
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                    {
                        esp_now_peer_info_t *peer = new esp_now_peer_info_t();
                        if (peer == NULL)
                        {
                            ESP_LOGE(TAG, "Peer information allocation failed");
                            EspNowDeinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK(esp_now_add_peer(peer));
                        delete peer;
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST)
                {
                    // Assuming the payload is a string type message but this could be replaced with some binary data
                    EspNowDataType *buf = (EspNowDataType *)recv_cb->data;
                    ESP_LOGI(TAG, "Receive %dth unicast data from: %02x:%02x:%02x:%02x:%02x:%02x, msg: %s", recv_seq, MAC2STR(recv_cb->mac_addr), buf->payload);
                }
                else
                {
                    ESP_LOGI(TAG, "Receive error data from: %02x:%02x:%02x:%02x:%02x:%02x", MAC2STR(recv_cb->mac_addr));
                }
                delete[] recv_cb->data;

                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
            }
        }
    } // EspNowTask()

    //
    // Prepare ESPNOW data to be sent
    //
    void EspNowDataPrepare(EspNowSendParamType *send_param, void* data, size_t sizeOfDataInBytes)
    {
        EspNowDataType *buf = (EspNowDataType *)send_param->buffer;

        assert(send_param->len >= sizeof(EspNowDataType));

        buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
        buf->state = send_param->state;
        buf->seq_num = s_espNowSeq[buf->type]++;
        buf->crc = 0;
        buf->magic = send_param->magic;

        // Copy the transmit data into the payload
        if (sizeOfDataInBytes > sizeof(buf->payload))
        {
            ESP_LOGE(TAG, "Data to send is greater than allowed, capping to %d bytes!", sizeof(buf->payload));
            sizeOfDataInBytes = sizeof(buf->payload);
        }
        memset(buf->payload, 0, sizeof(buf->payload));
        memcpy(buf->payload, data, sizeOfDataInBytes);
        //esp_fill_random(buf->payload, send_param->len - sizeof(EspNowDataType));
        buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
    } // EspNowDataPrepare()
    
    //
    // Parse received ESPNOW data
    //
    int EspNowDataParse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
    {
        EspNowDataType *buf = (EspNowDataType *)data;
        uint16_t crc, crc_cal = 0;

        if (data_len < sizeof(EspNowDataType))
        {
            ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
            return -1;
        }

        *state = buf->state;
        *seq = buf->seq_num;
        *magic = buf->magic;
        crc = buf->crc;
        buf->crc = 0;
        crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

        if (crc_cal == crc)
        {
            return buf->type;
        }

        return -1;
    } // EspNowDataParse()

    // ESPNOW sending or receiving callback function is called in WiFi task.
    // Users should not do lengthy operations from this task. Instead, post
    // necessary data to a queue and handle it from a lower priority task.
    void EspNowSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        EspNowEventType evt;
        EspNowEventSendCallbackType *send_cb = &evt.info.send_cb;

        if (mac_addr == NULL)
        {
            ESP_LOGE(TAG, "Send callback arg error");
            return;
        }

        evt.id = ESPNOW_SEND_CB;
        memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
        send_cb->status = status;
        if (xQueueSend(s_espNowQueue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
        {
            ESP_LOGW(TAG, "Send send queue fail");
        }
    } // EspNowSendCallback()

    void EspNowRecvCallback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
    {
        EspNowEventType evt;
        EspNowEventRecvCallbackType *recv_cb = &evt.info.recv_cb;
        uint8_t *mac_addr = recv_info->src_addr;
        uint8_t *des_addr = recv_info->des_addr;

        if (mac_addr == NULL || data == NULL || len <= 0)
        {
            ESP_LOGE(TAG, "Receive callback arg error");
            return;
        }

        if (IS_BROADCAST_ADDR(des_addr))
        {
            // If added a peer with encryption before, the receive packets may be
            // encrypted as peer-to-peer message or unencrypted over the broadcast channel.
            // Users can check the destination address to distinguish it.
            ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
        }
        else
        {
            ESP_LOGD(TAG, "Receive unicast ESPNOW data");
        }

        evt.id = ESPNOW_RECV_CB;
        memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
        recv_cb->data = new uint8_t[len];
        if (recv_cb->data == NULL)
        {
            ESP_LOGE(TAG, "Receive data memory allocation failed");
            return;
        }
        memcpy(recv_cb->data, data, len);
        recv_cb->data_len = len;
        if (xQueueSend(s_espNowQueue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
        {
            ESP_LOGW(TAG, "Send receive queue fail");
            delete[] recv_cb->data;
        }
    } // EspNowRecvCallback()
} // EspNowUtils