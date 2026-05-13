#include <string.h>
#include <sys/param.h>
#include <lwip/netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "udp_client.h"

#define HOST_IP_ADDR "192.168.0.114"
#define PORT 16161
#define PAYLOAD_LEN 18U
#define DATA_LEN 14U
#define TIMESTAMP_IDX 14U
TaskHandle_t udp_client_task_handle;
extern QueueHandle_t rf_queue_handle;
static const char *TAG = "UDP CLIENT:";

typedef struct __attribute__((packed))
{
    int16_t data[DATA_LEN / 2];
    uint32_t timestamp;
} payload_t;

static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb)
{
    uint16_t u = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u;
}

static inline uint32_t conv_timestamp(uint8_t *src)
{
    return ((uint32_t)*(src + TIMESTAMP_IDX) << 24) |
           ((uint32_t)*(src + TIMESTAMP_IDX + 1) << 16) |
           ((uint32_t)*(src + TIMESTAMP_IDX + 2) << 8) |
           (uint32_t)*(src + TIMESTAMP_IDX + 3);
}

static payload_t parse_payload(uint8_t *raw_payload)
{
    payload_t payload;
    for(size_t i = 0; i < DATA_LEN / 2; i++)
    {
        payload.data[i] = conv_to_i16(raw_payload[2 * i], raw_payload[2 * i + 1]);
    }
    payload.timestamp = conv_timestamp(raw_payload);
    return payload;
}

static void udp_client_task(void *pvParameters)
{
    uint8_t payload[PAYLOAD_LEN] = {0};
    payload_t parsed_payload = {0};

    struct sockaddr_in dest_addr =
    {
        .sin_addr.s_addr = inet_addr(HOST_IP_ADDR),
        .sin_family = AF_INET,
        .sin_port = htons(PORT)
    };

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    for(;;)
    {
        if(xQueueReceive(rf_queue_handle, payload, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            parsed_payload = parse_payload(payload);

            int err = sendto(sock, &parsed_payload, sizeof(payload_t), 0,
                            (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            }
            else
            {
                ESP_LOGI(TAG, "Message sent");
            }
        }
        else
        {
            ESP_LOGE(TAG, "No data received from RF module");
            memset(&parsed_payload, 0, (DATA_LEN / 2) * sizeof(int16_t));
            sendto(sock, &parsed_payload, sizeof(payload_t), 0,
                    (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
    vTaskDelete(NULL);
}

void start_udp_client(void) 
{
    BaseType_t pd = xTaskCreate(udp_client_task, "udp_client",
                                4096, NULL, 7, &udp_client_task_handle);
    if(pd != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create udp client task");
    }
}
