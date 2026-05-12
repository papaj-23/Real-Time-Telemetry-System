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
#define PAYLOAD_LEN 14U

TaskHandle_t udp_client_task_handle;
extern QueueHandle_t rf_queue_handle;
static const char *TAG = "UDP CLIENT:";

static inline int16_t conv_to_i16(uint8_t msb, uint8_t lsb) {
    uint16_t u = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)u;
}

static void parse_payload(const uint8_t *raw_payload, int16_t *dest)
{
    for (size_t i = 0; i < PAYLOAD_LEN / 2; i++)
    {
        dest[i] = conv_to_i16(raw_payload[2*i], raw_payload[2*i + 1]);
    }
}

static void udp_client_task(void *pvParameters)
{
    uint8_t payload[PAYLOAD_LEN] = {0};
    int16_t parsed_payload[PAYLOAD_LEN / 2] = {0};

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
        if(xQueueReceive(rf_queue_handle, payload, pdMS_TO_TICKS(100)) == pdPASS)
        {
            parse_payload(payload, parsed_payload);

            int err = sendto(sock, parsed_payload, PAYLOAD_LEN / 2 * sizeof(int16_t), 0,
                            (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                close(sock);
                vTaskDelete(NULL);
            }
            ESP_LOGI(TAG, "Message sent");
        }
        else
        {
            ESP_LOGI(TAG, "No data received from RF module");
        }
    }
    vTaskDelete(NULL);
}

void start_udp_client(void) 
{
    BaseType_t pd = xTaskCreate(udp_client_task, "udp_client",
                                4096, NULL, 6, &udp_client_task_handle);
    if(pd != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create udp client task");
    }
}
