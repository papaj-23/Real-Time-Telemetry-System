#include <string.h>
#include <sys/param.h>
#include <lwip/netdb.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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

TaskHandle_t udp_client_task_handle;
static const char *TAG = "UDP CLIENT:";
static const char *payload = "Message from ESP32";


static void udp_client_task(void *pvParameters)
{
    //char rx_buffer[128];
    //char host_ip[] = HOST_IP_ADDR;

   for(;;)
   {
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
            break;
        }

        // Set timeout
        struct timeval timeout = {0};
        timeout.tv_sec = 10;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1)
        {

            int err = sendto(sock, payload, strlen(payload), 0,
                            (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            //ESP_LOGI(TAG, "Message sent");

            /*struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }*/

            vTaskDelay(6000 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
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
