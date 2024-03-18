#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define EXAMPLE_ESP_WIFI_SSID      "123"           // 名称
#define EXAMPLE_ESP_WIFI_PASS      "10101010"      // Wi-Fi密码
#define EXAMPLE_ESP_MAXIMUM_RETRY  5               // 最大重试次数

static EventGroupHandle_t s_wifi_event_group;      // Wi-Fi事件组句柄

#define WIFI_FAIL_BIT      BIT1                    // Wi-Fi连接失败
#define WIFI_CONNECTED_BIT BIT0                    // Wi-Fi连接成功

static const char *TAG = "wifi station";          // 日志标签

static int s_retry_num = 0;                       // Wi-Fi连接重试次数

// Wi-Fi事件处理函数
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();                         // Wi-Fi站点模式启动后尝试连接
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();                     // Wi-Fi连接失败时重新连接
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // 达到最大重试次数时设置连接失败标志位
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // 获取到IP地址后打印日志
        s_retry_num = 0;                          // 重试次数重置为0
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // 设置连接成功标志位
    }
}

// Wi-Fi初始化函数
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // 创建Wi-Fi事件组

    ESP_ERROR_CHECK(esp_netif_init());       // 初始化网络接口
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 创建事件循环
    esp_netif_create_default_wifi_sta();      // 创建默认的Wi-Fi站点

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // 默认的Wi-Fi初始化配置
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));     // 初始化Wi-Fi

    // 注册Wi-Fi事件处理函数
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // 配置Wi-Fi参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_HUNT_AND_PECK,
            .sae_h2e_identifier = "123123123",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) ); // 设置Wi-Fi工作模式为站点模式
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) ); // 设置Wi-Fi配置
    ESP_ERROR_CHECK(esp_wifi_start() );         // 启动Wi-Fi

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // 等待连接成功或失败
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // 根据连接结果打印相应的日志信息
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


void app_main(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA"); // 打印日志
    wifi_init_sta(); // 初始化Wi-Fi站点模式
}
