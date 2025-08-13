/* main.c - Integración: sensor ultrasónico + servo + LED + MQTTs + OTA
   Compilar con ESP-IDF (idf.py build)
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h" // example_connect()
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN_PROYECTO";

/* ------------- PINOUT ------------- */
#define TRIG_GPIO    GPIO_NUM_15
#define ECHO_GPIO    GPIO_NUM_4
#define LED_GPIO     GPIO_NUM_2
#define SERVO_GPIO   GPIO_NUM_18

/* ------------- PARAMS ------------- */
#define MIN_DISTANCE 10
#define MAX_DISTANCE 50

/* ------------- MQTT / OTA config (ajusta según tu entorno) ------------- */
#define BROKER_URI "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER  "big-data-001"
#define MQTT_PASS  "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID "2022371065" 
#define APP_VERSION "v1.3"

/* Si quieres OTA automática al inicio, coloca aquí la URL del .bin accesible por HTTP/HTTPS.
   Si la dejas vacía (""), no hará OTA al inicio. */
#define OTA_URL_DEFAULT "http://192.168.100.60:8000/firmware/esp32_ota_firmware_v1.3.bin"
#define MANIFEST_URL "https://firmware-host-8vrxf29xb-jennifers-projects-f3205073.vercel.app/manifest.json"


/* CA root PEM del broker (REEMPLAZA con el certificado real del broker) */
static const char mqtt_root_cert_pem[] = "-----BEGIN CERTIFICATE-----\n"
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
"-----END CERTIFICATE-----\n";


/* ------------- VARIABLES GLOBALES ------------- */
static esp_mqtt_client_handle_t mqtt_client_global = NULL;
static char ota_url[256] = OTA_URL_DEFAULT;

/* --------- FUNCIONES: SERVO (LEDC) ---------- */
void servo_set_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    const ledc_timer_bit_t duty_resolution = LEDC_TIMER_13_BIT;
    const int max_duty = (1 << duty_resolution) - 1;
    float duty_min = 0.025f, duty_max = 0.125f;
    float duty = duty_min + ((float)angle / 180.0f) * (duty_max - duty_min);
    int duty_count = (int)(duty * max_duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_count);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void init_servo_pwm(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

/* --------- FUNCIONES: ULTRASÓNICO ---------- */
float measure_distance_cm() {
    // trigger pulse
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    int64_t start = esp_timer_get_time();
    int64_t timeout = start + 200000; // 200ms
    while (gpio_get_level(ECHO_GPIO) == 0 && esp_timer_get_time() < timeout) {}
    if (esp_timer_get_time() >= timeout) return -1.0f;

    int64_t t1 = esp_timer_get_time();
    timeout = t1 + 200000;
    while (gpio_get_level(ECHO_GPIO) == 1 && esp_timer_get_time() < timeout) {}
    int64_t t2 = esp_timer_get_time();
    if (esp_timer_get_time() >= timeout) return -1.0f;

    float distance = (t2 - t1) * 0.0343f / 2.0f; // velocidad sonido cm/us
    return distance;
}

/* --------- FUNCIONES: OTA (descarga y escribe partición) ---------- */
static void ota_task_from_url(void *pvParameter) {
    const char *url = (const char *)pvParameter;
    ESP_LOGI(TAG, "OTA: iniciando descarga desde: %s", url);


    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "OTA: fallo init http client");
        vTaskDelete(NULL);
        return;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG, "OTA: no se pudo abrir conexión HTTP");
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "OTA: HTTP status %d content_length %d", status_code, content_length);

    if (status_code != 200) {
        ESP_LOGE(TAG, "OTA: HTTP GET fallo status=%d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "OTA: no hay particion de actualizacion");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    esp_ota_handle_t update_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_begin fallo %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    char *buffer = malloc(1024);
    if (!buffer) {
        ESP_LOGE(TAG, "OTA: malloc buffer fallo");
        esp_ota_abort(update_handle);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }

    int binary_file_length = 0;
    while (1) {
        int data_read = esp_http_client_read(client, buffer, 1024);
        if (data_read < 0) {
            ESP_LOGE(TAG, "OTA: error lectura datos");
            break;
        } else if (data_read > 0) {
            err = esp_ota_write(update_handle, (const void *)buffer, data_read);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA: esp_ota_write fallo %s", esp_err_to_name(err));
                break;
            }
            binary_file_length += data_read;
        } else { // data_read == 0 -> fin
            ESP_LOGI(TAG, "OTA: descarga completa");
            break;
        }
    }

    free(buffer);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (binary_file_length > 0) {
        err = esp_ota_end(update_handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(update_partition);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "OTA: Exito, datos escritos=%d, reiniciando...", binary_file_length);
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            } else {
                ESP_LOGE(TAG, "OTA: esp_ota_set_boot_partition fallo %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "OTA: esp_ota_end fallo %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "OTA: no se recibieron datos");
        esp_ota_abort(update_handle);
    }

    vTaskDelete(NULL);
}

static void start_ota_from_url(const char *url) {
    if (!url || strlen(url) == 0) {
        ESP_LOGW(TAG, "OTA: URL vacia");
        return;
    }
    // pasar una copia porque el task usa el puntero al parámetro
    char *url_copy = strdup(url);
    xTaskCreate(ota_task_from_url, "ota_task", 8192, url_copy, 5, NULL);
}

/* --------- MQTT EVENT HANDLER ---------- */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT: conectado");
        mqtt_client_global = client;
        esp_mqtt_client_subscribe(client, "/ota/update", 1);
        esp_mqtt_client_subscribe(client, "/actuador/led", 1);
        esp_mqtt_client_subscribe(client, "/actuador/servo", 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT: desconectado");
        mqtt_client_global = NULL;
        break;

    case MQTT_EVENT_DATA: {
        char topic[128];
        char data[256];
        snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
        snprintf(data, sizeof(data), "%.*s", event->data_len, event->data);
        ESP_LOGI(TAG, "MQTT: topic=%s message=%s", topic, data);

        if (strcmp(topic, "/ota/update") == 0) {
            // data contiene la URL (http/https)
            start_ota_from_url(data);
        } else if (strcmp(topic, "/actuador/led") == 0) {
            if (strcasecmp(data, "ON") == 0) gpio_set_level(LED_GPIO, 1);
            else if (strcasecmp(data, "OFF") == 0) gpio_set_level(LED_GPIO, 0);
        } else if (strcmp(topic, "/actuador/servo") == 0) {
            int angle = atoi(data);
            servo_set_angle(angle);
        }
        break;
    }

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT: suscripción OK id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT: mensaje publicado id=%d", event->msg_id);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT: error");
        break;

    default:
        break;
    }
}

/* --------- INICIO MQTT CLIENT ---------- */
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = BROKER_URI,
            .verification.certificate = mqtt_root_cert_pem
        },
        .credentials = {
            .username = MQTT_USER,
            .authentication.password = MQTT_PASS,
            .client_id = MQTT_CLIENT_ID
        }
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

/* --------- TAREA: LECTURA SENSOR Y PUBLICACION ---------- */
void sensor_publish_task(void *pvParameters) {
    while (1) {
        float distance = measure_distance_cm();
        int led_state = (distance > 0 && distance < MIN_DISTANCE) ? 1 : 0;
        gpio_set_level(LED_GPIO, led_state);

        int angle;
        if (distance < 0) angle = 0;
        else if (distance > MAX_DISTANCE) angle = 180;
        else angle = (int)((distance - MIN_DISTANCE) * 180.0 / (MAX_DISTANCE - MIN_DISTANCE));
        servo_set_angle(angle);

        char payload[128];
        if (distance < 0) snprintf(payload, sizeof(payload), "{\"distancia\":null,\"angulo\":%d,\"led\":%d}", angle, led_state);
        else snprintf(payload, sizeof(payload), "{\"distancia\":%.2f,\"angulo\":%d,\"led\":%d}", distance, angle, led_state);

        if (mqtt_client_global) {
            esp_mqtt_client_publish(mqtt_client_global, "/idgs11/2022371065", payload, 0, 1, 0);
        }

        ESP_LOGI(TAG, "Publicando: %s", payload);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* --------- APP MAIN ---------- */
void app_main(void) {
      ESP_LOGI(TAG, "Firmware version: %s", APP_VERSION);
    ESP_LOGI(TAG, "Arrancando app");
   
    
    ESP_LOGI(TAG, "Arrancando app");

    // NVS, netif, event loop
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Conectar a WiFi (usa menuconfig o ajusta en protocol_examples_common)
    ESP_LOGI(TAG, "Conectando a WiFi...");
    ESP_ERROR_CHECK(example_connect());

    // Inicializar hardware
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TRIG_GPIO);
    gpio_reset_pin(ECHO_GPIO);
    gpio_set_direction(TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_GPIO, GPIO_MODE_INPUT);
    init_servo_pwm();

    // Iniciar MQTT (TLS)
    mqtt_app_start();

    // Si OTA_URL_DEFAULT no esta vacía, iniciar comprobación automática (una sola vez)
    if (strlen(ota_url) > 0) {
        ESP_LOGI(TAG, "OTA automático habilitado. URL: %s", ota_url);
        start_ota_from_url(ota_url);
    }

    // Crear tarea de sensor + publicación MQTT
    xTaskCreate(sensor_publish_task, "sensor_pub", 4096, NULL, 5, NULL);
}
