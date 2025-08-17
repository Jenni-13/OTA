#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
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
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "esp_https_ota.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


static const char *TAG = "MAIN";


/* ------------- PINOUT ------------- */
#define TRIG_GPIO    GPIO_NUM_15
#define ECHO_GPIO    GPIO_NUM_4
#define LED_GPIO     GPIO_NUM_2
#define SERVO_GPIO   GPIO_NUM_21
#define LDR_ADC_UNIT ADC_UNIT_1
#define LDR_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34


/* ------------- PARAMS ------------- */
#define MIN_DISTANCE_CM 10
#define MAX_DISTANCE_CM 50
#define LDR_DARK_THRESHOLD 3500 // Valor de umbral para considerar "oscuro"
#define APP_VERSION "v1.7"


/* ------------- MQTT / OTA config ------------- */
#define BROKER_URI "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER  "big-data-001"
#define MQTT_PASS  "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID "2022371065"
#define MANIFEST_URL "https://firmware-host.onrender.com/firmware/manifest.json"
#define TOPIC_SENSOR_DATA "esp32/sensor_data"
#define TOPIC_OTA_ALERT "esp32/ota_alert"
#define OTA_MANIFEST_MAX_SIZE 1024


/* ------------- VARIABLES GLOBALES ------------- */
typedef enum {
    STATE_INITIALIZING,
    STATE_RUNNING,
    STATE_OTA_PENDING
} app_state_t;

static app_state_t current_state = STATE_INITIALIZING;
static esp_mqtt_client_handle_t mqtt_client_global = NULL;
static adc_oneshot_unit_handle_t adc_handle;


/* --------- DECLARACIONES DE FUNCIONES DE ESTADO ---------- */
static void run_state_initializing();
static void run_state_running();
static void run_state_ota_pending();


/* --------- DECLARACIONES DE FUNCIONES ---------- */
static esp_err_t start_ota_from_url(const char *url);
static esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt);
static esp_err_t init_servo_pwm(void);
static esp_err_t servo_set_angle(int angle);
static esp_err_t init_adc(void);
static esp_err_t init_sensors(void);
static float measure_distance_cm();
static int read_ldr_value();
static int calculate_angle_by_distance(float distance_cm);


/* --------- FUNCION OTA UPDATE ---------- */
static esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ON_CONNECTED) {
        esp_http_client_set_header(evt->client, "Accept-Encoding", "identity");
    }
    return ESP_OK;
}


static esp_err_t start_ota_from_url(const char *url) {
    ESP_LOGI(TAG, "Iniciando OTA desde URL: %s", url);

    esp_http_client_config_t http_config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = _ota_http_event_handler,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA exitosa, reiniciando...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Fallo OTA, código de error: %s", esp_err_to_name(ret));
    }
    return ret;
}


/* --------- FUNCIONES PARA INICIALIZAR PERIFÉRICOS ---------- */
esp_err_t init_servo_pwm(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if(ret != ESP_OK) return ret;

    ledc_channel_config_t ch_conf = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    return ledc_channel_config(&ch_conf);
}

esp_err_t init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LDR_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12, // Cambio de ADC_ATTEN_DB_11 a ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &config));

    return ESP_OK;
}

esp_err_t init_sensors(void) {
    gpio_reset_pin(TRIG_GPIO);
    gpio_set_direction(TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_GPIO);
    gpio_set_direction(ECHO_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(init_servo_pwm());
    ESP_ERROR_CHECK(init_adc());
    
    ESP_LOGI(TAG, "Perifericos inicializados correctamente");
    return ESP_OK;
}


/* --------- FUNCIONES SECUNDARIAS ---------- */
esp_err_t servo_set_angle(int angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
   
    const int duty_min = 410;
    const int duty_max = 2050;
   
    int duty = duty_min + (int)((float)angle / 180.0f * (duty_max - duty_min));
   
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar duty: %s", esp_err_to_name(ret));
        return ret;
    }
   
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ret;
}

float measure_distance_cm() {
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    int64_t start = esp_timer_get_time();
    int64_t timeout_rise = start + 20000;
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() >= timeout_rise) {
            return -1.0f;
        }
    }
    int64_t t1 = esp_timer_get_time();

    int64_t timeout_fall = t1 + 200000;
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() >= timeout_fall) {
            return -1.0f;
        }
    }
    int64_t t2 = esp_timer_get_time();

    return (float)(t2 - t1) * 0.0343f / 2.0f;
}

static int read_ldr_value() {
    int raw_value;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw_value));
    return raw_value;
}

static int calculate_angle_by_distance(float distance_cm) {
    if(distance_cm < MIN_DISTANCE_CM) return 180;
    if(distance_cm > MAX_DISTANCE_CM) return 0; // Se corrige para que devuelva 0
    return (int)(180 - ((distance_cm - MIN_DISTANCE_CM) * (180.0f/(MAX_DISTANCE_CM - MIN_DISTANCE_CM))));
}


/* --------- MQTT EVENT HANDLER ---------- */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            mqtt_client_global = event->client;
            esp_mqtt_client_subscribe(mqtt_client_global, TOPIC_OTA_ALERT, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT desconectado");
            mqtt_client_global = NULL;
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, TOPIC_OTA_ALERT, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Comando OTA recibido. Transicionando a estado OTA_PENDING.");
                current_state = STATE_OTA_PENDING;
            }
            break;
        default:
            break;
    }
}


/* --------- FUNCIONES QUE IMPLEMENTAN LOS ESTADOS ---------- */
static void run_state_initializing() {
    ESP_LOGI(TAG, "Estado: INICIALIZANDO");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(init_sensors());

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
        .credentials.client_id = MQTT_CLIENT_ID,
        .session.keepalive = 60,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    int timeout = 0;
    while (!mqtt_client_global && timeout < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        timeout++;
    }

    if (mqtt_client_global) {
        ESP_LOGI(TAG, "Transicionando a estado: RUNNING");
        current_state = STATE_RUNNING;
    } else {
        ESP_LOGE(TAG, "Fallo al conectar a MQTT. Reintentando...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


static void run_state_running() {
    float distance_cm = measure_distance_cm();
    int ldr_value = read_ldr_value();
    
    ESP_LOGI(TAG, "Estado: RUNNING | Distancia: %.2f cm | LDR: %d", distance_cm, ldr_value);

    if (ldr_value < LDR_DARK_THRESHOLD && distance_cm > 0 && distance_cm < MAX_DISTANCE_CM) {
        int angulo = calculate_angle_by_distance(distance_cm);
        servo_set_angle(angulo);
        gpio_set_level(LED_GPIO, 1);
        
        if (mqtt_client_global) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "matricula", MQTT_CLIENT_ID);
            cJSON_AddNumberToObject(root, "distancia_cm", distance_cm);
            cJSON_AddNumberToObject(root, "angulo", angulo);
            cJSON_AddStringToObject(root, "estado_led", "on");
            cJSON_AddNumberToObject(root, "ldr", ldr_value);
            char *json_string = cJSON_PrintUnformatted(root);
            esp_mqtt_client_publish(mqtt_client_global, TOPIC_SENSOR_DATA, json_string, 0, 1, 0);
            cJSON_Delete(root);
            free(json_string);
        }
    } else {
        gpio_set_level(LED_GPIO, 0);
        servo_set_angle(0);
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
}


static void run_state_ota_pending() {
    ESP_LOGI(TAG, "Estado: OTA_PENDING | Iniciando la actualizacion del firmware.");
    
    esp_http_client_config_t http_config = {
        .url = MANIFEST_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_config);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            char *json_buffer = malloc(OTA_MANIFEST_MAX_SIZE);
            int read_len = esp_http_client_read(client, json_buffer, OTA_MANIFEST_MAX_SIZE - 1);
            if (read_len > 0) {
                json_buffer[read_len] = '\0';
                cJSON *root = cJSON_Parse(json_buffer);
                if (root) {
                    cJSON *version = cJSON_GetObjectItem(root, "version");
                    cJSON *bin_url = cJSON_GetObjectItem(root, "bin_url");
                    if (version && bin_url && strcmp(version->valuestring, APP_VERSION) != 0) {
                        start_ota_from_url(bin_url->valuestring);
                    }
                    cJSON_Delete(root);
                }
            }
            free(json_buffer);
        }
    }
    
    esp_http_client_cleanup(client);
    
    current_state = STATE_RUNNING;
}


/* --------- MAIN ---------- */
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    while (1) {
        switch (current_state) {
            case STATE_INITIALIZING:
                run_state_initializing();
                break;
            case STATE_RUNNING:
                run_state_running();
                break;
            case STATE_OTA_PENDING:
                run_state_ota_pending();
                break;
            default:
                ESP_LOGE(TAG, "Estado invalido. Reiniciando...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
                break;
        }
    }
}