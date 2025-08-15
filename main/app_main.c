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

static const char *TAG = "MAIN";

/* ------------- PINOUT ------------- */
#define TRIG_GPIO    GPIO_NUM_15
#define ECHO_GPIO    GPIO_NUM_4
#define LED_GPIO     GPIO_NUM_2
#define SERVO_GPIO   GPIO_NUM_21  // Cambiado a GPIO 21 para evitar conflictos

/* ------------- PARAMS ------------- */
#define MIN_DISTANCE 10
#define MAX_DISTANCE 50

/* ------------- MQTT / OTA config ------------- */
#define BROKER_URI "mqtts://l46d1e5e.ala.us-east-1.emqxsl.com:8883"
#define MQTT_USER  "big-data-001"
#define MQTT_PASS  "1Q2W3E4R5T6Y"
#define MQTT_CLIENT_ID "2022371065"
#define APP_VERSION "v1.7"
#define MANIFEST_URL "https://firmware-dzt70qwej-jennifers-projects-f3205073.vercel.app/manifest.json"

#define TOPIC_SENSOR_DATA "esp32/sensor_data"
#define TOPIC_OTA_ALERT "esp32/ota_alert"


/* ------------- VARIABLES GLOBALES ------------- */
static esp_mqtt_client_handle_t mqtt_client_global = NULL;
static bool ota_update_detected = false;

/* --------- DECLARACIONES DE FUNCIONES ---------- */
static void start_ota_from_url(const char *url);
static void sensor_task(void *pvParameter);
/* --------- FUNCION OTA UPDATE ---------- */
static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            break;
        case HTTP_EVENT_ON_DATA:
            break;
        case HTTP_EVENT_ON_FINISH:
            break;
        case HTTP_EVENT_DISCONNECTED:
            break;
        case HTTP_EVENT_REDIRECT:
            break;
    }
    return ESP_OK;
}

static void start_ota_from_url(const char *url) {
    ESP_LOGI(TAG, "Iniciando OTA desde URL: %s", url);
    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = _http_event_handler,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA exitosa, reiniciando...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Fallo OTA, código de error: %s", esp_err_to_name(ret));
    }
}
/* --------- FUNCIONES SERVO ---------- */
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

esp_err_t servo_set_angle(int angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    
    // Mapeo de ángulo a duty cycle (valores típicos para SG90)
    // 0° -> 500 us (2.5% de 20ms)
    // 180° -> 2500 us (12.5% de 20ms)
    const int duty_min = 410; // 500us / (1000000us / 50Hz) * 8192 = 410
    const int duty_max = 2050; // 2500us / (1000000us / 50Hz) * 8192 = 2050
    
    int duty = duty_min + (int)((float)angle / 180.0f * (duty_max - duty_min));
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ret;
}

/* --------- FUNCIONES: ULTRASÓNICO ---------- */
float measure_distance_cm() {
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    int64_t start = esp_timer_get_time();
    int64_t timeout_rise = start + 20000; // 20ms timeout para flanco de subida

    // Espera flanco de subida (ECHO == 1)
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() >= timeout_rise) {
            ESP_LOGW(TAG, "Timeout esperando flanco de subida");
            return -1.0f; // Retorna un valor de error
        }
    }
    int64_t t1 = esp_timer_get_time();

    // Espera flanco de bajada (ECHO == 0)
    int64_t timeout_fall = t1 + 200000; // 200ms timeout para flanco de bajada
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() >= timeout_fall) {
            ESP_LOGW(TAG, "Timeout esperando flanco de bajada");
            return -1.0f;
        }
    }
    int64_t t2 = esp_timer_get_time();

    return (float)(t2 - t1) * 0.0343f / 2.0f;
}

/* --------- INICIALIZACIÓN DE SENSORES ---------- */
esp_err_t init_sensors(void) {
    gpio_reset_pin(TRIG_GPIO);
    gpio_set_direction(TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_GPIO);
    gpio_set_direction(ECHO_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t ret = init_servo_pwm();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar servo PWM: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Sensores inicializados correctamente");
    return ESP_OK;
}


/* --------- FUNCIÓN PARA CALCULAR ÁNGULO ---------- */
static int calcular_angulo_por_distancia(float distancia_cm) {
    if(distancia_cm < MIN_DISTANCE) return 180;
    if(distancia_cm > MAX_DISTANCE) return 0;
    return (int)(180 - ((distancia_cm - MIN_DISTANCE) * (180.0f/(MAX_DISTANCE-MIN_DISTANCE))));
}

/* --------- TAREA PARA LA LECTURA DE SENSORES Y ENVÍO MQTT ---------- */
static void sensor_task(void *pvParameter) {
    init_sensors();
    while (1) {
        float distancia_cm = measure_distance_cm();
        
        if (distancia_cm > 0) {
            ESP_LOGI(TAG, "Distancia medida: %.2f cm", distancia_cm);

            if (distancia_cm < MAX_DISTANCE) {
                // Objeto detectado: mover servo, encender LED y enviar datos
                int angulo = calcular_angulo_por_distancia(distancia_cm);
                servo_set_angle(angulo);
                gpio_set_level(LED_GPIO, 1);
                
                if (mqtt_client_global) {
                    cJSON *root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "matricula", MQTT_CLIENT_ID);
                    cJSON_AddNumberToObject(root, "distancia_cm", distancia_cm);
                    cJSON_AddNumberToObject(root, "angulo", angulo);
                    cJSON_AddStringToObject(root, "estado_led", "encendido");
                    
                    char *json_string = cJSON_PrintUnformatted(root);
                    
                    int msg_id = esp_mqtt_client_publish(mqtt_client_global, TOPIC_SENSOR_DATA, json_string, 0, 1, 0);
                    if (msg_id < 0) {
                        ESP_LOGE(TAG, "Error al publicar datos del sensor (Código: %d)", msg_id);
                    } else {
                        ESP_LOGI(TAG, "Datos del sensor publicados en MQTT, ID: %d", msg_id);
                    }
                    
                    cJSON_Delete(root);
                    free(json_string);
                } else {
                    ESP_LOGW(TAG, "MQTT no disponible para enviar datos del sensor");
                }
            } else {
                // No hay objeto cerca: apagar LED y volver a la posición inicial del servo
                gpio_set_level(LED_GPIO, 0);
                servo_set_angle(0);
                
                // Opcionalmente, puedes publicar un mensaje de "estado normal"
                // No se enviarán datos si el sensor no detecta nada, como lo pediste
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Espera entre mediciones
    }
}
/* --------- MQTT EVENT HANDLER ---------- */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                             int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            mqtt_client_global = event->client;
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT desconectado");
            mqtt_client_global = NULL;
            break;
            
        default:
            break;
    }
}
/* --------- OTA TASK ---------- */
static void ota_check_task(void *pvParameter) {
    while (1) {
        ESP_LOGI(TAG, "Comprobando actualizaciones...");
        
        // 1. Configuración mejorada del cliente HTTP
        esp_http_client_config_t config = {
            .url = MANIFEST_URL,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .buffer_size = 2048,
            .buffer_size_tx = 512,
            .timeout_ms = 10000,
            .disable_auto_redirect = false,
            .max_redirection_count = 5,
        };
        
        esp_http_client_handle_t client = esp_http_client_init(&config);
        
        // 2. Headers adicionales para evitar caché
        esp_http_client_set_header(client, "Cache-Control", "no-cache");
        esp_http_client_set_header(client, "Pragma", "no-cache");
        
        // 3. Realizar petición
        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error al abrir conexión: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }
        
        // 4. Obtener información de la respuesta
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP Status: %d", status_code);
        
        if (status_code == 200) {
            // 5. Leer contenido
            char *buffer = malloc(512);
            if (!buffer) {
                ESP_LOGE(TAG, "Error al asignar memoria");
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                vTaskDelay(pdMS_TO_TICKS(60000));
                continue;
            }
            
            int read_len = esp_http_client_read(client, buffer, 511);
            if (read_len > 0) {
                buffer[read_len] = '\0';
                ESP_LOGI(TAG, "Contenido recibido (%d bytes): %.*s", 
                       read_len, 100, buffer); // Muestra primeros 100 caracteres
                
                // 6. Procesar JSON
                cJSON *root = cJSON_Parse(buffer);
                if (root) {
                    cJSON *version = cJSON_GetObjectItem(root, "version");
                    cJSON *bin_url = cJSON_GetObjectItem(root, "bin_url");
                    
                    if (version && bin_url) {
                        ESP_LOGI(TAG, "Versión remota: %s", version->valuestring);
                        
                        if (strcmp(version->valuestring, APP_VERSION) != 0) {
                            ESP_LOGI(TAG, "Nueva versión disponible!");
                            start_ota_from_url(bin_url->valuestring);
                        }
                    }
                    cJSON_Delete(root);
                } else {
                    ESP_LOGE(TAG, "Error al parsear JSON");
                }
            } else {
                ESP_LOGE(TAG, "Error al leer contenido (leídos: %d bytes)", read_len);
            }
            free(buffer);
        }
        
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
/* --------- MAIN ---------- */
void app_main(void) {
    // Inicialización básica
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Conexión WiFi
    ESP_ERROR_CHECK(example_connect());

    // Configuración MQTT
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

    // Espera conexión MQTT
    int timeout = 0;
    while (!mqtt_client_global && timeout < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        timeout++;
    }

    if (!mqtt_client_global) {
        ESP_LOGE(TAG, "No se pudo conectar a MQTT. La lógica del sensor no se ejecutará.");
    } else {
        // Se crea la tarea de OTA, la cual, al detectar una actualización,
        // establecerá una bandera y luego se eliminará.
        xTaskCreate(ota_check_task, "ota_check", 8192, NULL, 5, NULL);

        // Se espera a que la tarea de OTA detecte una actualización
        while (!ota_update_detected) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Una vez que se detecta la actualización, se inicia la tarea del sensor
        ESP_LOGI(TAG, "Actualización detectada. Iniciando la tarea del sensor.");
        xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);
    }
}
