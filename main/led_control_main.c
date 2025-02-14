/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.
 *
 * TueTD customization
 */
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include <stdio.h>

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)
// Number of port has LED
#define LED_STRIP_NUMBER_OF_STRIP 4
// Number of port
#define PORT_NUMBER 6

ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_1_BASE);
ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_2_BASE);
ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_3_BASE);
ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_1_BASE);
ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_2_BASE);
ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_3_BASE);
/*
 * Maximum numbers of the LED in the strip
 * Adjust this if you want to increase the number of led in strip
 */
#define LED_STRIP_LED_COUNT 10
/*
 * ESP pins that drive led strip data control pin
 * MCU can control LED brightness by these pins
 * MCU IO level HIGH -> Output 5V
 * MCO IO Level LOW -> Output 0
 */
#define LED_STRIP_DATA_PIN_1 5
#define LED_STRIP_DATA_PIN_2 17
#define LED_STRIP_DATA_PIN_3 16
#define LED_STRIP_DATA_PIN_4 4
/*
 * ESP pins that control power of each port
 * MCU IO level HIGH -> Output 5V
 * MCO IO Level LOW -> Output 0
 */
#define LED_EN_PORT_POWER_1 32
#define LED_EN_PORT_POWER_2 33
#define LED_EN_PORT_POWER_3 25
#define LED_EN_PORT_POWER_4 26
#define LED_EN_PORT_POWER_5 27
#define LED_EN_PORT_POWER_6 14

#define PORT_ON 1
#define PORT_OFF 0
// Define button event types
typedef enum {
    BUTTON_EVENT_0,
    BUTTON_EVENT_1,
    BUTTON_EVENT_2,
    BUTTON_EVENT_3,
    BUTTON_EVENT_4,
    BUTTON_EVENT_5,
    BUTTON_EVENT_6,
    BUTTON_EVENT_7,
    BUTTON_EVENT_8
} button_event_t;

typedef struct led_strip_t {
    led_strip_handle_t led_handle;
    led_strip_config_t led_config;
    uint32_t led_power_en;
} led_strip_t;

static const char *TAG = "led_strips";

static const gpio_num_t led_strip_data_pin[LED_STRIP_NUMBER_OF_STRIP] = {
    LED_STRIP_DATA_PIN_1,
    LED_STRIP_DATA_PIN_2,
    LED_STRIP_DATA_PIN_3,
    LED_STRIP_DATA_PIN_4,
};
static const gpio_num_t port_en_pin[PORT_NUMBER] = {
    LED_EN_PORT_POWER_1, LED_EN_PORT_POWER_2, LED_EN_PORT_POWER_3,
    LED_EN_PORT_POWER_4, LED_EN_PORT_POWER_5, LED_EN_PORT_POWER_6,
};

led_strip_t led_strip[LED_STRIP_NUMBER_OF_STRIP];

/**/
static esp_event_loop_handle_t event_loop_handle;

/*
 * @brief initialization the RGB Led pin and RMT peripheral drivers
 * */
void configure_led(void) {
    for (uint32_t strip_index = 0; strip_index < (sizeof(led_strip) / sizeof(led_strip[0])); strip_index++) {
        led_strip[strip_index].led_config.strip_gpio_num = led_strip_data_pin[strip_index];
        led_strip[strip_index].led_config.max_leds = LED_STRIP_LED_COUNT;
        led_strip[strip_index].led_config.led_model = LED_MODEL_WS2812;
        led_strip[strip_index].led_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        led_strip[strip_index].led_config.flags.invert_out = false;
    }

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,               // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }};

    for (int strip_index = 0; strip_index < (sizeof(led_strip) / sizeof(led_strip[0])); strip_index++) {
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip[strip_index].led_config, &rmt_config,
                                                 &led_strip[strip_index].led_handle));
    }
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
}

void configure_port_en(void) {
    for (int port_index = 0; port_index < (sizeof(port_en_pin) / sizeof(port_en_pin[0])); port_index++) {
        gpio_config_t io_conf = {0};
        // disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = port_en_pin[port_index];
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // disable pull-up mode
        io_conf.pull_up_en = 0;
        // configure GPIO with the given settings
        gpio_config(&io_conf);
        ESP_LOGI(TAG, "Configure port enable :%u", (unsigned int)port_en_pin[port_index]);
    }
}
/*
 * Control the OUTPUT value of port
 * port_number: PORT number from 1 -> 6
 * level: 0 -> Output 0V
 * level: =! 0 -> Output 5V
 * */
static inline esp_err_t port_en(gpio_num_t port_number, uint32_t level) {
    return gpio_set_level(port_number, level);
}

static void port_1_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == BUTTON_PRESS_1_BASE) {
        switch (event_id) {
        case BUTTON_EVENT_0:
            port_en(port_en_pin[0], PORT_ON);
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip[0].led_handle, 0, 255, 0, 0));
            break;
        }
    }
}
static void port_2_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
static void port_3_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
static void port_4_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
static void port_5_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
static void port_6_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}

void app_main(void) {
    configure_port_en();
    configure_led();
    esp_event_loop_args_t loop_args = {.queue_size = 10,
                                       .task_name = "event_task",
                                       .task_priority = uxTaskPriorityGet(NULL),
                                       .task_stack_size = 4096,
                                       .task_core_id = tskNO_AFFINITY};
    esp_event_loop_create(&loop_args, &event_loop_handle);

    // Register event handler
    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                             port_1_event_handler, NULL, NULL);
    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,

                                             port_2_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                             port_3_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                             port_4_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                             port_5_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                             port_6_event_handler, NULL, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
