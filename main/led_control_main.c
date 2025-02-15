/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.
 *
 * TueTD customization
 */
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ir_nec_encoder.h"
#include "led_strip.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
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

ESP_EVENT_DECLARE_BASE(LED_1_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(LED_1_EVENT_BASE);
ESP_EVENT_DECLARE_BASE(LED_2_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(LED_2_EVENT_BASE);
ESP_EVENT_DECLARE_BASE(LED_3_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(LED_3_EVENT_BASE);
ESP_EVENT_DECLARE_BASE(LED_4_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(LED_4_EVENT_BASE);

ESP_EVENT_DECLARE_BASE(TIMER_EVENT_BASE);
ESP_EVENT_DEFINE_BASE(TIMER_EVENT_BASE);

#define EXAMPLE_IR_RX_GPIO_NUM 19
#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200 // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0 9000
#define NEC_LEADING_CODE_DURATION_1 4500
#define NEC_PAYLOAD_ZERO_DURATION_0 560
#define NEC_PAYLOAD_ZERO_DURATION_1 560
#define NEC_PAYLOAD_ONE_DURATION_0 560
#define NEC_PAYLOAD_ONE_DURATION_1 1690
#define NEC_REPEAT_CODE_DURATION_0 9000
#define NEC_REPEAT_CODE_DURATION_1 2250

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
#define LED_EN_PORT_POWER_1 GPIO_NUM_32
#define LED_EN_PORT_POWER_2 GPIO_NUM_33
#define LED_EN_PORT_POWER_3 GPIO_NUM_25
#define LED_EN_PORT_POWER_4 GPIO_NUM_26
#define LED_EN_PORT_POWER_5 GPIO_NUM_27
#define LED_EN_PORT_POWER_6 GPIO_NUM_14

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

typedef enum { LED_COLOR_OFF, LED_COLOR_RED, LED_COLOR_BLUE } led_color_t;

typedef enum {
    LED_STATUS_ALL_TURN_OFF,
    LED_STATUS_ALL_TURN_ON,
    LED_STATUS_ALL_BLINK,
    LED_STATUS_STEP_ON,
    LED_STATUS_ALL_BLINK_10_SEC,
    LED_STATUS_MAX
} led_event_t;

typedef struct {
    uint32_t led_index;
    bool led_off_on;
    bool led_off_on_10_s;
    led_event_t led_status;
} led_behavior_t;

static const char *TAG = "led_strips";

static const gpio_num_t led_strip_data_pin[LED_STRIP_NUMBER_OF_STRIP] = {
    LED_STRIP_DATA_PIN_1,
    LED_STRIP_DATA_PIN_2,
    LED_STRIP_DATA_PIN_3,
    LED_STRIP_DATA_PIN_4,
};
/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;
static rmt_rx_channel_config_t rx_channel_cfg;
static rmt_channel_handle_t rx_channel = NULL;
static QueueHandle_t receive_queue;
static rmt_receive_config_t receive_config;

typedef enum { TIMER_EVENT_ID_TIMEOUT_500MS, TIMER_EVENT_ID_TIMEOUT_10S } timer_event_id_t;

static led_strip_t led_strip[LED_STRIP_NUMBER_OF_STRIP];
// static QueueHandle_t led_status_queue[LED_STRIP_NUMBER_OF_STRIP]; // Queue handle
static led_behavior_t leds_behavior[LED_STRIP_NUMBER_OF_STRIP];

//
/**/
static esp_event_loop_handle_t event_loop_handle;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration) {
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols) {
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {
    printf("NEC frame start---\r\n");
    for (size_t i = 0; i < symbol_num; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols)) {
            printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols)) {
            printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata,
                                         void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}
///////////////////////////////////////END OF IR CODE////////////////////////////////////////////////////////
///////////////////////////////////////START LED CODE////////////////////////////////////////////////////////

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

    gpio_config_t io_conf = {0};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << LED_EN_PORT_POWER_1) | (1ULL << LED_EN_PORT_POWER_2) |
                           (1ULL << LED_EN_PORT_POWER_3) | (1ULL << LED_EN_PORT_POWER_4) |
                           (1ULL << LED_EN_PORT_POWER_5) | (1ULL << LED_EN_PORT_POWER_6),
    // disable pull-down mode
        io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    // ESP_LOGI(TAG, "Configure port enable :%u", (unsigned int)port_en_pin[port_index]);
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

// Task to blink LED
// void led_1_task(void *pvParameter) {
//
//     while (1) {
//         if (!on_off) {
//             ESP_ERROR_CHECK(led_strip_set_pixel(led_strip[0].led_handle, 0, 255, 0, 0));
//             ESP_ERROR_CHECK(led_strip_refresh(led_strip[0].led_handle));
//             on_off = false;
//         } else {
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
//
// void led_2_task(void *pvParameter) {
//
//     led_event_t status;
//     while (1) {
//         if (xQueueReceive(led_status_queue[1], &status, portMAX_DELAY)) {
//             switch (status) {
//             case LED_STATUS_ALL_TURN_OFF:
//                 ESP_ERROR_CHECK(led_strip_clear(led_strip[1].led_handle));
//                 break;
//             case LED_STATUS_ALL_TURN_ON:
//                 ESP_ERROR_CHECK(led_strip_set_pixel(led_strip[1].led_handle, 0, 255, 0, 0));
//                 ESP_ERROR_CHECK(led_strip_refresh(led_strip[1].led_handle));
//                 break;
//             case LED_STATUS_ALL_BLINK:
//             case LED_STATUS_STEP_ON:
//             case LED_STATUS_ALL_TURN_ON_10_SEC:
//             default:
//                 break;
//             }
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
// void led_3_task(void *pvParameter) {
//
//     bool on_off = false;
//     while (1) {
//         if (!on_off) {
//             ESP_ERROR_CHECK(led_strip_set_pixel(led_strip[0].led_handle, 0, 255, 0, 0));
//             ESP_ERROR_CHECK(led_strip_refresh(led_strip[0].led_handle));
//             on_off = false;
//         } else {
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
// void led_4_task(void *pvParameter) {
//
//     bool on_off = false;
//     while (1) {
//         if (!on_off) {
//             ESP_ERROR_CHECK(led_strip_set_pixel(led_strip[0].led_handle, 0, 255, 0, 0));
//             ESP_ERROR_CHECK(led_strip_refresh(led_strip[0].led_handle));
//             on_off = false;
//         } else {
//         }
//
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
static void led_behavior(esp_event_base_t base, int32_t event_id, led_behavior_t *led_behavior,
                         led_strip_handle_t led_handle) {
    if (base == LED_1_EVENT_BASE && event_id != LED_STATUS_STEP_ON) {
        led_behavior->led_index = 0;
    }
    switch (led_behavior->led_status) {
    case LED_STATUS_ALL_TURN_OFF:
        ESP_ERROR_CHECK(led_strip_clear(led_handle));
        break;
    case LED_STATUS_ALL_TURN_ON:
        ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, 0, 255, 0, 0));
        ESP_ERROR_CHECK(led_strip_refresh(led_handle));
        break;
    case LED_STATUS_ALL_BLINK:
        if (event_id == TIMER_EVENT_ID_TIMEOUT_500MS && base == TIMER_EVENT_BASE) {
            if (led_behavior->led_off_on) {
                // Turn on all led
                for (int i = 0; i < LED_STRIP_NUMBER_OF_STRIP; i++) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, i, 255, 0, 0));
                }
                ESP_ERROR_CHECK(led_strip_refresh(led_handle));
                led_behavior->led_off_on = false;
            } else {
                led_behavior->led_off_on = true;

                ESP_ERROR_CHECK(led_strip_clear(led_handle));
            }
        }
        break;
    case LED_STATUS_STEP_ON:
        if (event_id == TIMER_EVENT_ID_TIMEOUT_500MS && base == TIMER_EVENT_BASE) {
            ESP_ERROR_CHECK(led_strip_clear(led_handle));
            ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, led_behavior->led_index, 255, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_handle));
            led_behavior->led_index++;
        }
        break;
    case LED_STATUS_ALL_BLINK_10_SEC:
        if (base == TIMER_EVENT_BASE) {
            if (event_id == TIMER_EVENT_ID_TIMEOUT_500MS) {
                if (led_behavior->led_off_on_10_s) {
                    // Turn on all led
                    for (int i = 0; i < LED_STRIP_NUMBER_OF_STRIP; i++) {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, i, 255, 0, 0));
                    }
                    ESP_ERROR_CHECK(led_strip_refresh(led_handle));
                    led_behavior->led_off_on_10_s = false;
                } else {
                    led_behavior->led_off_on_10_s = true;
                    ESP_ERROR_CHECK(led_strip_clear(led_handle));
                }
            } else if (event_id == TIMER_EVENT_ID_TIMEOUT_10S) {
                led_behavior->led_off_on_10_s = false;
                ESP_ERROR_CHECK(led_strip_clear(led_handle));
            }
        }
        led_behavior->led_status = LED_STATUS_MAX;
        break;
    // Do nothing
    case LED_STATUS_MAX:
        break;
    default:
        ESP_LOGE(TAG, "Not a valid event :%d", (int)led_behavior->led_status);
        break;
    }
}
/*

*Port 1 - Led strip with RED color
 * */
static void port_1_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_1_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        leds_behavior[0].led_status = event_id;
    }

    led_behavior(base, event_id, &leds_behavior[0], led_strip[0].led_handle);
}
static void port_2_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_2_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        leds_behavior[1].led_status = event_id;
    }

    led_behavior(base, event_id, &leds_behavior[1], led_strip[1].led_handle);
}
static void port_3_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_3_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        leds_behavior[2].led_status = event_id;
    }

    led_behavior(base, event_id, &leds_behavior[2], led_strip[2].led_handle);
}
static void port_4_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_4_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        leds_behavior[3].led_status = event_id;
    }

    led_behavior(base, event_id, &leds_behavior[3], led_strip[3].led_handle);
}
static void port_5_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
static void port_6_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {}
// Timer callback function
static void timer_callback(void *arg) {
    // printf("Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, TIMER_EVENT_BASE, TIMER_EVENT_ID_TIMEOUT_500MS, NULL, 0,
                      portMAX_DELAY);
}
static void configure_ir(void) {
    ESP_LOGI(TAG, "create RMT RX channel");

    rx_channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_channel_cfg.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    rx_channel_cfg.mem_block_symbols = 64; // amount of RMT symbols that the channel can store at a time
    rx_channel_cfg.gpio_num = EXAMPLE_IR_RX_GPIO_NUM;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if (receive_queue == NULL) {
        ESP_LOGE(TAG, "Error when create queue");
    }
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol

    receive_config.signal_range_min_ns = 1250;     // the shortest duration for NEC signal is 560us, 1250ns <
                                                   // 560us, valid signal won't be treated as noise
    receive_config.signal_range_max_ns = 12000000; // the longest duration for NEC signal is 9000us,
                                                   // 12000000ns > 9000us, the receive won't stop early

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = EXAMPLE_IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
}

static void nvs_read() {

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
        switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("Restart counter = %" PRIu32 "\n", restart_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        // printf("Updating restart counter in NVS ... ");
        // restart_counter++;
        // err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
        // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        //
        // // Commit written value.
        // // After setting any values, nvs_commit() must be called to ensure changes are written
        // // to flash storage. Implementations may write to storage at other times,
        // // but this is not guaranteed.
        // printf("Committing updates in NVS ... ");
        // err = nvs_commit(my_handle);
        // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        //
        // // Close
        nvs_close(my_handle);
    }
}
void app_main(void) {

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;

    configure_port_en();
    configure_led();
    configure_ir();

    nvs_read();
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
    // Create a timer
    const esp_timer_create_args_t timer_args = {.callback = &timer_callback, .name = "timer_500ms"};

    esp_timer_handle_t timer_handle;
    esp_timer_create(&timer_args, &timer_handle);
    // Start the timer (fires every 2 seconds)
    esp_timer_start_periodic(timer_handle, 5000); // Time in microseconds
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    while (1) {
        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            // parse the receive symbols and print the result
            example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } else {
            // timeout, transmit predefined IR NEC packets
            // const ir_nec_scan_code_t scan_code = {
            //     .address = 0x0440,
            //     .command = 0x3003,
            // };
            // ESP_ERROR_CHECK(
            //     rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
        }
    }
}
