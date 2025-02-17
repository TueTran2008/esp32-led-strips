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
// #include "ir_nec_encoder.h"
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

// ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_1_BASE);
// ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_2_BASE);
// ESP_EVENT_DECLARE_BASE(BUTTON_PRESS_3_BASE);
// ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_1_BASE);
// ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_2_BASE);
// ESP_EVENT_DEFINE_BASE(BUTTON_PRESS_3_BASE);

/**
 * @brief Base event for LED 1.
 */
ESP_EVENT_DECLARE_BASE(LED_1_EVENT_BASE);

/**
 * @brief Definition of the base event for LED 1.
 */
ESP_EVENT_DEFINE_BASE(LED_1_EVENT_BASE);

/**
 * @brief Base event for LED 2.
 */
ESP_EVENT_DECLARE_BASE(LED_2_EVENT_BASE);

/**
 * @brief Definition of the base event for LED 2.
 */
ESP_EVENT_DEFINE_BASE(LED_2_EVENT_BASE);

/**
 * @brief Base event for LED 3.
 */
ESP_EVENT_DECLARE_BASE(LED_3_EVENT_BASE);

/**
 * @brief Definition of the base event for LED 3.
 */
ESP_EVENT_DEFINE_BASE(LED_3_EVENT_BASE);

/**
 * @brief Base event for LED 4.
 */
ESP_EVENT_DECLARE_BASE(LED_4_EVENT_BASE);

/**
 * @brief Definition of the base event for LED 4.
 */
ESP_EVENT_DEFINE_BASE(LED_4_EVENT_BASE);

/**
 * @brief Base event for timer events.
 */
ESP_EVENT_DECLARE_BASE(TIMER_1_EVENT_BASE);

/**
 * @brief Definition of the base event for timer events.
 */
ESP_EVENT_DEFINE_BASE(TIMER_1_EVENT_BASE);

/**
 * @brief Base event for timer events.
 */
ESP_EVENT_DECLARE_BASE(TIMER_2_EVENT_BASE);

/**
 * @brief Definition of the base event for timer events.
 */
ESP_EVENT_DEFINE_BASE(TIMER_2_EVENT_BASE);

/**
 * @brief Base event for timer events.
 */
ESP_EVENT_DECLARE_BASE(TIMER_3_EVENT_BASE);
/**
 * @brief Definition of the base event for timer events.
 */
ESP_EVENT_DEFINE_BASE(TIMER_3_EVENT_BASE);

/**
 * @brief Base event for timer events.
 */
ESP_EVENT_DECLARE_BASE(TIMER_4_EVENT_BASE);

/**
 * @brief Definition of the base event for timer events.
 */
ESP_EVENT_DEFINE_BASE(TIMER_4_EVENT_BASE);

#define TIMER_EVENT_BASE(port) TIMER_##port##_EVENT_BASE

#define EXAMPLE_IR_RX_GPIO_NUM 19
#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200 // Tolerance for parsing RMT symbols into bit stream

#define BUTTON_NUMBER 9
#define BUTTON_PRESS_MAX_TIMES 3
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

/**
 * @file led_control_main.c
 * @brief This file contains the definitions for the PORT_ON and PORT_OFF constants.
 */

#define PORT_ON 1  /**< Constant representing the state of a port being turned on. */
#define PORT_OFF 0 /**< Constant representing the state of a port being turned off. */

/**
 * @file led_control_main.c
 * @brief This file contains the definitions of button event types for IR decoding.
 */

/**
 * @def BUTTON_NEC_CODE_ON_OFF
 * @brief Button event type for the "ON/OFF" button.
 */

/**
 * @def BUTTON_NEC_CODE_1
 * @brief Button event type for the "1" button.
 */

/**
 * @def BUTTON_NEC_CODE_2
 * @brief Button event type for the "2" button.
 */

/**
 * @def BUTTON_NEC_CODE_3
 * @brief Button event type for the "3" button.
 */

/**
 * @def BUTTON_NEC_CODE_4
 * @brief Button event type for the "4" button.
 */

/**
 * @def BUTTON_NEC_CODE_5
 * @brief Button event type for the "5" button.
 */

/**
 * @def BUTTON_NEC_CODE_6
 * @brief Button event type for the "6" button.
 */

/**
 * @def BUTTON_NEC_CODE_7
 * @brief Button event type for the "7" button.
 */

/**
 * @def BUTTON_NEC_CODE_8
 * @brief Button event type for the "8" button.
 */

#define BUTTON_NEC_CODE_ON_OFF 1
#define BUTTON_NEC_CODE_1 2
#define BUTTON_NEC_CODE_2 3
#define BUTTON_NEC_CODE_3 4
#define BUTTON_NEC_CODE_4 5
#define BUTTON_NEC_CODE_5 6
#define BUTTON_NEC_CODE_6 7
#define BUTTON_NEC_CODE_7 8
#define BUTTON_NEC_CODE_8 9

/**
 * @brief Enumeration representing different button events.
 */
typedef enum {
    BUTTON_EVENT_0, /**< Button event 0 */
    BUTTON_EVENT_1, /**< Button event 1 */
    BUTTON_EVENT_2, /**< Button event 2 */
    BUTTON_EVENT_3, /**< Button event 3 */
    BUTTON_EVENT_4, /**< Button event 4 */
    BUTTON_EVENT_5, /**< Button event 5 */
    BUTTON_EVENT_6, /**< Button event 6 */
    BUTTON_EVENT_7, /**< Button event 7 */
    BUTTON_EVENT_8  /**< Button event 8 */
} button_event_t;

/**
 * @brief Structure representing an LED strip.
 */
typedef struct led_strip_t {
    led_strip_handle_t led_handle; /**< Handle for the LED strip. */
    led_strip_config_t led_config; /**< Configuration for the LED strip. */
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

typedef struct {
    uint16_t address;
    uint16_t command;
} nec_code_t;

typedef union {
    struct _status {
        uint32_t port_1_en;
        uint32_t port_2_en;
        uint32_t port_3_en;
        uint32_t port_4_en;
        uint32_t port_5_en;
        uint32_t port_6_en;
        uint32_t led_1_status;
        uint32_t led_2_status;
        uint32_t led_3_status;
        uint32_t led_4_status;
    } status;
    uint32_t raw[10];
} board_port_status_t;

typedef enum { TIMER_EVENT_ID_TIMEOUT_500MS, TIMER_EVENT_ID_TIMEOUT_10S } timer_event_id_t;

typedef void (*remote_control_handle_t)(void);
typedef void (*timer_callback_handler_t)(void *args);

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

static led_strip_t led_strip[LED_STRIP_NUMBER_OF_STRIP];

static led_behavior_t leds_behavior[LED_STRIP_NUMBER_OF_STRIP];

static esp_event_loop_handle_t event_loop_handle;

static nvs_handle_t led_nvs;

//**Forward declaration for button handler*//
static board_port_status_t board_status = {0};

static void nvs_save_port_status(const char *key, uint32_t value);
// Function prototypes for each button press event
static void button_1_press_first_time(void);
static void button_1_press_second_time(void);
static void button_1_press_third_time(void);

static void button_2_press_first_time(void);
static void button_2_press_second_time(void);
// static void button_2_press_third_time(void);

static void button_3_press_first_time(void);
static void button_3_press_second_time(void);
// static void button_3_press_third_time(void);

static void button_4_press_first_time(void);
static void button_4_press_second_time(void);
static void button_4_press_third_time(void);

static void button_5_press_first_time(void);
static void button_5_press_second_time(void);
static void button_5_press_third_time(void);

static void button_6_press_first_time(void);
static void button_6_press_second_time(void);
static void button_6_press_third_time(void);

static void button_7_press_first_time(void);
static void button_7_press_second_time(void);
static void button_7_press_third_time(void);

static void timer_1_500ms_callback(void *arg);
static void timer_2_500ms_callback(void *arg);
static void timer_3_500ms_callback(void *arg);
static void timer_4_500ms_callback(void *arg);

// Initialize the button handler array with function pointers for all buttons
remote_control_handle_t button_handler[BUTTON_NUMBER][BUTTON_PRESS_MAX_TIMES] = {
    {button_1_press_first_time, button_1_press_second_time, button_1_press_third_time}, // Button 1
    {button_2_press_first_time, button_2_press_second_time, NULL},                      // Button 2
    {button_3_press_first_time, button_3_press_second_time, NULL},                      // Button 3
    {button_4_press_first_time, button_4_press_second_time, button_4_press_third_time}, // Button 4
    {button_5_press_first_time, button_5_press_second_time, button_5_press_third_time}, // Button 5
    {button_6_press_first_time, button_6_press_second_time, button_6_press_third_time}, // Button 6
    {button_7_press_first_time, button_7_press_second_time, button_7_press_third_time}, // Button 7
    {NULL, NULL, NULL},                                                                 // Button 8
    {NULL, NULL, NULL}                                                                  // Button 9
};

static esp_timer_create_args_t timer_500_ms_args[LED_STRIP_NUMBER_OF_STRIP],
    timer_10_s_args[LED_STRIP_NUMBER_OF_STRIP];
static timer_callback_handler_t timer_500_ms_callback[LED_STRIP_NUMBER_OF_STRIP] = {
    timer_1_500ms_callback, timer_2_500ms_callback, timer_3_500ms_callback, timer_4_500ms_callback};
static esp_timer_handle_t timer_500_ms_handle[LED_STRIP_NUMBER_OF_STRIP] /*, timer_10_s_handle*/;

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
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols, nec_code_t *nec_code) {
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
    nec_code->address = address;
    nec_code->address = command;
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
static esp_err_t parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num,
                                 nec_code_t *nec_code) {
    esp_err_t err = ESP_FAIL;
    if (nec_code == NULL) {
        return err;
    }
    ESP_LOGI(TAG, "NEC frame start---\r\n");
    for (size_t i = 0; i < symbol_num; i++) {
        ESP_LOGI(TAG, "{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
                 rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    ESP_LOGI(TAG, "---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols, nec_code)) {
            ESP_LOGI(TAG, "Address=%04X, Command=%04X\r\n\r\n", nec_code->address, nec_code->command);
            err = ESP_OK;
        }
        return err;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols)) {
            ESP_LOGI(TAG, "Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address,
                     s_nec_code_command);
        }
        return err;
    default:
        ESP_LOGI(TAG, "Unknown NEC frame\r\n\r\n");
        return err;
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
 * @brief Initialization of the RGB LED pin and RMT peripheral drivers.
 *
 * This function configures the RGB LED strip by setting the GPIO pin, maximum number of LEDs,
 * LED model, color component format, and output inversion for each strip. It also configures
 * the RMT (Remote Control) backend for the LED strip.
 */
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
        .clk_src = RMT_CLK_SRC_DEFAULT, // Different clock source can lead to different power consumption.
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency.
        .mem_block_symbols = 64,               // The memory size of each RMT channel, in words (4 bytes).
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4.
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

static void led_behavior(esp_event_base_t base, int32_t event_id, led_behavior_t *led_behavior,
                         led_strip_handle_t led_handle, uint32_t port_number,
                         esp_event_base_t target_timer_base) {
    switch (led_behavior->led_status) {
    case LED_STATUS_ALL_TURN_OFF:
        ESP_ERROR_CHECK(led_strip_clear(led_handle));
        break;
    case LED_STATUS_ALL_TURN_ON:
        ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, 0, 255, 0, 0));
        ESP_ERROR_CHECK(led_strip_refresh(led_handle));
        break;
    case LED_STATUS_ALL_BLINK:
        if (event_id == TIMER_EVENT_ID_TIMEOUT_500MS && base == target_timer_base) {
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
        if (event_id == TIMER_EVENT_ID_TIMEOUT_500MS && base == target_timer_base) {
            ESP_ERROR_CHECK(led_strip_clear(led_handle));
            ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, led_behavior->led_index, 255, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_handle));
            led_behavior->led_index++;
        }
        break;
    case LED_STATUS_ALL_BLINK_10_SEC:
        if (base == target_timer_base) {
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
                led_behavior->led_status = LED_STATUS_MAX;
            }
        }
        break;
    // Do nothing
    case LED_STATUS_MAX:
        break;
    default:
        ESP_LOGE(TAG, "Not a valid event :%d", (int)led_behavior->led_status);
        break;
    }
}

static void timer_handler_500ms_start_stop(esp_timer_handle_t handler, int32_t event_id) {
    if (event_id == LED_STATUS_ALL_BLINK) {
        if (esp_timer_is_active(handler) == false) {
            esp_timer_start_periodic(handler, 5000);
        }
    } else {
        if (esp_timer_is_active(handler)) {
            esp_timer_stop(handler);
        }
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
        if (event_id != LED_STATUS_STEP_ON) {
            leds_behavior[0].led_index = 0;
        }
        leds_behavior[0].led_status = event_id;
        nvs_save_port_status("led_1_status", event_id);
        timer_handler_500ms_start_stop(timer_500_ms_handle[0], event_id);
    }
    led_behavior(base, event_id, &leds_behavior[0], led_strip[0].led_handle, 1, TIMER_1_EVENT_BASE);
}

static void port_2_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_2_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        if (event_id != LED_STATUS_STEP_ON) {
            leds_behavior[1].led_index = 0;
        }
        timer_handler_500ms_start_stop(timer_500_ms_handle[1], event_id);
        leds_behavior[1].led_status = event_id;
        nvs_save_port_status("led_2_status", event_id);
    }
    led_behavior(base, event_id, &leds_behavior[1], led_strip[1].led_handle, 2, TIMER_2_EVENT_BASE);
}

static void port_3_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_3_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        if (event_id != LED_STATUS_STEP_ON) {
            leds_behavior[2].led_index = 0;
        }
        timer_handler_500ms_start_stop(timer_500_ms_handle[2], event_id);
        leds_behavior[2].led_status = event_id;
        nvs_save_port_status("led_3_status", event_id);
    }
    led_behavior(base, event_id, &leds_behavior[2], led_strip[2].led_handle, 3, TIMER_3_EVENT_BASE);
}
static void port_4_event_handler(void *handler_arg, esp_event_base_t base, int32_t event_id,
                                 void *event_data) {
    if (base == LED_4_EVENT_BASE) {
        if (event_id > LED_STATUS_MAX) {
            ESP_LOGE(TAG, "Invalid led status event_id :%d", (int)event_id);
        }
        if (event_id != LED_STATUS_STEP_ON) {
            leds_behavior[3].led_index = 0;
        }
        leds_behavior[3].led_status = event_id;
        nvs_save_port_status("led_4_status", event_id);
        timer_handler_500ms_start_stop(timer_500_ms_handle[3], event_id);
    }
    led_behavior(base, event_id, &leds_behavior[3], led_strip[3].led_handle, 4, TIMER_4_EVENT_BASE);
}
// Timer callback function
static void timer_1_500ms_callback(void *arg) {
    // ESP_LOGI(TAG,"Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, TIMER_1_EVENT_BASE, TIMER_EVENT_ID_TIMEOUT_500MS, NULL, 0,
                      portMAX_DELAY);
}
static void timer_2_500ms_callback(void *arg) {
    // ESP_LOGI(TAG,"Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, TIMER_2_EVENT_BASE, TIMER_EVENT_ID_TIMEOUT_500MS, NULL, 0,
                      portMAX_DELAY);
}
static void timer_3_500ms_callback(void *arg) {
    // ESP_LOGI(TAG,"Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, TIMER_3_EVENT_BASE, TIMER_EVENT_ID_TIMEOUT_500MS, NULL, 0,
                      portMAX_DELAY);
}
static void timer_4_500ms_callback(void *arg) {
    // ESP_LOGI(TAG,"Timer expired! Posting event...\n");
    esp_event_post_to(event_loop_handle, TIMER_4_EVENT_BASE, TIMER_EVENT_ID_TIMEOUT_500MS, NULL, 0,
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
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
}

static esp_err_t nvs_read_status(const char *key, uint32_t *out_val) {
    esp_err_t err = ESP_FAIL;

    err = nvs_get_u32(led_nvs, key, out_val);

    switch (err) {
    case ESP_OK:
        ESP_LOGI(TAG, "Board led status done %d\n", (int)*out_val);

        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI(TAG, "The value is not initialized yet!\n");
        break;
    default:
        ESP_LOGI(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }
    return err;
}

static void nvs_restore_config() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("storage", NVS_READWRITE, &led_nvs);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Done\n");
        ESP_LOGI(TAG, "Reading led_status from NVS ... ");
        nvs_read_status("port_5_en", &board_status.status.port_5_en);
        nvs_read_status("port_6_en", &board_status.status.port_6_en);
        nvs_read_status("led_1_status", &board_status.status.led_1_status);
        nvs_read_status("led_2_status", &board_status.status.led_2_status);
        nvs_read_status("led_3_status", &board_status.status.led_3_status);
        nvs_read_status("led_4_status", &board_status.status.led_1_status);
    }
}

/*
 * Saving the
 * */
static void nvs_save_port_status(const char *key, uint32_t value) {
    nvs_set_u32(led_nvs, key, value);
    nvs_commit(led_nvs);
}
///////////////////////////////////////////////////////////////////////////
static void button_1_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);

    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
}
static void button_1_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_BLINK, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_BLINK, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_BLINK, NULL, 0, portMAX_DELAY);

    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_BLINK, NULL, 0, portMAX_DELAY);
}
static void button_1_press_third_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);

    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}
/////////////////////////////////////////////////////////////////////
static void button_1_press_handler(uint32_t number_of_press_time) {
    if (button_handler[0][number_of_press_time])
        button_handler[0][number_of_press_time]();
}

static void button_2_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
}
static void button_2_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}
// static void button_2_press_third_time(void) {
//     ESP_LOGI(TAG, "Do nothing if pressed button 2 the third times");
// }

static void button_2_press_handler(uint32_t number_of_press_time) {
    if (button_handler[1][number_of_press_time])
        button_handler[1][number_of_press_time]();
}
//////////////////////////////////////////////////////////
static void button_3_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_TURN_ON, NULL, 0, portMAX_DELAY);
}
static void button_3_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}
// static void button_3_press_third_time(void) {
//     ESP_LOGI(TAG, "Do nothing if pressed button 3 the third times");
// }

static void button_3_press_handler(uint32_t number_of_press_time) {
    if (button_handler[2][number_of_press_time])
        button_handler[2][number_of_press_time]();
}
/////////////////////////////////////////////////////////////////////////
static void button_4_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_STEP_ON, NULL, 0, portMAX_DELAY);
}
static void button_4_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
}
static void button_4_press_third_time(void) {
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}

static void button_4_press_handler(uint32_t number_of_press_time) {
    if (button_handler[3][number_of_press_time])
        button_handler[3][number_of_press_time]();
}
///////////////////////////////////////////////////////////////////////////
static void button_5_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_STEP_ON, NULL, 0, portMAX_DELAY);
}
static void button_5_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
}
static void button_5_press_third_time(void) {
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}

static void button_5_press_handler(uint32_t number_of_press_time) {
    if (button_handler[4][number_of_press_time])
        button_handler[4][number_of_press_time]();
}
///////////////////////////////////////////////////////////////////////
static void button_6_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_STEP_ON, NULL, 0, portMAX_DELAY);
}
static void button_6_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
}
static void button_6_press_third_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}

static void button_6_press_handler(uint32_t number_of_press_time) {
    if (button_handler[5][number_of_press_time])
        button_handler[5][number_of_press_time]();
}
////////////////////////////////////////////////////////////////////////
static void button_7_press_first_time(void) {
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_STEP_ON, NULL, 0, portMAX_DELAY);
}
static void button_7_press_second_time(void) {
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, LED_STATUS_ALL_BLINK_10_SEC, NULL, 0,
                      portMAX_DELAY);
}
static void button_7_press_third_time(void) {
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, LED_STATUS_ALL_TURN_OFF, NULL, 0, portMAX_DELAY);
}

static void button_7_press_handler(uint32_t number_of_press_time) {
    if (button_handler[3][number_of_press_time])
        button_handler[3][number_of_press_time]();
}

static void button_8_press_handler(uint32_t number_of_press_time) {
    (void)number_of_press_time;
    if (board_status.status.port_5_en) {
        port_en(LED_EN_PORT_POWER_5, PORT_OFF);
        board_status.status.port_5_en = 0;
    } else {
        port_en(LED_EN_PORT_POWER_5, PORT_ON);
        board_status.status.port_5_en = 1;
    }
    nvs_save_port_status("port_5_en", board_status.status.port_5_en);
}

static void button_9_press_handler(uint32_t number_of_press_time) {
    (void)number_of_press_time;
    if (board_status.status.port_6_en) {
        port_en(LED_EN_PORT_POWER_6, PORT_OFF);
        board_status.status.port_6_en = 0;
    } else {
        port_en(LED_EN_PORT_POWER_6, PORT_ON);
        board_status.status.port_6_en = 1;
    }
    nvs_save_port_status("port_5_en", board_status.status.port_5_en);
}

static void restore_port_status(void) {
    port_en(LED_EN_PORT_POWER_5, board_status.status.port_5_en);
    port_en(LED_EN_PORT_POWER_6, board_status.status.port_6_en);

    esp_event_post_to(event_loop_handle, LED_1_EVENT_BASE, board_status.status.led_1_status, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_2_EVENT_BASE, board_status.status.led_2_status, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_3_EVENT_BASE, board_status.status.led_3_status, NULL, 0,
                      portMAX_DELAY);
    esp_event_post_to(event_loop_handle, LED_4_EVENT_BASE, board_status.status.led_4_status, NULL, 0,
                      portMAX_DELAY);
}

static void timer_initialize(void) {
    for (int i = 0; i < LED_STRIP_NUMBER_OF_STRIP; i++) {
        char name[20] = {0};
        sprintf(name, "timer_%d_500ms", i + 1);
        timer_500_ms_args[i].callback = timer_500_ms_callback[i];
        esp_timer_create(&timer_500_ms_args[i], &timer_500_ms_handle[i]);
    }
}

void app_main(void) {

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data = {0};

    uint32_t last_remote_signal = 0xFFFFFFFF;
    uint32_t signal_count = 1;
    bool reset_count = false;

    configure_port_en();
    configure_led();
    configure_ir();

    nvs_restore_config();
    esp_event_loop_args_t loop_args = {.queue_size = 10,
                                       .task_name = "event_task",
                                       .task_priority = uxTaskPriorityGet(NULL),
                                       .task_stack_size = 4096,
                                       .task_core_id = tskNO_AFFINITY};

    esp_event_loop_create(&loop_args, &event_loop_handle);
    // Register event handler
    esp_event_handler_instance_register_with(event_loop_handle, LED_1_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_1_event_handler, NULL, NULL);
    esp_event_handler_instance_register_with(event_loop_handle, TIMER_1_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_1_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, LED_2_EVENT_BASE, ESP_EVENT_ANY_ID,

                                             port_2_event_handler, NULL, NULL);
    esp_event_handler_instance_register_with(event_loop_handle, TIMER_2_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_2_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, LED_3_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_3_event_handler, NULL, NULL);
    esp_event_handler_instance_register_with(event_loop_handle, TIMER_3_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_3_event_handler, NULL, NULL);

    esp_event_handler_instance_register_with(event_loop_handle, LED_4_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_4_event_handler, NULL, NULL);
    esp_event_handler_instance_register_with(event_loop_handle, TIMER_4_EVENT_BASE, ESP_EVENT_ANY_ID,
                                             port_4_event_handler, NULL, NULL);
    // const esp_timer_create_args_t timer_10s_args = {.callback = &timer_500ms_callback, .name =
    // "timer_500ms"};
    timer_initialize();
    // esp_timer_start_periodic(timer_500_ms_handle, 5000); // Time in microseconds

    // Restore last port status
    restore_port_status();

    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    while (1) {
        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            uint32_t control_value = 0;
            nec_code_t nec_code = {0};
            // parse the receive symbols and print the result
            parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols, &nec_code);
            switch (control_value) {
            case BUTTON_NEC_CODE_1:
                button_2_press_handler(signal_count);
                if (signal_count >= 1) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_2:
                button_3_press_handler(signal_count);
                if (signal_count >= 1) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_3:
                button_4_press_handler(signal_count);
                if (signal_count >= 2) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_4:
                button_5_press_handler(signal_count);
                if (signal_count >= 2) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_5:
                button_6_press_handler(signal_count);
                if (signal_count >= 2) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_6:
                button_7_press_handler(signal_count);
                if (signal_count >= 2) {
                    signal_count = 0;
                    reset_count = true;
                }
                break;
            case BUTTON_NEC_CODE_7:
                button_8_press_handler(signal_count);
                break;
            case BUTTON_NEC_CODE_8:
                button_9_press_handler(signal_count);
                break;
            case BUTTON_NEC_CODE_ON_OFF:
                button_1_press_handler(signal_count);
                if (signal_count >= 2) {
                    signal_count = 0;
                }
                break;
            default:
                ESP_LOGW(TAG, "Invalid signal from remote: %d", (int)control_value);
                break;
            }
            if (last_remote_signal == control_value) {
                if (reset_count) {
                    reset_count = false;
                } else {
                    signal_count++;
                }
            } else {
                signal_count = 0;
            }
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }
    }
}
