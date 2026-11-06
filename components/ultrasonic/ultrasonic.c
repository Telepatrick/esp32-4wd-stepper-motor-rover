#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define US_TRIG_PIN GPIO_NUM_12
#define US_ECHO_LEFT GPIO_NUM_13
#define US_ECHO_LEFT_CENTER GPIO_NUM_14
#define US_ECHO_RIGHT_CENTER GPIO_NUM_27
#define US_ECHO_RIGHT GPIO_NUM_26

void init_ultrasonic_sensors(void) {
    gpio_config_t trig_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << US_TRIG_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&trig_conf);
    gpio_set_level(US_TRIG_PIN, 0);

    gpio_config_t echo_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << US_ECHO_LEFT) | (1ULL << US_ECHO_LEFT_CENTER) |
                        (1ULL << US_ECHO_RIGHT_CENTER) | (1ULL << US_ECHO_RIGHT),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&echo_conf);
}

void read_distances(float distances[4]) {
    gpio_set_level(US_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 0);

    gpio_num_t echo_pins[4] = {US_ECHO_LEFT, US_ECHO_LEFT_CENTER, US_ECHO_RIGHT_CENTER, US_ECHO_RIGHT};
    int64_t start_times[4] = {0};
    int64_t end_times[4] = {0};
    bool measured[4] = {false, false, false, false};

    int64_t timeout_timer = esp_timer_get_time();

    while (esp_timer_get_time() - timeout_timer < 30000) {
        bool all_started = true;
        for (int i = 0; i < 4; i++) {
            if (!start_times[i]) {
                if (gpio_get_level(echo_pins[i]) == 1) {
                    start_times[i] = esp_timer_get_time();
                } else {
                    all_started = false;
                }
            }
        }
        if (all_started) break;
    }

    timeout_timer = esp_timer_get_time();
    while (esp_timer_get_time() - timeout_timer < 30000) {
        bool all_finished = true;
        for (int i = 0; i < 4; i++) {
            if (start_times[i] && !measured[i]) {
                if (gpio_get_level(echo_pins[i]) == 0) {
                    end_times[i] = esp_timer_get_time();
                    measured[i] = true;
                } else {
                    all_finished = false;
                }
            }
        }
        if (all_finished) break;
    }

    for (int i = 0; i < 4; i++) {
        if (measured[i] && end_times[i] > start_times[i]) {
            uint32_t duration = (uint32_t)(end_times[i] - start_times[i]);
            distances[i] = (float)duration / 58.0f;
        } else {
            distances[i] = 400.0f;
        }
    }
}