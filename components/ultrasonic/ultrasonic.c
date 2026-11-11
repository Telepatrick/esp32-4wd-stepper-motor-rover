#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"

#define US_TRIG_PIN GPIO_NUM_12
#define US_ECHO_LEFT GPIO_NUM_13
#define US_ECHO_LEFT_CENTER GPIO_NUM_14
#define US_ECHO_RIGHT_CENTER GPIO_NUM_27
#define US_ECHO_RIGHT GPIO_NUM_26

void init_ultrasonic_sensors(void)
{
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
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&echo_conf);
}

void read_distances(float distances[4])
{
    typedef struct {
        uint32_t start_time;
        uint32_t stop_time;
        bool start_flag;
        bool stop_flag;
    } sonar_state_t;

    sonar_state_t states[4];
    
    uint32_t sonar_masks[4] = {
        (1U << US_ECHO_LEFT),
        (1U << US_ECHO_LEFT_CENTER),
        (1U << US_ECHO_RIGHT_CENTER),
        (1U << US_ECHO_RIGHT)
    };

    for (int i = 0; i < 4; i++) {
        states[i].start_time = 0;
        states[i].stop_time = 0;
        states[i].start_flag = false;
        states[i].stop_flag = false;
        distances[i] = 400.0f;
    }

    gpio_set_level(US_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 0);

    uint32_t start_loop_time = (uint32_t)esp_timer_get_time();
    uint32_t timeout_us = 30000;
    uint32_t end_time_limit = start_loop_time + timeout_us;
    int sensors_closed = 0;

    while (((uint32_t)esp_timer_get_time() < end_time_limit) && (sensors_closed < 4)) {
        uint32_t current_time = (uint32_t)esp_timer_get_time();
        uint32_t reg_in = REG_READ(GPIO_IN_REG);

        for (int i = 0; i < 4; i++) {
            if (!states[i].stop_flag) {
                if (!states[i].start_flag) {
                    if ((reg_in & sonar_masks[i]) != 0) {
                        states[i].start_time = current_time;
                        states[i].start_flag = true;
                    }
                } else {
                    if ((reg_in & sonar_masks[i]) == 0) {
                        states[i].stop_time = current_time;
                        states[i].stop_flag = true;
                        sensors_closed++;
                    }
                }
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        if (states[i].stop_flag && states[i].stop_time > states[i].start_time) {
            uint32_t pulse_time = states[i].stop_time - states[i].start_time;
            distances[i] = (float)pulse_time * 0.017f;
        } else {
            distances[i] = 400.0f;
        }
    }
}