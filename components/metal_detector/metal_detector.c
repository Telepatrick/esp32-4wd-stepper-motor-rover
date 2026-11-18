#include "metal_detector.h"

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define COIL_PIN GPIO_NUM_5
#define SENSOR_PIN ADC_CHANNEL_3
#define BUZZER_PIN GPIO_NUM_17

#define PULSE_US 40
#define SAMPLE_DELAY_US 2

#define BASELINE_SAMPLES 100

#define MEASURE_COUNT 4
#define MEASURE_INTERVAL_MS 20

#define METAL_THRESHOLD 30

#define METAL_CONFIRMATIONS 4
#define METAL_WINDOW 5

#define BASELINE_TRACKING_DIVIDER 64

#define BUZZER_FREQUENCY 2000
#define BUZZER_DUTY 128

#define BEEP_ON_MS 100
#define BEEP_OFF_MS 150

static adc_oneshot_unit_handle_t adc_handle;
static portMUX_TYPE measure_mux = portMUX_INITIALIZER_UNLOCKED;

static int baseline = 0;
static int current_adc = 0;
static int current_difference = 0;

static volatile bool metal_detected = false;

static void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, BUZZER_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void init_coil(void)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << COIL_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&config);
    gpio_set_level(COIL_PIN, 0);
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    adc_oneshot_new_unit(&unit_config, &adc_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(adc_handle, SENSOR_PIN, &channel_config);
}

static void init_buzzer(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = BUZZER_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel_config);
    buzzer_off();
}

static int measure(void)
{
    int adc = 0;

    portENTER_CRITICAL(&measure_mux);
    gpio_set_level(COIL_PIN, 1);
    esp_rom_delay_us(PULSE_US);
    gpio_set_level(COIL_PIN, 0);
    esp_rom_delay_us(SAMPLE_DELAY_US);
    portEXIT_CRITICAL(&measure_mux);

    adc_oneshot_read(adc_handle, SENSOR_PIN, &adc);

    return adc;
}

static int measure_best(void)
{
    int adc = measure();

    for (int i = 1; i < MEASURE_COUNT; i++) {
        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));

        int current = measure();

        if (current < adc) {
            adc = current;
        }
    }

    return adc;
}

void metal_detector_calibrate(void)
{
    printf("Calibrating metal detector...\n");

    long sum = 0;

    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        int adc = measure_best();
        sum += adc;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    baseline = sum / BASELINE_SAMPLES;
    printf("Baseline = %d. Calibration complete.\n", baseline);
}

static void detector_task(void *pvParameters)
{
    (void)pvParameters;

    uint8_t metal_history[METAL_WINDOW] = {0};
    uint8_t metal_history_index = 0;
    uint64_t buzzer_timer = 0;
    bool buzzer_on_state = false;

    while (1) {
        int adc = measure_best();
        current_adc = adc;

        int difference = baseline - adc;
        current_difference = difference;

        bool metal_signal = difference >= METAL_THRESHOLD;

        metal_history[metal_history_index] = metal_signal ? 1 : 0;
        metal_history_index++;

        if (metal_history_index >= METAL_WINDOW) {
            metal_history_index = 0;
        }

        uint8_t confirmations = 0;
        for (int i = 0; i < METAL_WINDOW; i++) {
            confirmations += metal_history[i];
        }

        metal_detected = confirmations >= METAL_CONFIRMATIONS;

        if (!metal_detected) {
            baseline += (adc - baseline) / BASELINE_TRACKING_DIVIDER;
        }

        uint64_t now_ms = esp_timer_get_time() / 1000;

        if (metal_detected) {
            if (buzzer_on_state) {
                if (now_ms - buzzer_timer >= BEEP_ON_MS) {
                    buzzer_on_state = false;
                    buzzer_timer = now_ms;
                    buzzer_off();
                }
            } else {
                if (now_ms - buzzer_timer >= BEEP_OFF_MS) {
                    buzzer_on_state = true;
                    buzzer_timer = now_ms;
                    buzzer_on();
                }
            }
        } else {
            buzzer_on_state = false;
            buzzer_timer = now_ms;
            buzzer_off();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void metal_detector_init(void)
{
    init_coil();
    init_adc();
    init_buzzer();
}

void metal_detector_start(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    metal_detector_calibrate();
    xTaskCreate(detector_task, "metal_detector", 4096, NULL, 2, NULL);
}

bool metal_detector_is_detected(void)
{
    return metal_detected;
}

int metal_detector_get_adc(void)
{
    return current_adc;
}

int metal_detector_get_baseline(void)
{
    return baseline;
}

int metal_detector_get_difference(void)
{
    return current_difference;
}