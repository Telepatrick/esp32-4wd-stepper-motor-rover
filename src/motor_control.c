#include "motor_control.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"

#define STEP_L GPIO_NUM_18
#define DIR_L GPIO_NUM_5
#define STEP_R GPIO_NUM_23
#define DIR_R GPIO_NUM_22
#define STEPER_EN GPIO_NUM_4

static bool step_l_level = false;
static bool step_r_level = false;

static bool timer_volume_l = false;
static bool timer_volume_r = false;

static gptimer_handle_t gptimer_l = NULL;
static gptimer_handle_t gptimer_r = NULL;

static bool IRAM_ATTR timer_l_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    step_l_level = !step_l_level;
    gpio_set_level(STEP_L, step_l_level);
    return false;
}

static bool IRAM_ATTR timer_r_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    step_r_level = !step_r_level;
    gpio_set_level(STEP_R, step_r_level);
    return false;
}

void setup_motor_system(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_L) | (1ULL << DIR_L) |
                        (1ULL << STEP_R) | (1ULL << DIR_R) |
                        (1ULL << STEPER_EN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(STEPER_EN, 1);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };

    gptimer_new_timer(&timer_config, &gptimer_l);
    gptimer_new_timer(&timer_config, &gptimer_r);

    gptimer_event_callbacks_t cbs_l = {
        .on_alarm = timer_l_callback
    };

    gptimer_event_callbacks_t cbs_r = {
        .on_alarm = timer_r_callback
    };
    
    gptimer_register_event_callbacks(gptimer_l, &cbs_l, NULL);
    gptimer_register_event_callbacks(gptimer_r, &cbs_r, NULL);

    gptimer_enable(gptimer_l);
    gptimer_enable(gptimer_r);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 5000, 
        .flags.auto_reload_on_alarm = true
    };

    gptimer_set_alarm_action(gptimer_l, &alarm_config);
    gptimer_set_alarm_action(gptimer_r, &alarm_config);
}

void motor_on(void)
{
    gpio_set_level(STEPER_EN, 0);
}

void motor_off(void)
{
    gpio_set_level(STEPER_EN, 1);
}

void forward(void)
{
    motor_on();
    gpio_set_level(DIR_L, 1);
    gpio_set_level(DIR_R, 0);

    if (!timer_volume_l) { 
        gptimer_start(gptimer_l); 
        timer_volume_l = true; 
    }
    if (!timer_volume_r) { 
        gptimer_start(gptimer_r); 
        timer_volume_r = true; 
    }
}

void stop_rover(void)
{
    if (timer_volume_l) {
        gptimer_stop(gptimer_l);
        timer_volume_l = false;
    }
    if (timer_volume_r) {
        gptimer_stop(gptimer_r);
        timer_volume_r = false;
    }
    motor_off();
}