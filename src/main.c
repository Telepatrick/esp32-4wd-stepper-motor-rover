#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"

void app_main(void)
{
    setup_motor_system();

    forward();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}