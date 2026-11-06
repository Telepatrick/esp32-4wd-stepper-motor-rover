#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "ultrasonic.h"

#define SAFE_DISTANCE_CM 30.0f

void app_main(void)
{
    init_motor_system();
    init_ultrasonic_sensors();

    printf("System initialized. Moving forward\n");
    fflush(stdout);
    
    forward();

    while (true) {
        float distances[4];
        read_distances(distances);

        printf("S1: %.1f cm | S2: %.1f cm | S3: %.1f cm | S4: %.1f cm\n", 
               distances[0], distances[1], distances[2], distances[3]);
        fflush(stdout);

        bool obstacle_detected = false;
        for (int i = 0; i < 4; i++) {
            if (distances[i] > 0.0f && distances[i] < SAFE_DISTANCE_CM) {
                obstacle_detected = true;
                break;
            }
        }

        if (obstacle_detected) {
            float left_min = (distances[0] < distances[1]) ? distances[0] : distances[1];
            float right_min = (distances[2] < distances[3]) ? distances[2] : distances[3];

            if (left_min < right_min) {
                printf(">>> Obstacle on the LEFT! Turning RIGHT...\n");
                turn_right();
            } else {
                printf(">>> Obstacle on the RIGHT! Turning LEFT...\n");
                turn_left();
            }
            fflush(stdout);

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(150));
                read_distances(distances);

                printf("Turning... S1: %.1f | S2: %.1f | S3: %.1f | S4: %.1f\n", 
                       distances[0], distances[1], distances[2], distances[3]);
                fflush(stdout);

                bool path_clear = true;
                for (int i = 0; i < 4; i++) {
                    if (distances[i] > 0.0f && distances[i] < SAFE_DISTANCE_CM) {
                        path_clear = false;
                        break;
                    }
                }

                if (path_clear) {
                    printf(">>> Path is clear. Resuming forward motion.\n");
                    fflush(stdout);
                    break;
                }
            }
            forward();
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}