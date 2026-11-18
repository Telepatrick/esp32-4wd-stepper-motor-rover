#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "ultrasonic.h"
#include "metal_detector.h"

#define SAFE_DISTANCE_CM 30.0f

#define METAL_ALERT_STOP_MS 1500
#define METAL_REVERSE_MS 5400
#define METAL_TURN_MS 10800

void app_main(void)
{
    init_motor_system();
    init_ultrasonic_sensors();
    metal_detector_init();

    metal_detector_start();

    printf("System initialized. Moving forward\n");
    fflush(stdout);
    
    forward();

    while (true) {
        if (metal_detector_is_detected()) {
            printf(">>> METAL DETECTED! (DIFF: %d) Alert 1.5s...\n", 
                   metal_detector_get_difference());
            fflush(stdout);

            stop_rover();
            vTaskDelay(pdMS_TO_TICKS(METAL_ALERT_STOP_MS));

            printf(">>> Reversing (~20 cm)...\n");
            fflush(stdout);
            backward();
            vTaskDelay(pdMS_TO_TICKS(METAL_REVERSE_MS));

            float dist_check[4];
            read_distances(dist_check);
            float l_min = (dist_check[0] < dist_check[1]) ? dist_check[0] : dist_check[1];
            float r_min = (dist_check[2] < dist_check[3]) ? dist_check[2] : dist_check[3];

            if (l_min < r_min) {
                printf(">>> Turning RIGHT to clear object width (~36 cm)...\n");
                turn_right();
            } else {
                printf(">>> Turning LEFT to clear object width (~36 cm)...\n");
                turn_left();
            }
            fflush(stdout);

            vTaskDelay(pdMS_TO_TICKS(METAL_TURN_MS));

            printf(">>> Maneuver complete. Moving forward.\n");
            fflush(stdout);
            forward();
            continue;
        }

        float distances[4];
        read_distances(distances);

        printf("S1: %.1f cm | S2: %.1f cm | S3: %.1f cm | S4: %.1f cm | MD_DIFF: %d\n", 
               distances[0], distances[1], distances[2], distances[3],
               metal_detector_get_difference());
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

                if (metal_detector_is_detected()) {
                    break;
                }

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
                    forward();
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}