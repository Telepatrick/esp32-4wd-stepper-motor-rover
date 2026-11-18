#pragma once

void metal_detector_init(void);
void metal_detector_calibrate(void);
void metal_detector_start(void);
bool metal_detector_is_detected(void);
int metal_detector_get_adc(void);
int metal_detector_get_baseline(void);
int metal_detector_get_difference(void);