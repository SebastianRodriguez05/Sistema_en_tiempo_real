#pragma once
#include "esp_err.h"
#include <stdint.h>

#define PIN_LED_R 4
#define PIN_LED_G 5
#define PIN_LED_B 6

esp_err_t rgb_init(void);
esp_err_t rgb_set_raw(uint16_t duty_r, uint16_t duty_g, uint16_t duty_b);  // 0..4095
esp_err_t rgb_set_percent(float pr, float pg, float pb);                   // 0..100 %
esp_err_t rgb_off(void);

