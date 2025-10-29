#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RGB_CATHODE_COMMON = 0,
    RGB_ANODE_COMMON
} rgb_polarity_t;

typedef struct {
    int pin_r, pin_g, pin_b;
    ledc_mode_t       speed_mode;
    ledc_timer_t      timer_num;
    ledc_timer_bit_t  duty_resolution;
    uint32_t          freq_hz;
    ledc_channel_t    ch_r, ch_g, ch_b;
    rgb_polarity_t    polarity;
} rgb_cfg_t;

typedef struct rgb_handle_s* rgb_handle_t;

esp_err_t rgb_create(const rgb_cfg_t* cfg, rgb_handle_t* out);
esp_err_t rgb_set_rgb8(rgb_handle_t h, uint8_t r, uint8_t g, uint8_t b);
esp_err_t rgb_off(rgb_handle_t h);
void      rgb_destroy(rgb_handle_t h);

#ifdef __cplusplus
}
#endif
