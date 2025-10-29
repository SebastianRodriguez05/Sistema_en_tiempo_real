#include "drv_rgb.h"
#include "driver/ledc.h"
#include "esp_log.h"

esp_err_t rgb_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    const int pins[3] = { PIN_LED_R, PIN_LED_G, PIN_LED_B };
    for (int i = 0; i < 3; ++i) {
        ledc_channel_config_t c = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
    return ESP_OK;
}

esp_err_t rgb_set_raw(uint16_t duty_r, uint16_t duty_g, uint16_t duty_b)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_r));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_g));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_b));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
    return ESP_OK;
}

esp_err_t rgb_set_percent(float pr, float pg, float pb)
{
    if (pr < 0)   pr = 0;
    if (pr > 100) pr = 100;

    if (pg < 0)   pg = 0;
    if (pg > 100) pg = 100;

    if (pb < 0)   pb = 0;
    if (pb > 100) pb = 100;

    uint16_t r = (uint16_t)(pr * 40.95f);  // 100% -> 4095
    uint16_t g = (uint16_t)(pg * 40.95f);
    uint16_t b = (uint16_t)(pb * 40.95f);
    return rgb_set_raw(r, g, b);
}

esp_err_t rgb_off(void)
{
    return rgb_set_raw(0, 0, 0);
}
