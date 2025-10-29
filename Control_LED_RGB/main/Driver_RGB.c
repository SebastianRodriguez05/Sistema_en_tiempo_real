#include "Driver_RGB.h"
#include <stdlib.h>
#include "esp_log.h"        // <-- para TAG / logs
#include "esp_check.h"      // <-- para ESP_RETURN_ON_ERROR

static const char* TAG = "Driver_RGB_MIN";

// Handle interno
typedef struct rgb_handle_s {
    rgb_cfg_t  cfg;
    uint32_t   duty_max; // (1 << duty_resolution) - 1
} rgb_handle_int_t;

// Helpers
static inline uint32_t map_u8_to_duty(const rgb_handle_int_t* h, uint8_t v8) {
    uint32_t duty = (uint32_t)((v8 * (uint32_t)h->duty_max) / 255U);
    if (h->cfg.polarity == RGB_ANODE_COMMON) {
        duty = h->duty_max - duty; // invertir si es ánodo común
    }
    return duty;
}

esp_err_t rgb_create(const rgb_cfg_t* cfg, rgb_handle_t* out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;

    rgb_handle_int_t* h = (rgb_handle_int_t*)calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->cfg = *cfg;
    h->duty_max = (1U << (unsigned)cfg->duty_resolution) - 1U;

    // TIMER
    ledc_timer_config_t tcfg = {
        .speed_mode      = cfg->speed_mode,
        .duty_resolution = cfg->duty_resolution,
        .timer_num       = cfg->timer_num,
        .freq_hz         = cfg->freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) { free(h); return err; }

    // CANALES
    ledc_channel_config_t ccfg = {
        .speed_mode = cfg->speed_mode,
        .timer_sel  = cfg->timer_num,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0
    };

    ccfg.channel = cfg->ch_r; ccfg.gpio_num = cfg->pin_r;
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) { free(h); return err; }

    ccfg.channel = cfg->ch_g; ccfg.gpio_num = cfg->pin_g;
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) { free(h); return err; }

    ccfg.channel = cfg->ch_b; ccfg.gpio_num = cfg->pin_b;
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) { free(h); return err; }

    *out = (rgb_handle_t)h;
    return ESP_OK;
}

esp_err_t rgb_set_rgb8(rgb_handle_t handle, uint8_t r, uint8_t g, uint8_t b)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    rgb_handle_int_t* h = (rgb_handle_int_t*)handle;

    uint32_t dr = map_u8_to_duty(h, r);
    uint32_t dg = map_u8_to_duty(h, g);
    uint32_t db = map_u8_to_duty(h, b);

    esp_err_t err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_r, dr); if (err != ESP_OK) return err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_g, dg); if (err != ESP_OK) return err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_b, db); if (err != ESP_OK) return err;

    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_r); if (err != ESP_OK) return err;
    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_g); if (err != ESP_OK) return err;
    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_b); if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t rgb_off(rgb_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    rgb_handle_int_t* h = (rgb_handle_int_t*)handle;

    uint32_t dzero = (h->cfg.polarity == RGB_ANODE_COMMON) ? h->duty_max : 0;

    esp_err_t err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_r, dzero); if (err != ESP_OK) return err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_g, dzero); if (err != ESP_OK) return err;
    err = ledc_set_duty(h->cfg.speed_mode, h->cfg.ch_b, dzero); if (err != ESP_OK) return err;

    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_r); if (err != ESP_OK) return err;
    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_g); if (err != ESP_OK) return err;
    err = ledc_update_duty(h->cfg.speed_mode, h->cfg.ch_b); if (err != ESP_OK) return err;

    return ESP_OK;
}

void rgb_destroy(rgb_handle_t handle)
{
    if (!handle) return;
    (void)rgb_off(handle);
    free(handle);
}
