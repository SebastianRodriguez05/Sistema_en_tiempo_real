// main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Driver_RGB.h"

// ======= EDITA AQUÍ EL COLOR (0–255) =======
#define COLOR_R   255
#define COLOR_G   0
#define COLOR_B   0
// ===========================================

void app_main(void)
{
    rgb_cfg_t cfg = {
        .pin_r = 15, .pin_g = 2, .pin_b = 4,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 4000,
        .ch_r = LEDC_CHANNEL_0, .ch_g = LEDC_CHANNEL_1, .ch_b = LEDC_CHANNEL_2,
        .polarity = RGB_CATHODE_COMMON   // cambia a RGB_CATHODE_COMMON si tu LED es cátodo común
    };

    rgb_handle_t h = NULL;
    if (rgb_create(&cfg, &h) != ESP_OK) return;

    // Aplica el color elegido
    rgb_set_rgb8(h, COLOR_R, COLOR_G, COLOR_B);

    // Mantener app viva (no hace nada más)
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
