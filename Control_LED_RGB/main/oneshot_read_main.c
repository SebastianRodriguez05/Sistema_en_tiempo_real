#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drv_rgb.h"
#include "chromein.h"

static inline float clampf(float x,float a,float b){ return (x<a)?a:((x>b)?b:x); }
static float brightness_from_pot(uint16_t pot_mv){
    float x = clampf((float)pot_mv / 3300.0f, 0.f, 1.f);
    return powf(x, 1.2f); // leve gamma
}

void app_main(void)
{
    // Drivers
    rgb_init();

    ChromeInCtx cin = {0};
    chromein_init(&cin);

    // Umbrales de ejemplo (ajusta a tu gusto, esta es tu lógica)
    const float rmin = 28.0f;
    const float gmin = 20.0f;
    const float bmin = 0.0f;

    while (1) {
        ChromeInData d = {0};
        if (chromein_read_all(&cin, &d) == ESP_OK) {
            float br = brightness_from_pot(d.pot_mv);   // 0..1
            float r = (d.temp_c >= rmin) ? br : 0.f;
            float g = (d.temp_c >= gmin) ? br : 0.f;
            float b = (d.temp_c >= bmin) ? br : 0.f;

            rgb_set_percent(r*100.f, g*100.f, b*100.f);

            float rntc = chromein_ntc_res_from_mv(d.ntc_mv);
            printf("POT:%4u mV | NTC:%4u mV | Rntc:%7.0f ohm | Temp:%6.2f C | br:%.3f\n",
                   (unsigned)d.pot_mv, (unsigned)d.ntc_mv, (double)rntc, (double)d.temp_c, (double)br);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
