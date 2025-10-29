#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

// ---- Pines/Canales (ESP32-C6) ----
#define PIN_POT_ADC_GPIO 0      // ADC1_CH0
#define PIN_NTC_ADC_GPIO 1      // ADC1_CH1

// ---- Divisor NTC: 3V3 -> R_SERIES -> nodo -> NTC -> GND ----
#define V_SUPPLY_MV      3300
#define R_SERIES_OHM     1000.0f     // 1k arriba
#define NTC_R25_OHM      10000.0f    // 10k @ 25°C
#define NTC_BETA         3950.0f
#define T0_K             298.15f     // 25°C

typedef struct {
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t cali_pot;
    adc_cali_handle_t cali_ntc;
    bool cal_ok_pot;
    bool cal_ok_ntc;
} ChromeInCtx;

typedef struct {
    uint16_t pot_mv;   // Potenciómetro (mV)
    uint16_t ntc_mv;   // Nodo NTC (mV)
    float    temp_c;   // Temperatura estimada (°C)
} ChromeInData;

// ---- API ----
esp_err_t chromein_init(ChromeInCtx *ctx);
esp_err_t chromein_read_all(ChromeInCtx *ctx, ChromeInData *out);

// ---- Helpers (por si los quieres usar en tu lógica) ----
float chromein_ntc_res_from_mv(uint16_t mv_node);
float chromein_ntc_temp_c_from_res(float r_ntc);
