/*
 * Lectura de POTENCIÓMETRO con ADC OneShot (ESP-IDF v5+)
 * - Muestra RAW, mV, V
 * - Calcula R_top (desde 3V3 al wiper) y R_bottom (desde wiper a GND)
 *   asumiendo potenciómetro ideal de resistencia total conocida (POT_TOTAL_OHM).
 *
 * Conexión:
 *   3V3 --/\/\/\--(POT)---/\/\/\-- GND
 *                 |
 *                WIPER ----> ADC1_CHx (GPIO32 en ESP32 clásico)
 *
 * Autor: tú :)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ----------------- Parámetros de medición -----------------
#define VCC_MV             3300              // Alimentación (en mV). Ajusta si tu 3V3 real difiere.
#define SAMPLES_AVG        16                // Promedio de lecturas
#define LOOP_PERIOD_MS     100               // 10 Hz aprox.
#define POT_TOTAL_OHM      5000.0f          // Resistencia total del potenciómetro (p.ej., 10k)

// ----------------- Selección de canal / atenuación -----------------
#if CONFIG_IDF_TARGET_ESP32
// ESP32 clásico: ADC1_CH4 = GPIO32
#   define ADC_CH          ADC_CHANNEL_4
#   define ADC_ATTEN       ADC_ATTEN_DB_11   // Cubre ~0–3.3 V
#else
// ESP32-S2/S3/C3: usa un canal válido de ADC1 (revisa el pinout de tu placa)
#   define ADC_CH          ADC_CHANNEL_2
#   define ADC_ATTEN       ADC_ATTEN_DB_12
#endif

static const char *TAG = "POT_ADC";

// ------------- Prototipos calibración -------------
static bool adc_cali_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                          adc_cali_handle_t *out_handle);
static void adc_cali_deinit(adc_cali_handle_t handle);

// ------------- Helper: convertir mV -> V (float) -------------
static inline float mv_to_volt(int mv) { return (float)mv / 1000.0f; }

// ------------- Lógica de resistencias del pot -------------
// Asumiendo pot ideal y sin carga: V = Vcc * (R_bottom / (R_top + R_bottom)) = Vcc * (R_bottom / R_total)
// => R_bottom = (V / Vcc) * R_total,  R_top = R_total - R_bottom
static inline void pot_resistances_from_voltage(int vmv, float *Rtop, float *Rbottom, float *pos_percent)
{
    if (vmv < 0) vmv = 0;
    if (vmv > VCC_MV) vmv = VCC_MV;
    const float V    = (float)vmv;
    const float Rbot = (V / (float)VCC_MV) * POT_TOTAL_OHM;
    const float Rtop_val = POT_TOTAL_OHM - Rbot;

    if (Rtop)  *Rtop  = Rtop_val;
    if (Rbottom) *Rbottom = Rbot;
    if (pos_percent) *pos_percent = (Rbot / POT_TOTAL_OHM) * 100.0f;  // 0% = GND, 100% = 3V3
}

void app_main(void)
{
    // ---------- Crear unidad ADC1 ----------
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    // ---------- Configurar canal ----------
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CH, &ch_cfg));

    // ---------- Calibración ----------
    adc_cali_handle_t cali = NULL;
    const bool has_cali = adc_cali_init(ADC_UNIT_1, ADC_CH, ADC_ATTEN, &cali);

    ESP_LOGI(TAG, "Pot total=%.0f Ω, VCC=%d mV, AVG=%d, periodo=%d ms",
             (double)POT_TOTAL_OHM, VCC_MV, SAMPLES_AVG, LOOP_PERIOD_MS);

    // ---------- Bucle de lectura ----------
    while (1) {
        int mv_acc = 0;
        int raw_last = 0;

        for (int i = 0; i < SAMPLES_AVG; ++i) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CH, &raw_last));
            int mv = 0;
            if (has_cali) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, raw_last, &mv));
            } else {
                // Fallback aproximado si no hay eFuse de calibración
                mv = (int)((raw_last * (int64_t)VCC_MV) / 4095); // aprox 12-bit
            }
            mv_acc += mv;
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        const int vmv = mv_acc / SAMPLES_AVG;
        float Rtop = 0.0f, Rbot = 0.0f, pos = 0.0f;
        pot_resistances_from_voltage(vmv, &Rtop, &Rbot, &pos);

        ESP_LOGI(TAG, "RAW=%d  V=%dmV (%.3f V)  R_top=%.1f Ω  R_bottom=%.1f Ω  Pos=%.1f%%",
                 raw_last, vmv, (double)mv_to_volt(vmv), (double)Rtop, (double)Rbot, (double)pos);

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    // Limpieza (no se suele alcanzar)
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1));
    if (has_cali) adc_cali_deinit(cali);
}

// ---------------- Calibración ADC ----------------
static bool adc_cali_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                          adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración ADC: Curve Fitting");
        adc_cali_curve_fitting_config_t cfg = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración ADC: Line Fitting");
        adc_cali_line_fitting_config_t cfg = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibración OK");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "Sin eFuse para calibración: usando mapeo aproximado.");
    } else {
        ESP_LOGE(TAG, "Error de calibración (arg inválido o falta memoria)");
    }
    return calibrated;
}

static void adc_cali_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando calibración: Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando calibración: Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
