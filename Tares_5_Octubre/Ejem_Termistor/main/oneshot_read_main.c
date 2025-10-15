/*
 * Termistor NTC 10k con ADC oneshot (ESP-IDF v5+)
 * Lee mV calibrados, promedia y convierte a °C (modelo Beta).
 * 
 * Conecta 3V3 - R_FIXED(10k) -●- NTC(10k@25°C) - GND, y ● al canal ADC1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// --------- Configuración NTC / divisor ----------
#define VCC_MV          3300            // Alimentación del divisor (mV)
#define R_FIXED_OHM     1000           // Resistencia fija (10kΩ)
#define NTC_R0_OHM      10000           // NTC a 25°C
#define NTC_T0_K        298.15f         // 25°C en Kelvin
#define NTC_BETA        3950.0f         // Ajusta según tu datasheet (p.ej. 3950)

#define SAMPLES_AVG     16              // Promedio de lecturas
#define LOOP_PERIOD_MS  100             // Periodo de impresión (10 Hz aprox.)

// --------- ADC / canal ----------
#if CONFIG_IDF_TARGET_ESP32
// ADC1_CH4 == GPIO32 (recomendado)
#   define EXAMPLE_ADC1_CHAN0      ADC_CHANNEL_4
#   define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_11   // ~0-3.3V (ESP32 usa 11 dB)
#else
// En S2/S3/C3, ADC1_CH2 suele mapear a un pin apto (revisa tu board)
#   define EXAMPLE_ADC1_CHAN0      ADC_CHANNEL_2
#   define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12   // Cobertura hasta ~3.5V
#endif

static const char *TAG = "NTC_TEMP";

// Prototipos calibración
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Conversión mV -> °C (modelo Beta)
static float ntc_temp_from_mv(int vmv) {
    if (vmv <= 0)       vmv = 1;
    if (vmv >= VCC_MV)  vmv = VCC_MV - 1;

    const float V     = (float)vmv;
    const float Rntc  = (float)R_FIXED_OHM * (V / (float)(VCC_MV - V));   // R_NTC = Rf * V/(Vcc-V)

    const float invT  = (1.0f/NTC_T0_K) + (1.0f/NTC_BETA) * logf(Rntc / (float)NTC_R0_OHM);
    const float T_K   = 1.0f / invT;
    return T_K - 273.15f; // °C
}

void app_main(void)
{
    // ---------- Crear unidad ADC1 ----------
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ---------- Config canal ----------
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT, // usa el por defecto del chip
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &chan_cfg));

    // ---------- Calibración ----------
    adc_cali_handle_t adc1_cali = NULL;
    const bool do_cal = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0,
                                                     EXAMPLE_ADC_ATTEN, &adc1_cali);

    ESP_LOGI(TAG, "Inicio OK. Promedio=%d, Periodo=%d ms, VCC=%dmV, Rf=%dΩ, Beta=%.0f",
             SAMPLES_AVG, LOOP_PERIOD_MS, VCC_MV, R_FIXED_OHM, (double)NTC_BETA);

    // ---------- Bucle de medida ----------
    while (1) {
        int mv_acc = 0;
        int raw_last = 0;

        for (int i = 0; i < SAMPLES_AVG; ++i) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &raw_last));

            int mv = 0;
            if (do_cal) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali, raw_last, &mv));
            } else {
                // Aproximación si no hay eFuse de calibración disponible (mejor siempre calibrar)
                // Nota: para distintos chips/atenuaciones el rango/escala varía; usa solo como fallback.
                mv = (int)((raw_last * (int64_t)VCC_MV) / 4095); // suponer 12-bit
            }

            mv_acc += mv;
            vTaskDelay(pdMS_TO_TICKS(2)); // pequeño espaciamiento anti-ruido
        }

        const int vmv = mv_acc / SAMPLES_AVG;
        const float temp_c = ntc_temp_from_mv(vmv);

        // (Opcional) calcula e imprime resistencia estimada
        const float Rntc = (float)R_FIXED_OHM * ((float)vmv / (float)(VCC_MV - vmv));

        ESP_LOGI(TAG, "RAW=%d  V=%dmV  Rntc=%.1fΩ  Temp=%.2f°C",
                 raw_last, vmv, (double)Rntc, (double)temp_c);

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }

    // ---------- Nunca llega aquí, pero por limpieza ----------
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_cal) example_adc_calibration_deinit(adc1_cali);
}

/*-----------------------------
        Calibración ADC
------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibración OK");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse no soporta calibración: usando lectura sin calibrar (aprox).");
    } else {
        ESP_LOGE(TAG, "Error de calibración (arg inválido o sin memoria)");
    }
    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando calibración: Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando calibración: Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

