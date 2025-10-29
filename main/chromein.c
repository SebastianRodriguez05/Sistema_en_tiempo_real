#include "chromein.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>

static bool _cali_init(adc_unit_t unit, adc_atten_t att, adc_cali_handle_t *out)
{
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = unit,
        .atten   = att,
        .bitwidth= ADC_BITWIDTH_DEFAULT
    };
    adc_cali_handle_t h = NULL;
    if (adc_cali_create_scheme_curve_fitting(&cfg, &h) == ESP_OK) {
        *out = h; return true;
    }
    return false;
}

esp_err_t chromein_init(ChromeInCtx *ctx)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;
    *ctx = (ChromeInCtx){0};

    // Unidad ADC1 (oneshot)
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &ctx->unit));

    // Canales con 12 dB (rango ~3.3 V)
    adc_oneshot_chan_cfg_t ch = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(ctx->unit, ADC_CHANNEL_0, &ch)); // POT
    ESP_ERROR_CHECK(adc_oneshot_config_channel(ctx->unit, ADC_CHANNEL_1, &ch)); // NTC

    // Calibración (si hay eFuses)
    ctx->cal_ok_pot = _cali_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &ctx->cali_pot);
    ctx->cal_ok_ntc = _cali_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &ctx->cali_ntc);
    return ESP_OK;
}

esp_err_t chromein_read_all(ChromeInCtx *ctx, ChromeInData *out)
{
    if (!ctx || !out) return ESP_ERR_INVALID_ARG;

    int raw_pot=0, raw_ntc=0;
    int mv_pot=0,  mv_ntc=0;

    ESP_ERROR_CHECK(adc_oneshot_read(ctx->unit, ADC_CHANNEL_0, &raw_pot));
    ESP_ERROR_CHECK(adc_oneshot_read(ctx->unit, ADC_CHANNEL_1, &raw_ntc));

    if (ctx->cal_ok_pot) ESP_ERROR_CHECK(adc_cali_raw_to_voltage(ctx->cali_pot, raw_pot, &mv_pot));
    else mv_pot = raw_pot;   // fallback aproximado
    if (ctx->cal_ok_ntc) ESP_ERROR_CHECK(adc_cali_raw_to_voltage(ctx->cali_ntc, raw_ntc, &mv_ntc));
    else mv_ntc = raw_ntc;

    float r = chromein_ntc_res_from_mv((uint16_t)mv_ntc);
    float t = chromein_ntc_temp_c_from_res(r);

    out->pot_mv = (uint16_t)mv_pot;
    out->ntc_mv = (uint16_t)mv_ntc;
    out->temp_c = t;
    return ESP_OK;
}

// ----- Helpers -----
float chromein_ntc_res_from_mv(uint16_t mv_node)
{
    float v = (float)mv_node;
    if (v <= 0.0f) v = 0.1f;
    if (v >= V_SUPPLY_MV - 0.1f) v = V_SUPPLY_MV - 0.1f;
    // Vnode = Vs * (Rntc / (Rseries + Rntc)) => Rntc = Rseries * Vnode / (Vs - Vnode)
    return R_SERIES_OHM * v / (V_SUPPLY_MV - v);
}

float chromein_ntc_temp_c_from_res(float r_ntc)
{
    if (r_ntc < 1.0f) r_ntc = 1.0f;
    float invT = (1.0f / T0_K) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R25_OHM);
    return (1.0f / invT) - 273.15f;
}
