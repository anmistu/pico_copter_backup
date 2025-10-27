#include "ina219.hpp"
#include <cmath>
#include <algorithm>

// ====== INA219 レジスタ ======
static constexpr uint8_t REG_CONFIG      = 0x00;
static constexpr uint8_t REG_SHUNT_V     = 0x01; // 未使用（電流は測らない運用）
static constexpr uint8_t REG_BUS_V       = 0x02;
static constexpr uint8_t REG_CALIBRATION = 0x05; // 未使用

// ====== CONFIG ビット（データシート準拠） ======
// BRNG: 1=32V
static constexpr uint16_t CFG_BRNG_32V = (1u << 13);

// PG: ±320mV（最大）
static constexpr uint16_t CFG_PG_320mV = (0x3u << 11);

// ADC: 12bit/平均（0x0B=12bit x 32 samples）
static constexpr uint16_t ADC_12BIT_32S = 0x0Bu;

// MODE: Shunt+Bus, Continuous
static constexpr uint16_t MODE_SHUNT_BUS_CONT = 0x7u;

// ---- 内部状態 ----
static i2c_inst_t* g_i2c  = i2c1;
static uint8_t     g_addr = 0x40;   // 7bit

static volatile float g_vbus_V      = 0.0f; // 生値
static volatile float g_vbus_filt_V = 0.0f; // IIR
static float g_vref_V   = 11.1f;            // 名目（3S）
static float g_alpha    = 0.10f;            // IIR係数

static float g_warn_V   = 11.2f;            // Warn しきい値
static float g_crit_V   = 10.8f;            // Critical しきい値

static bool  g_flag_warn = false;
static bool  g_flag_crit = false;
static bool  g_flag_sag  = false;

static float g_prev_filt = 0.0f;
static uint64_t g_prev_us = 0;

// I2C ユーティリティ（BE 16bit）
static inline void reg_write_u16(uint8_t reg, uint16_t val_be) {
    uint8_t buf[3] = {reg, static_cast<uint8_t>(val_be >> 8), static_cast<uint8_t>(val_be & 0xFF)};
    i2c_write_blocking(g_i2c, g_addr, buf, 3, false);
}
static inline uint16_t reg_read_u16(uint8_t reg) {
    i2c_write_blocking(g_i2c, g_addr, &reg, 1, true);
    uint8_t buf[2] = {0,0};
    i2c_read_blocking(g_i2c, g_addr, buf, 2, false);
    return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
}

// BusVoltage 読み出し（LSB=4mV, 下位3bitはステータス→右3bitシフト）
static float read_bus_voltage_V() {
    uint16_t raw = reg_read_u16(REG_BUS_V);
    uint16_t v13 = (raw >> 3) & 0x1FFF; // 13bit
    return static_cast<float>(v13) * 0.004f; // 4mV LSB
}

void ina219_setup(i2c_inst_t* i2c, uint8_t addr7) {
    g_i2c  = i2c;
    g_addr = addr7;

    // Config 設定：32Vレンジ, ±320mV, 12bit×32平均（Bus/Shuntとも）, 連続測定
    uint16_t config =
        CFG_BRNG_32V |
        CFG_PG_320mV |
        (static_cast<uint16_t>(ADC_12BIT_32S) << 7) |   // BADC
        (static_cast<uint16_t>(ADC_12BIT_32S) << 3) |   // SADC
        MODE_SHUNT_BUS_CONT;

    reg_write_u16(REG_CONFIG, config);

    // 初期読みでフィルタを安定化
    g_vbus_V      = read_bus_voltage_V();
    g_vbus_filt_V = g_vbus_V;
    g_prev_filt   = g_vbus_filt_V;
    g_prev_us     = time_us_64();

    // 電流は使わないのでキャリブレーションは未設定でOK
}

void ina219_poll() {
    const float v = read_bus_voltage_V();
    g_vbus_V = v;

    // IIR
    const float alpha = std::clamp(g_alpha, 0.01f, 0.5f);
    g_vbus_filt_V = (1.0f - alpha) * g_vbus_filt_V + alpha * v;

    // 低電圧フラグ
    g_flag_warn = (g_vbus_filt_V <= g_warn_V);
    g_flag_crit = (g_vbus_filt_V <= g_crit_V);

    // サグ検出（dV/dt）
    const uint64_t now = time_us_64();
    const float dt_s = (now > g_prev_us) ? (static_cast<float>(now - g_prev_us) * 1e-6f) : 0.0f;
    if (dt_s > 0.0f) {
        const float dv = g_vbus_filt_V - g_prev_filt;
        const float dvdt = dv / dt_s; // V/s
        // 急降下を閾値に
        g_flag_sag = (dvdt < -0.15f); // 例：-0.15 V/s より速い降下
    }
    g_prev_filt = g_vbus_filt_V;
    g_prev_us   = now;
}

float ina219_get_vbus_V()       { return g_vbus_V; }
float ina219_get_vbus_filt_V()  { return g_vbus_filt_V; }

// 補償係数：Vref / V_filt（過補償を抑えるためクランプ）
float ina219_get_comp_factor() {
    // volatile をローカルに退避してから処理（std::max との型不一致を回避）
    float vf = g_vbus_filt_V;
    if (vf < 9.0f) vf = 9.0f;   // 下限9Vのクランプ

    float c  = g_vref_V / vf;
    return std::clamp(c, 0.85f, 1.25f);
}


void  ina219_set_vref_V(float vref_V)                 { g_vref_V = vref_V; }
void  ina219_set_thresholds(float warn_V, float crit_V){ g_warn_V = warn_V; g_crit_V = crit_V; }
void  ina219_set_iir_alpha(float alpha)               { g_alpha  = alpha; }
void  ina219_get_flags(bool* warn, bool* crit, bool* sag) {
    if (warn) *warn = g_flag_warn;
    if (crit) *crit = g_flag_crit;
    if (sag)  *sag  = g_flag_sag;
}