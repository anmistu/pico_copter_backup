#pragma once
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ---- Public API ----
// 初期化：I2C1共有（SDA=GP26, SCL=GP27）・アドレス既定0x40
void  ina219_setup(i2c_inst_t* i2c = i2c1, uint8_t addr7 = 0x40);

// 周期更新：25〜50Hz程度で呼ぶ（I2Cレジスタから最新BusVoltageを取得＆IIRフィルタ＆フラグ更新）
void  ina219_poll();

// 取得系
float ina219_get_vbus_V();        // 生の計測電圧[V]
float ina219_get_vbus_filt_V();   // IIR後の平滑電圧[V]
float ina219_get_comp_factor();   // 電圧補償係数 = Vref / V_filt（クランプ済み）

// 設定系
void  ina219_set_vref_V(float vref_V);                 // 名目電圧（例：3Sなら11.1V）
void  ina219_set_thresholds(float warn_V, float crit_V); // 低電圧しきい値（例：11.2V, 10.8V）
void  ina219_set_iir_alpha(float alpha);               // IIR係数（0.05〜0.2推奨）

// 状態フラグ（true/false）
void  ina219_get_flags(bool* warn, bool* crit, bool* sag);