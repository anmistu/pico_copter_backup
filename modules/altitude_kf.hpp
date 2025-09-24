#pragma once
#include <stdint.h>

struct AltKFState {
  float z_m;     // 推定高さ [m]
  float vz_mps;  // 推定鉛直速度 [m/s]
};

void altkf_init(float h_fixed = 0.01f, float q_z=1e-4f, float q_v=0.25f, float r_z=0.0004f);
void altkf_reset(float z0_m, float vz0_mps=0.0f);
void altkf_step_acc(float a_wz_mps2);                 // 予測（加速度のみ） h_fixedで1ステップ
void altkf_update_z(float z_meas_m);                  // 観測（ToF有効時のみ）
AltKFState altkf_get();
