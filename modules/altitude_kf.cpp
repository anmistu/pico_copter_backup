#include "altitude_kf.hpp"
#include <math.h>

static float h=0.01f;             // KF内部の固定周期（100 Hz）
static float z=0.0f, vz=0.0f;     // 状態
static float P11=1e-2f, P12=0.0f, P21=0.0f, P22=1e-2f;  // 共分散
static float Qz=1e-4f, Qv=0.25f;  // プロセス雑音 z[m], v[m/s]
static float Rz=0.0004f;          // 観測雑音 z[m]^2

void altkf_init(float h_fixed, float q_z, float q_v, float r_z){
  h  = h_fixed; Qz = q_z; Qv = q_v; Rz = r_z;
  altkf_reset(0.0f,0.0f);
}
void altkf_reset(float z0_m, float vz0_mps){
  z=z0_m; vz=vz0_mps;
  P11=1e-2f; P22=1e-2f; P12=P21=0.0f;
}

void altkf_step_acc(float a_wz_mps2){
  // 予測：x = A x + B a,  A=[[1,h],[0,1]], B=[0.5*h^2; h]
  z  = z  + h*vz + 0.5f*h*h*a_wz_mps2;
  vz = vz + h*a_wz_mps2;
  // P = A P A' + Q
  const float nP11 = P11 + h*(P12+P21) + h*h*P22 + Qz;
  const float nP12 = P12 + h*P22;
  const float nP21 = P21 + h*P22;
  const float nP22 = P22 + Qv;
  P11=nP11; P12=nP12; P21=nP21; P22=nP22;
}

void altkf_update_z(float z_meas_m){
  // 観測：y = [1 0] x + v
  const float y_pred = z;
  const float S = P11 + Rz;
  if (S <= 0.0f) return;
  const float K1 = P11 / S;
  const float K2 = P21 / S;
  const float innov = z_meas_m - y_pred;
  z  += K1 * innov;
  vz += K2 * innov;
  // P ← (I-KH)P
  const float t11 = (1.0f - K1)*P11;  const float t12 = (1.0f - K1)*P12;
  const float t21 =      - K2 *P11;   const float t22 =      - K2 *P12 + P22;
  P11=t11; P12=t12; P21=t21; P22=t22;
}

AltKFState altkf_get(){ AltKFState s{z,vz}; return s; }
