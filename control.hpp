#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <stdio.h>
#include "pico_copter.hpp"
#include "pico/stdlib.h"
#include "sensor.hpp"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <Eigen/Dense>
#include "modules/rgbled/rgbled.hpp"
#include "ekf.hpp"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

#define BATTERY_VOLTAGE (18.5)


// ===== Altitude Hold thin-layer (CH5でON/OFF) =====
struct AltHoldParams {
  float z_deadband_m;    // デッドバンド(±m)
  float ki;              // Iゲイン [duty/(m*s)]
  float duty_bias_min;   // dutyバイアス下限
  float duty_bias_max;   // dutyバイアス上限
  float i_clamp;         // 積分飽和(安全)
  float tof_timeout_s;   // ToF喪失でOFFまでの猶予[s]
  float bias_rise_rate;  // bias立上り速度[duty/s]
  float bias_fall_rate;  // bias立下り速度[duty/s]
};

extern volatile bool g_alt_hold;  // 高度ホールドON/OFF
extern AltHoldParams g_ahp;

// ★ 追加：二重PIDとKF出力の共有
extern float g_z_hat_m;            // KF高さ[m]
extern float g_vz_hat_mps;         // KF鉛直速度[m/s]
extern float g_v_ref_mps;          // 外側PIDが出す目標速度[m/s]

void  alt_hold_reset();
void  alt_hold_set_ref(float z_ref_m);
void  alt_hold_on();
void  alt_hold_off();
float alt_hold_apply(float dt, float base_duty, bool thr_low,
                     bool tof_valid, float z_meas_m, float *out_bias);
float alt_hold_get_ref();


//グローバル関数の宣言
void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);
void cam_link_touch();

//グローバル変数
extern uint8_t LockMode;
extern volatile uint8_t Logoutputflag;


class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_filter_time_constant;
    float m_err,m_err2,m_err3;
    float m_h;
  public:
    float m_filter_output;
    float m_integral;
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float filter_time_constant, 
        float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    float filter(float x);
    float update(float err);
};

class Filter
{
  private:
    float m_state;
    float m_T;
    float m_h;
  public:
    float m_out;
    Filter();
    void set_parameter(
        float T,
        float h);
    void reset(void);
    float update(float u);
};

// === Follow (USB dx,dy,depth) shared variables ===
extern volatile float     g_follow_dx;          // [px]
extern volatile float     g_follow_dy;          // [px]
extern volatile float     g_follow_depth_m;     // [m]
extern volatile uint32_t  g_follow_last_us;     // [us] timestamp of last USB update
extern volatile bool      g_follow_data_valid;  // true if last parse OK
extern volatile bool      g_follow_enabled;     // CH5 AltHold ON → true, OFF → false


#endif