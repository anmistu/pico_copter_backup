// control.cpp
//
// preset(CH5)中も姿勢補正(PID)が効くように修正
// - 平均だけプリセット値に合わせるΔ方式
// - 実効dutyで安全/角度PIDの判定
// - ログに最終4輪出力を追加 (DATANUM=44)
//
// + ToF(VL53L1X)最小統合（読み取りのみ）
//   - Core1(angle_control)ループで非ブロッキングpoll
//   - 共有変数 g_tof_valid / g_tof_mm に最新値を保存
//   - ログ列の末尾(#45)に ToF[mm] を追加（無効時は -1）
//   - ToFの初期化(tof_setup)は main 側で実行してください

#include "control.hpp"
#include "modules/tof/tof_bridge.hpp"
#include "pico/stdlib.h"
#include "modules/altitude_kf.hpp"      // ★追加
#include "modules/ina219.hpp"

#ifndef CAM_TIMEOUT_MS
#define CAM_TIMEOUT_MS 800   // 受信がこのミリ秒以上途絶→リンクダウン扱い
#endif

static uint32_t g_cam_last_ms  = 0;   // 最終受信時刻[ms]
static bool     g_cam_link_down = true; // 起動直後は未接続扱い

// ====== Follow control parameters ======
volatile float     g_follow_dx         = 0.0f;
volatile float     g_follow_dy         = 0.0f;
volatile float     g_follow_depth_m    = 0.0f;
volatile uint32_t  g_follow_last_us    = 0;
volatile bool      g_follow_data_valid = false;
volatile bool      g_follow_enabled    = false;
volatile float     g_follow_rx_hz      = 0.0f;

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }

// Target distance [m]
static const float FOLLOW_DISTANCE_M   = 2.0f;
// Distance deadband [m]
static const float FOLLOW_DEADBAND_M   = 0.05f;
// Pitch command limit [rad] (e.g., ±8 deg)
static const float FOLLOW_PITCH_MAX    = deg2rad(8.0f);
// Distance → pitch gain [rad/m] (1.0 m error => ~8 deg)
static const float FOLLOW_KP_DIST      = FOLLOW_PITCH_MAX / 1.0f;
// Slew per 100Hz update for pitch [rad] (e.g., 2 deg/update)
static const float FOLLOW_PITCH_SLEW   = deg2rad(2.0f);


// Yaw: dx full-scale and limits
static const float FOLLOW_DX_FS_PX     = 320.0f;   // tune to your camera width/2
static const float FOLLOW_DX_DB_PX     = 10.0f;    // deadband
static const float FOLLOW_YAW_RATE_MAX = deg2rad(80.0f); // [rad/s]

// USB data timeout to disable follow [us]
static const uint32_t FOLLOW_TIMEOUT_US = 200000;  // 0.2s
// === 二重PID 用 ===
PID   alt_pos_pid;                      // 外側：位置→速度
float g_z_hat_m   = 0.0f;               // KF推定
float g_vz_hat_mps= 0.0f;
float g_v_ref_mps = 0.0f;


// Sensor data
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;

// Times
float Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

// Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

// Control
float FR_duty, FL_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
const float Phi_trim   = 0.01f;
const float Theta_trim = 0.02f;
const float Psi_trim   = 0.0f;

// === Altitude Hold（ToFベース）===
PID           alt_pid;                 // 高さPID
volatile bool g_alt_hold       = false; // ON/OFF（control.hpp に extern があれば一致します）
static bool   g_alt_hold_prev  = false;
static float  g_alt_target_m   = 0.5f;  // ONした瞬間の高さ[m]
static float  g_alt_hover_base = 2.2f;  // [V] ON時のT_refを保持（非プリセット時用）
static float  g_alt_u_v        = 0.0f;  // PID出力[V]（デバッグ/解析用）


// Extended Kalman filter
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q;
Matrix<float, 6, 6> R;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> Beta;

// Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=23;                 // ★ 66
const uint32_t LOGDATANUM=48000;
float Logdata[LOGDATANUM]={0.0f};

// === Preset Fixed-Thrust Mode (CH5) ===
// 平均推力のみを合わせ、姿勢補正(差分)は保持する
static const float PRESET_DUTY      = 0.40f;                 // 目標平均duty
static const float PRESET_RAMP_SEC  = 0.30f;                 // ランプ時間[s]
static const float PRESET_RAMP_STEP = (1.0f/400.0f)/PRESET_RAMP_SEC;

static bool  g_preset_mode = false; // true while CH5 is ON
static bool  g_ch5_prev     = false; // edge detect
static float g_cmd_duty     = 0.0f;  // ramp中の目標平均duty
static float g_out_duty     = 0.0f;  // 出力したdutyの平均（ログ用）
// 実際にPWMへ出した各輪の最終値（ログ用）
static float g_out_fr = 0.0f;
static float g_out_fl = 0.0f;
static float g_out_rr = 0.0f;
static float g_out_rl = 0.0f;

// === ToF 共有（他スレッドから読み取りのみ） ===
static volatile bool     g_tof_valid = false;
static volatile uint16_t g_tof_mm    = 0;    // [mm]

// State Machine
uint8_t LockMode=0;
float Disable_duty =0.10f;
float Flight_duty  =0.18f;
uint8_t OverG_flag = 0;

// PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
PID follow_pos_pid;
PID follow_yaw_pid;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
void log_output(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);

#define AVERAGE    2000
#define KALMANWAIT 6000


void cam_link_touch() {
  g_cam_last_ms   = to_ms_since_boot(get_absolute_time());
  g_cam_link_down = false;
}

void led_control(void)
{
  const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
  g_cam_link_down = (now_ms - g_cam_last_ms > CAM_TIMEOUT_MS);

  // プリセット中（CH5 ON）は今まで通り：青 or 赤でリンク状態表示
  if (g_preset_mode) {
    if (g_cam_link_down) rgbled_red();
    else                 rgbled_blue();
    return;
  }

  static uint16_t cnt = 0;

  if (Arm_flag == 0 || Arm_flag == 1) {
    // アーム前：待機表示
    rgbled_wait();
  }
  else if (Arm_flag == 2) {
    // フライト中：常時緑
    rgbled_green();
  }
  else if (Arm_flag == 3) {
    // ★ Kalman収束後 ＋ depthが届いているかをチェックして表示を変える

    // RealSense からの depth 情報が「有効で、かつ新しい」か？
    //   - g_follow_data_valid : Jetson側で正しくパースできたか
    //   - g_follow_last_us    : 最終受信時刻
    //   - FOLLOW_TIMEOUT_US   : 0.2秒以内なら fresh とみなす
    bool depth_fresh = g_follow_data_valid &&
                       (time_us_32() - g_follow_last_us <= FOLLOW_TIMEOUT_US);

    if (depth_fresh) {
      // depth 情報が来ている → 緑点滅で「追従準備OK」を示す
      if (cnt == 0)  rgbled_green();
      if (cnt == 50) rgbled_off();
      if (++cnt == 100) cnt = 0;
    } else {
      // depth まだ来ていない or 途絶 → 待機パターンにしておく
      rgbled_wait();
      cnt = 0; // 点滅フェーズはリセット
    }
  }
}

// Main loop (called from PWM interrupt @400Hz)
void loop_400Hz(void)
{
  static uint8_t led=1;
  S_time=time_us_32();

  // 割り込みフラグリセット
  pwm_clear_irq(3);

  led_control();

  if (Arm_flag==0)
  {
    // motor_stop();  // 完全停止は他分岐に委ね
    Elevator_center = 0.0f;
    Aileron_center  = 0.0f;
    Rudder_center   = 0.0f;
    Pbias = Qbias = Rbias = 0.0f;
    Phi_bias = Theta_bias = Psi_bias = 0.0f;
    return;
  }
  else if (Arm_flag==1)
  {
    motor_stop();
    // Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      // Sensor Read
      sensor_read();
      Aileron_center  += Chdata[3];
      Elevator_center += Chdata[1];
      Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if(BiasCounter<KALMANWAIT)
    {
      sensor_read();
      if(BiasCounter == AVERAGE)
      {
        Elevator_center /= AVERAGE;
        Aileron_center  /= AVERAGE;
        Rudder_center   /= AVERAGE;
        Pbias           /= AVERAGE;
        Qbias           /= AVERAGE;
        Rbias           /= AVERAGE;
        Mx_ave          /= AVERAGE;
        My_ave          /= AVERAGE;
        Mz_ave          /= AVERAGE;

        Xe(4,0) = Pbias; Xp(4,0) = Pbias;
        Xe(5,0) = Qbias; Xp(5,0) = Qbias;
        Xe(6,0) = Rbias; Xp(6,0) = Rbias;
        MN = Mx_ave; ME = My_ave; MD = Mz_ave;
      }

      AngleControlCounter++;
      if(AngleControlCounter==4){
        AngleControlCounter=0;
        sem_release(&sem);
      }
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias   = Phi_bias  / KALMANWAIT;
      Theta_bias = Theta_bias/ KALMANWAIT;
      Psi_bias   = Psi_bias  / KALMANWAIT;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1){
        LockMode=3; // Disable Flight
        led=0;
        return;
      }
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }

    if(Logflag==1&&LedBlinkCounter<100) LedBlinkCounter++;
    else { LedBlinkCounter=0; led=!led; }

    // Rate Control (400Hz)
    rate_control();

    // Angle Control (100Hz)
    if(AngleControlCounter==4){
      AngleControlCounter=0;
      sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1); LedBlinkCounter++;
    } else if(LedBlinkCounter<100){
      gpio_put(LED_PIN, 0); LedBlinkCounter++;
    } else LedBlinkCounter=0;

    // Get Stick Center
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];

    if(LockMode==0){
      if( lock_com()==1 ){ LockMode=1; return; }
    }
    else if(LockMode==1){
      if(lock_com()==0){ LockMode=2; Arm_flag=2; }
      return;
    }

    if(logdata_out_com()==1){ Arm_flag=4; return; }
  }
  else if(Arm_flag==4)
  {
    motor_stop();
    Logoutputflag=1;
    if(LedBlinkCounter<400) LedBlinkCounter++;
    else { LedBlinkCounter=0; led=!led; }
  }

  E_time=time_us_32();
  D_time=E_time-S_time;
}

void control_init(void)
{
  acc_filter.set_parameter(0.005f, 0.0025f);
  // Rate control
  p_pid.set_parameter(  1.7f, 0.20f, 0.020f, 0.015f, 0.0025f);
  q_pid.set_parameter(  1.7f, 0.20f, 0.020f, 0.015f, 0.0025f);
  r_pid.set_parameter( 14.0f, 0.50f, 0.0012f, 0.015f, 0.0025f);
  // Angle control
  phi_pid.set_parameter  ( 5.5f, 9.5f, 0.005f, 0.018f, 0.01f);
  theta_pid.set_parameter( 5.5f, 9.5f, 0.005f, 0.018f, 0.01f);
  psi_pid.set_parameter  ( 0.0f,10.0f, 0.010f, 0.030f, 0.01f);

  // ★ KF初期化（100 Hz想定）
  altkf_init(0.01f, /*Qz*/1e-4f, /*Qv*/0.25f, /*Rz*/0.0004f);
  altkf_reset(/*z0*/0.3f, /*vz0*/0.0f); // 適当に開始高さ。飛行開始時に上書きされます

  alt_pos_pid.set_parameter(1.0f, 100000.0f, 0.0f, 0.05f, 0.01f);
  alt_pos_pid.reset();

  // Altitude PID（入力: 高さ誤差[m] → 出力: 推力補正[V]）
  alt_pid.set_parameter( 1.7f, 1000.0f, 0.050f, 0.020f, 0.10f ); // まずはP主体
  alt_pid.reset();

  // ★ 追従：位置PI（距離[m] → ピッチ角[rad]）
  follow_pos_pid.set_parameter( 1.0f, 1000.0f ,0.07f ,0.020f ,0.01f);
  follow_pos_pid.reset();
  // ★ 追従：Yaw位置PI（dx正規化[-1..1] → 角速度[rad/s]）
  follow_yaw_pid.set_parameter( 1.85f, 1000000.0f ,0.05f ,0.020f ,0.01f);
  follow_yaw_pid.reset();

}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[2]<CH3MIN+80
   && Chdata[0]>CH1MAX-80
   && Chdata[3]<CH4MIN+80
   && Chdata[1]>CH2MAX-80)
  {
    chatta++;
    if(chatta>50){ chatta=50; state=1; }
  }
  else { chatta=0; state=0; }
  return state;
}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[4]<(CH5MAX+CH5MIN)*0.5
   && Chdata[2]<CH3MIN+80
   && Chdata[0]<CH1MIN+80
   && Chdata[3]>CH4MAX-80
   && Chdata[1]>CH2MAX-80)
  {
    chatta++;
    if(chatta>50){ chatta=50; state=1; }
  }
  else { chatta=0; state=0; }
  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0f);
  set_duty_fl(0.0f);
  set_duty_rr(0.0f);
  set_duty_rl(0.0f);
}

// CH5だけで「ログ＋プリセット（固定平均duty＋Δ）＋AltHold(高さPID)」を完結
// 2ポジ想定：CH5=OFF→手動、CH5=ON→プリセット＋AltHold＋ログ
// CH5だけで「ログ＋プリセット（固定平均duty＋Δ）＋AltHold(二重PID)」を完結
// 2ポジ想定：CH5=OFF→手動、CH5=ON→プリセット＋AltHold＋ログ


void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  // --- Read Sensor Value ---
  sensor_read();

  // --- Body rates ---
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  // --- Rate references (from angle_control) ---
  p_ref = Pref; q_ref = Qref; r_ref = Rref;

  // --- Throttle base (stick → Volt) ---
  T_ref = 0.6f * BATTERY_VOLTAGE * (float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

  // ===== CH5（2ポジ：ONでプリセット＋AltHold＋ログ＋追従を全部ON）=====
  const uint16_t ch5_mid = (CH5MIN + CH5MAX) * 0.5f;
  bool ch5_on = (Chdata[4] > ch5_mid);

  // 立ち上がり/立ち下がり検出
  bool rising_edge  = ( ch5_on && !g_ch5_prev );
  bool falling_edge = (!ch5_on &&  g_ch5_prev );
  g_ch5_prev = ch5_on;

  // プリセットON/OFF
  g_preset_mode = ch5_on;

  // ★ 追従ON/OFFをCH5に連動（これが追従の主スイッチ）
  g_follow_enabled = ch5_on;

  // プリセットに入った瞬間：現在平均から滑らかに移行開始
  if (rising_edge) {
    rgbled_blue();
    float avg_now = 0.25f*(FR_duty + FL_duty + RR_duty + RL_duty);
    g_cmd_duty = avg_now;
    // AltHoldターゲット高さを現推定値に
    g_alt_target_m = g_tof_valid ? (0.001f*(float)g_tof_mm) : g_z_hat_m;
    alt_pos_pid.reset();
    alt_pid.reset();
  }

  // AltHold 安全判定用 実効duty
  float eff_duty_for_alt = g_preset_mode ? g_cmd_duty : (T_ref / BATTERY_VOLTAGE);
  bool  alt_safe = (eff_duty_for_alt >= Flight_duty);

  // CH5 OFFで AltHold もOFF
  if (falling_edge) {
    g_alt_hold = false;
    alt_pos_pid.reset();
    alt_pid.reset();
    g_alt_u_v   = 0.0f;
    g_v_ref_mps = 0.0f;
  }

  // CH5 ON中は AltHold を許可（ゲートは下の分岐でチェック）
  if (ch5_on) g_alt_hold = true;
  // --- 二重（位置→速度）PID with KF（400Hz→100Hzに間引き） ---
  float preset_target_duty = PRESET_DUTY;
  static uint8_t alt_div = 0;
  if (g_alt_hold && alt_safe && g_tof_valid) {
    // 外側：高さ誤差
    float ez = g_alt_target_m - g_z_hat_m;
    if (fabsf(ez) < 0.03f) ez = 0.0f;

    if (++alt_div >= 4) alt_div = 0;
    if (alt_div == 0) {
      // v_ref [m/s] 生成（安全上限）
      g_v_ref_mps = alt_pos_pid.update(ez);
      const float VREF_MAX = 0.6f;
      if (g_v_ref_mps >  VREF_MAX) g_v_ref_mps =  VREF_MAX;
      if (g_v_ref_mps < -VREF_MAX) g_v_ref_mps = -VREF_MAX;

      // 内側：速度誤差 → 推力補正[V]
      float ev = g_v_ref_mps - g_vz_hat_mps;
      g_alt_u_v = alt_pid.update(ev);
      const float UV_MAX = 1.5f;
      if (g_alt_u_v >  UV_MAX){ g_alt_u_v =  UV_MAX; alt_pid.i_reset(); }
      if (g_alt_u_v < -UV_MAX){ g_alt_u_v = -UV_MAX; alt_pid.i_reset(); }
    }

    // 平均duty目標へ換算（“固定duty＋Δ”の形は維持）
    float u_duty = g_alt_u_v / BATTERY_VOLTAGE;
    preset_target_duty = PRESET_DUTY + u_duty;
    if (preset_target_duty < Disable_duty) preset_target_duty = Disable_duty;
    if (preset_target_duty > 0.95f)       preset_target_duty = 0.95f;
  } else {
    // ゲート外：AltHoldの内部状態をクリーン
    alt_pos_pid.reset();
    alt_pid.reset();
    g_alt_u_v   = 0.0f;
    g_v_ref_mps = 0.0f;
  }

  // --- レートPID（姿勢） ---
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  // --- Mixer（before clamp） ---

  float vbat_mixer = ina219_get_vbus_filt_V(); if (vbat_mixer < 9.0f) vbat_mixer = 9.0f;
  const float inv_vbat = 1.0f / vbat_mixer;

  FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25f)*inv_vbat;
  FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25f)*inv_vbat;
  RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25f)*inv_vbat;
  RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25f)*inv_vbat;

  // --- Clamp ---
  const float maximum_duty=0.95f;
  const float minimum_duty=Disable_duty;
  if (FR_duty < minimum_duty) FR_duty = minimum_duty; if (FR_duty > maximum_duty) FR_duty = maximum_duty;
  if (FL_duty < minimum_duty) FL_duty = minimum_duty; if (FL_duty > maximum_duty) FL_duty = maximum_duty;
  if (RR_duty < minimum_duty) RR_duty = minimum_duty; if (RR_duty > maximum_duty) RR_duty = maximum_duty;
  if (RL_duty < minimum_duty) RL_duty = minimum_duty; if (RL_duty > maximum_duty) RL_duty = maximum_duty;

  // --- Δ方式の最終化（プリセットONなら平均を g_cmd_duty に合わせる） ---
  float out_fr, out_fl, out_rr, out_rl;
  if (g_preset_mode) {
    const float diff = preset_target_duty - g_cmd_duty;
    if      (diff >  PRESET_RAMP_STEP) g_cmd_duty += PRESET_RAMP_STEP;
    else if (diff < -PRESET_RAMP_STEP) g_cmd_duty -= PRESET_RAMP_STEP;
    else                               g_cmd_duty  = preset_target_duty;

    const float avg   = 0.25f*(FR_duty + FL_duty + RR_duty + RL_duty);
    const float delta = g_cmd_duty - avg;
    out_fr = FR_duty + delta;
    out_fl = FL_duty + delta;
    out_rr = RR_duty + delta;
    out_rl = RL_duty + delta;

    const float DUTY_MIN = Disable_duty, DUTY_MAX = 0.95f;
    if (out_fr < DUTY_MIN) out_fr = DUTY_MIN; if (out_fr > DUTY_MAX) out_fr = DUTY_MAX;
    if (out_fl < DUTY_MIN) out_fl = DUTY_MIN; if (out_fl > DUTY_MAX) out_fl = DUTY_MAX;
    if (out_rr < DUTY_MIN) out_rr = DUTY_MIN; if (out_rr > DUTY_MAX) out_rr = DUTY_MAX;
    if (out_rl < DUTY_MIN) out_rl = DUTY_MIN; if (out_rl > DUTY_MAX) out_rl = DUTY_MAX;
  } else {
    out_fr = FR_duty; out_fl = FL_duty; out_rr = RR_duty; out_rl = RL_duty;
  }
  
  float vbat = ina219_get_vbus_filt_V();
  if (vbat < 9.0f) vbat = 9.0f;              // 下限クランプ（過補償防止）
  const float eff_duty = g_preset_mode ? g_cmd_duty : (T_ref / vbat);

  // --- 出力（フェイルセーフ付き） ---
  if (eff_duty < Disable_duty) {
    motor_stop();
    p_pid.reset(); q_pid.reset(); r_pid.reset();
    g_out_fr = g_out_fl = g_out_rr = g_out_rl = 0.0f;
    g_out_duty = 0.0f;
  } else {
    if (OverG_flag==0){
      g_out_fr = out_fr; g_out_fl = out_fl; g_out_rr = out_rr; g_out_rl = out_rl;
      set_duty_fr(out_fr); set_duty_fl(out_fl); set_duty_rr(out_rr); set_duty_rl(out_rl);
      g_out_duty = 0.25f*(out_fr+out_fl+out_rr+out_rl);
    } else {
      motor_stop();
      g_out_fr = g_out_fl = g_out_rr = g_out_rl = 0.0f;
      g_out_duty = 0.0f;
    }
  }
}

void angle_control(void)
{
  float phi_err, theta_err, psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;

  while (1)
  {
    // 100 Hz駆動（semはrate_control側から4回に1回ポスト）
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    S_time2 = time_us_32();

    // === 姿勢EKF更新 ===
    kalman_filter();

    // === 四元数→方向余弦成分 ===
    q0 = Xe(0,0); q1 = Xe(1,0); q2 = Xe(2,0); q3 = Xe(3,0);
    e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    e12 = 2.0f*(q1*q2 + q0*q3);
    e13 = 2.0f*(q1*q3 - q0*q2);
    e23 = 2.0f*(q2*q3 + q0*q1);
    e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // === 機体角 ===
    Phi   = atan2f(e23, e33);
    Theta = atan2f(-e13, sqrtf(e23*e23 + e33*e33));
    Psi   = atan2f(e12, e11);

    // === スティック→角度参照 ===
    Phi_ref   = Phi_trim   + 0.3f*M_PI * (float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5f) * 2.0f/(CH4MAX-CH4MIN);
    Theta_ref = Theta_trim + 0.3f*M_PI * (float)(Chdata[1] - (CH2MAX+CH2MIN)*0.5f) * 2.0f/(CH2MAX-CH2MIN);
    Psi_ref   = Psi_trim   + 0.8f*M_PI * (float)(Chdata[0] - (CH1MAX+CH1MIN)*0.5f) * 2.0f/(CH1MAX-CH1MIN);

    // === 追従共有：dx正規化[-1..1] ===
    // ★Yawのみ追従（Pitch/Roll追従は無効化）
    const bool follow_on    = g_follow_enabled;
    const bool follow_fresh = g_follow_data_valid &&
                              (time_us_32() - g_follow_last_us <= FOLLOW_TIMEOUT_US);

    float follow_ex = 0.0f;      // dx正規化 [-1..1]
    if (follow_on && follow_fresh) {
      float dx  = g_follow_dx;
      float sgn = (dx >= 0.0f) ? 1.0f : -1.0f;
      float mag = fabsf(dx) - FOLLOW_DX_DB_PX;
      if (mag < 0.0f) mag = 0.0f;
      float ex = (FOLLOW_DX_FS_PX > 1e-6f) ? (mag / FOLLOW_DX_FS_PX) : 0.0f; // 0..1
      if (ex > 1.0f) ex = 1.0f;
      follow_ex = ex * sgn;  // -1..1
    }

    // === 角度偏差 ===
    phi_err   = Phi_ref   - (Phi   - Phi_bias);
    theta_err = Theta_ref - (Theta - Theta_bias);
    psi_err   = Psi_ref   - (Psi   - Psi_bias);

    // 実効duty（preset時はg_cmd_duty、通常時はT_ref/BATTERY_VOLTAGE）
    const float eff_duty = g_preset_mode ? g_cmd_duty : (T_ref / BATTERY_VOLTAGE);

    // === 角度PID（安全ゲート付き）===
    if (eff_duty < Flight_duty)
    {
      Pref = Qref = Rref = 0.0f;
      phi_pid.reset(); theta_pid.reset(); psi_pid.reset();

      // スティック中心・角度バイアス更新
      Aileron_center  = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center   = Chdata[0];
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;

      // 追従PIDもリセット（Yawのみ使用）
      follow_yaw_pid.reset();
      follow_pos_pid.reset();   // ※将来のPitch追従復活に備えて残す（現状未使用）
    }
    else
    {
      // 外側Angle-PID → 内側Rate参照
      Pref =  phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = psi_err;  // Yawは角度誤差→角速度参照がベース

      // === Yaw：dx → 角速度（位置PI） ===
      float r_add = 0.0f;

      if (follow_on && follow_fresh) {
        // follow_ex [-1..1] をそのまま入力
        r_add = follow_yaw_pid.update(follow_ex);

        // 上限（±FOLLOW_YAW_RATE_MAX）＋簡易アンチワインドアップ
        if (r_add >  FOLLOW_YAW_RATE_MAX) { r_add =  FOLLOW_YAW_RATE_MAX; follow_yaw_pid.i_reset(); }
        if (r_add < -FOLLOW_YAW_RATE_MAX) { r_add = -FOLLOW_YAW_RATE_MAX; follow_yaw_pid.i_reset(); }
      } else {
        follow_yaw_pid.reset();
        r_add = 0.0f;
      }

      // 角速度参照に加算（内側r_pidが速度制御）
      Rref += r_add;
    }

    // === ToF（非ブロッキング）===
    {
      uint16_t zmm = 0;
      tof_poll();
      bool ok = tof_read_valid(&zmm);
      if (ok) { g_tof_valid = true;  g_tof_mm = zmm; }
      else    { g_tof_valid = false; }
    }

    {
      static uint8_t ina_cnt = 0;
      if (++ina_cnt >= 2) {    // angle_controlが100Hz想定 → 100/2=50Hz
        ina_cnt = 0;
        ina219_poll();
      }
    }

    // === 縦方向 1D-KF（100Hz）===
    {
      const float a_wz = e13*Ax + e23*Ay + e33*Az - 9.80665f; // +: 上向き
      altkf_step_acc(a_wz);
      if (g_tof_valid) {
        const float z_meas_m = 0.001f * (float)g_tof_mm;
        altkf_update_z(z_meas_m);
      }
      AltKFState s = altkf_get();
      g_z_hat_m    = s.z_m;
      g_vz_hat_mps = s.vz_mps;
    }

    // === ログ ===
    logging();

    E_time2 = time_us_32();
    D_time2 = E_time2 - S_time2;
  }
}





void logging(void)
{
  // CH5スイッチでログON
  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5f)
  {
    if(Logflag==0){ Logflag=1; LogdataCounter=0; }
    if(LogdataCounter+DATANUM<LOGDATANUM)
    {
      // Logdata[LogdataCounter++]=Xe(0,0);                  //  1
      // Logdata[LogdataCounter++]=Xe(1,0);                  //  2
      // Logdata[LogdataCounter++]=Xe(2,0);                  //  3
      // Logdata[LogdataCounter++]=Xe(3,0);                  //  4
      // Logdata[LogdataCounter++]=Xe(4,0);                  //  5
      // Logdata[LogdataCounter++]=Xe(5,0);                  //  6
      // Logdata[LogdataCounter++]=Xe(6,0);                  //  7

      // Logdata[LogdataCounter++]=Wp;                       //  8
      // Logdata[LogdataCounter++]=Wq;                       //  9
      // Logdata[LogdataCounter++]=Wr;                       // 10

      // Logdata[LogdataCounter++]=Ax;                       // 11
      // Logdata[LogdataCounter++]=Ay;                       // 12
      // Logdata[LogdataCounter++]=Az;                       // 13
      // Logdata[LogdataCounter++]=Mx;                       // 14
      // Logdata[LogdataCounter++]=My;                       // 15
      // Logdata[LogdataCounter++]=Mz;                       // 16

      // Logdata[LogdataCounter++]=Pref;                     // 17
      // Logdata[LogdataCounter++]=Qref;                     // 18
      // Logdata[LogdataCounter++]=Rref;                     // 19

      // Logdata[LogdataCounter++]=Phi-Phi_bias;             // 20 roll現在[rad]
      // Logdata[LogdataCounter++]=Theta-Theta_bias;         // 21 pitch現在[rad]
      // Logdata[LogdataCounter++]=Psi-Psi_bias;             // 22 yaw現在[rad]

      // Logdata[LogdataCounter++]=Phi_ref;                  // 23 roll目標[rad]
      // Logdata[LogdataCounter++]=Theta_ref;                // 24 pitch目標[rad]
      // Logdata[LogdataCounter++]=Psi_ref;                  // 25 yaw目標[rad]

      // Logdata[LogdataCounter++]=P_com;                    // 26
      // Logdata[LogdataCounter++]=Q_com;                    // 27
      // Logdata[LogdataCounter++]=R_com;                    // 28

      // Logdata[LogdataCounter++]=p_pid.m_integral;         // 29
      // Logdata[LogdataCounter++]=q_pid.m_integral;         // 30
      // Logdata[LogdataCounter++]=r_pid.m_integral;         // 31
      // Logdata[LogdataCounter++]=phi_pid.m_integral;       // 32
      // Logdata[LogdataCounter++]=theta_pid.m_integral;     // 33

      // Logdata[LogdataCounter++]=Pbias;                    // 34
      // Logdata[LogdataCounter++]=Qbias;                    // 35
      // Logdata[LogdataCounter++]=Rbias;                    // 36

      // Logdata[LogdataCounter++]=T_ref;                    // 37
      // Logdata[LogdataCounter++]=Acc_norm;                 // 38
      // Logdata[LogdataCounter++]=(g_preset_mode ? 1.0f : 0.0f); // 39 preset状態
      // Logdata[LogdataCounter++]=g_out_duty;               // 40 平均duty

      // // 最終PWM出力
      // Logdata[LogdataCounter++]=g_out_fr;                 // 41
      // Logdata[LogdataCounter++]=g_out_fl;                 // 42
      // Logdata[LogdataCounter++]=g_out_rr;                 // 43
      // Logdata[LogdataCounter++]=g_out_rl;                 // 44

      // ToF距離[mm]（無効時は -1）
      Logdata[LogdataCounter++]= g_tof_valid ? (float)g_tof_mm : -1.0f; // 45

      // 高度制御系
      Logdata[LogdataCounter++]= g_alt_hold ? 1.0f : 0.0f;  // 46 AltHold状態
      Logdata[LogdataCounter++]= g_tof_valid ? 1.0f : 0.0f; // 47 ToF有効
      Logdata[LogdataCounter++]= g_alt_u_v;                 // 48 AltPID出力[V]
      Logdata[LogdataCounter++]= g_cmd_duty;                // 49 目標平均duty
      Logdata[LogdataCounter++]= (g_preset_mode ? 1.0f:0.0f); // 50 presetフラグ(再)

      // 電圧センサ INA219
      Logdata[LogdataCounter++] = ina219_get_vbus_V();        // 51 Vbus_raw [V]
      Logdata[LogdataCounter++] = ina219_get_vbus_filt_V();   // 52 Vbus_filt [V]
      Logdata[LogdataCounter++] = ina219_get_comp_factor();   // 53 Vcomp (=Vref/Vfilt)
      
      bool w=false,c=false,s=false;
      ina219_get_flags(&w,&c,&s);
      Logdata[LogdataCounter++] = w ? 1.0f : 0.0f;          // 54 warn flag
      Logdata[LogdataCounter++] = c ? 1.0f : 0.0f;          // 55 crit flag
      Logdata[LogdataCounter++] = s ? 1.0f : 0.0f;          // 56 sag  flag

      // ==== 追従：姿勢誤差 ======================================
      float roll_err  = Phi_ref   - (Phi   - Phi_bias);
      float pitch_err = Theta_ref - (Theta - Theta_bias);
      float yaw_err   = Psi_ref   - (Psi   - Psi_bias);

      Logdata[LogdataCounter++] = roll_err;                 // 57 roll error [rad]
      Logdata[LogdataCounter++] = pitch_err;                // 58 pitch error [rad]
      Logdata[LogdataCounter++] = yaw_err;                  // 59 yaw   error [rad]

      // 追従ONフラグとデータfreshフラグ
      bool follow_on    = g_follow_enabled;
      bool follow_fresh = g_follow_data_valid &&
                          (time_us_32() - g_follow_last_us <= FOLLOW_TIMEOUT_US);

      Logdata[LogdataCounter++] = follow_on    ? 1.0f : 0.0f; // 60 follow_enabled (CH5)
      Logdata[LogdataCounter++] = follow_fresh ? 1.0f : 0.0f; // 61 follow_fresh (USB新鮮)

      // ==== follow_target.py 用の追従ログ ======================
      // 距離[m]（無効時は -1）
      float follow_depth_m = g_follow_data_valid ? g_follow_depth_m : -1.0f;
      Logdata[LogdataCounter++] = follow_depth_m;           // 62: follow_depth_m [m]

      // 目標距離[m]（一定：FOLLOW_DISTANCE_M）
      Logdata[LogdataCounter++] = FOLLOW_DISTANCE_M;        // 63: follow_target_m [m]

      // 距離誤差[m] = target - current（データ無効時は0）
      float e_pos_follow = 0.0f;
      if (g_follow_data_valid) {
        e_pos_follow = FOLLOW_DISTANCE_M - g_follow_depth_m;
      }
      Logdata[LogdataCounter++] = e_pos_follow;             // 64: e_pos [m]

      // 画面上の横ずれ[px]（無効時は0）
      float dx_px = g_follow_data_valid ? g_follow_dx : 0.0f;
      Logdata[LogdataCounter++] = dx_px;                    // 65: dx_px [px]

      // 正規化された横誤差 ex_dx [-1..1]
      float ex_dx = 0.0f;
      if (g_follow_data_valid) {
        float dx    = g_follow_dx;
        float mag   = fabsf(dx) - FOLLOW_DX_DB_PX;
        if (mag < 0.0f) mag = 0.0f;
        if (FOLLOW_DX_FS_PX > 1e-6f) {
          float tmp = mag / FOLLOW_DX_FS_PX;  // 0..1
          if (tmp > 1.0f) tmp = 1.0f;
          float sgn = (dx >= 0.0f) ? 1.0f : -1.0f;
          ex_dx = sgn * tmp;                  // -1..1
        }
      }
      Logdata[LogdataCounter++] = ex_dx;                   // 66: ex_dx [-1..1]

      // 受信Hz（データがfreshでない時は 0）
      float rx_hz = follow_fresh ? g_follow_rx_hz : 0.0f;
      Logdata[LogdataCounter++] = rx_hz;                   // 67: follow_rx_hz [Hz]

    }
    else Logflag=2;
  }
  else
  {
    if(Logflag>0){ Logflag=0; LogdataCounter=0; }
  }
}



void log_output(void)
{
  if(LogdataCounter==0)
  {
    printPQR();
    printf("#Roll rate PID gain\n");  p_pid.printGain();
    printf("#Pitch rate PID gain\n"); q_pid.printGain();
    printf("#Yaw rate PID gain\n");   r_pid.printGain();
    printf("#Roll angle PID gain\n"); phi_pid.printGain();
    printf("#Pitch angle PID gain\n");theta_pid.printGain();
  }
  if(LogdataCounter+DATANUM<LOGDATANUM)
  {
    printf("%10.2f ", Log_time);
    Log_time += 0.01f;
    for (uint8_t i=0;i<DATANUM;i++){
      printf("%12.5f",Logdata[LogdataCounter+i]);
    }
    printf("\n");
    LogdataCounter += DATANUM;
  }
  else
  {
    Arm_flag=3;
    Logoutputflag=0;
    LockMode=0;
    Log_time=0.0f;
    LogdataCounter=0;
  }
}

void gyroCalibration(void)
{
  float sump=0.0f,sumq=0.0f,sumr=0.0f;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump+=Wp; sumq+=Wq; sumr+=Wr;
  }
  Pbias=sump/N; Qbias=sumq/N; Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax =-acceleration_mg[0]*GRAV*0.001f;
  Ay =-acceleration_mg[1]*GRAV*0.001f;
  Az = acceleration_mg[2]*GRAV*0.001f;
  Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6f;
  Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6f;
  Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6f;
  Mx0 =-magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 =-magnetic_field_mgauss[2];

  acc_norm = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>250.0f) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrtf(Wp*Wp + Wq*Wq + Wr*Wr);
  if (rate_norm > 6.0f) OverG_flag = 1;

  // 地磁気キャリブレーション（あなたの係数をそのまま使用）
  const float rot[9]={-0.78435472f, -0.62015392f, -0.01402787f,
                       0.61753358f, -0.78277935f,  0.07686857f,
                      -0.05865107f,  0.05162955f,  0.99694255f};
  const float center[3]={-109.32529343620176f, 72.76584808916506f, 759.2285249891385f};
  const float zoom[3]={0.002034773458122364f, 0.002173892202021849f, 0.0021819494099235273f};

  // 回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Mx0 +rot[1]*My0 +rot[2]*Mz0 -center[0]);
  my1 = zoom[1]*( rot[3]*Mx0 +rot[4]*My0 +rot[5]*Mz0 -center[1]);
  mz1 = zoom[2]*( rot[6]*Mx0 +rot[7]*My0 +rot[8]*Mz0 -center[2]);
  // 逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1;

  mag_norm=sqrtf(Mx*Mx +My*My +Mz*Mz);
  Mx/=mag_norm; My/=mag_norm; Mz/=mag_norm;
}

void variable_init(void)
{
  // Variable Initialize
  Xe << 1.00f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  Xp =Xe;

  Q <<  6.0e-5f, 0.0f   , 0.0f   , 0.0f   , 0.0f   , 0.0f,
        0.0f   , 5.0e-5f, 0.0f   , 0.0f   , 0.0f   , 0.0f,
        0.0f   , 0.0f   , 2.8e-5f, 0.0f   , 0.0f   , 0.0f,
        0.0f   , 0.0f   , 0.0f   , 5.0e-5f, 0.0f   , 0.0f,
        0.0f   , 0.0f   , 0.0f   , 0.0f   , 5.0e-5f, 0.0f,
        0.0f   , 0.0f   , 0.0f   , 0.0f   , 0.0f   , 5.0e-5f;

  R <<  1.701e0f, 0.0f     , 0.0f     , 0.0f , 0.0f , 0.0f,
        0.0f    , 2.799e0f , 0.0f     , 0.0f , 0.0f , 0.0f,
        0.0f    , 0.0f     , 1.056e0f , 0.0f , 0.0f , 0.0f,
        0.0f    , 0.0f     , 0.0f     , 2.3e-1f, 0.0f, 0.0f,
        0.0f    , 0.0f     , 0.0f     , 0.0f , 1.4e-1f, 0.0f,
        0.0f    , 0.0f     , 0.0f     , 0.0f , 0.0f , 0.49e-1f;

  G <<   1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 0.0f,
        -1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
        -1.0f, -1.0f,  1.0f, 0.0f, 0.0f, 0.0f,
         1.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
         0.0f,  0.0f,  0.0f, 1.0f, 0.0f, 0.0f,
         0.0f,  0.0f,  0.0f, 0.0f, 1.0f, 0.0f,
         0.0f,  0.0f,  0.0f, 0.0f, 0.0f, 1.0f;

  G = G*0.01f;
  Beta << 0.0f, 0.0f, 0.0f;

  P <<   1e0f, 0, 0, 0, 0, 0, 0,
          0  , 1e0f,0, 0, 0, 0, 0,
          0  , 0, 1e0f,0, 0, 0, 0,
          0  , 0, 0, 1e0f,0, 0, 0,
          0  , 0, 0, 0, 1e0f,0, 0,
          0  , 0, 0, 0, 0, 1e0f,0,
          0  , 0, 0, 0, 0, 0, 1e0f;
}

void printPQR(void)
{
  volatile int m=0, n=0;
  // Print P
  printf("#P\n");
  for (m=0;m<7;m++){
    printf("# ");
    for (n=0;n<7;n++) printf("%12.4e ",P(m,n));
    printf("\n");
  }
  // Print Q
  printf("#Q\n");
  for (m=0;m<6;m++){
    printf("# ");
    for (n=0;n<6;n++) printf("%12.4e ",Q(m,n));
    printf("\n");
  }
  // Print R
  printf("#R\n");
  for (m=0;m<6;m++){
    printf("# ");
    for (n=0;n<6;n++) printf("%12.4e ",R(m,n));
    printf("\n");
  }
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         "\n",
         Elapsed_time,
         Xe(0,0), Xe(1,0), Xe(2,0), Xe(3,0),
         Xe(4,0), Xe(5,0), Xe(6,0),
         D_time, D_time2,
         Ax, Ay, Az,
         Wp, Wq, Wr,
         Mx, My, Mz);
}

void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n",
         Elapsed_time,
         Ax, Ay, Az,
         Wp, Wq, Wr,
         Mx, My, Mz);
}

void kalman_filter(void)
{
  // Kalman Filter
  float dt=0.01f;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}

// ===== PID =====
PID::PID()
{
  m_kp=1.0e-8f;
  m_ti=1.0e8f;
  m_td=0.0f;
  m_integral=0.0f;
  m_filter_time_constant=0.01f;
  m_filter_output=0.0f;
  m_err=0.0f;
  m_h=0.01f;
}

void PID::set_parameter(float kp, float ti, float td, float filter_time_constant, float h)
{
  m_kp=kp; m_ti=ti; m_td=td; m_filter_time_constant=filter_time_constant; m_h=h;
}

void PID::reset(void)
{
  m_integral=0.0f;
  m_filter_output=0.0f;
  m_err=0.0f;
  m_err2=0.0f;
  m_err3=0.0f;
}

void PID::i_reset(void){ m_integral=0.0f; }

void PID::printGain(void)
{
  printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",
         m_kp,m_ti,m_td,m_filter_time_constant,m_h);
}

float PID::filter(float x)
{
  m_filter_output = m_filter_output * m_filter_time_constant/(m_filter_time_constant + m_h)
                  + x * m_h/(m_filter_time_constant + m_h);
  return m_filter_output;
}

float PID::update(float err)
{
  m_integral += m_h * err;
  if(m_integral> 30000.0f) m_integral = 30000.0f;
  if(m_integral<-30000.0f) m_integral =-30000.0f;
  m_filter_output = filter((err-m_err3)/m_h);
  m_err3 = m_err2;
  m_err2 = m_err;
  m_err  = err;
  return m_kp*(err + m_integral/m_ti + m_td * m_filter_output);
}

// ===== Filter =====
Filter::Filter(){ m_state = 0.0f; m_T = 0.0025f; m_h = 0.0025f; }
void   Filter::reset(void){ m_state = 0.0f; }
void   Filter::set_parameter(float T, float h){ m_T = T; m_h = h; }
float  Filter::update(float u)
{
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}