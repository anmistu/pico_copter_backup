#include "pico_copter.hpp"
#include "hardware/clocks.h"
#include "modules/tof/tof_bridge.hpp"
#include "modules/rgbled/rgbled.hpp"
#include <stdio.h>

// グローバル変数
uint8_t Arm_flag=0;
semaphore_t sem;

// USB経由でJetsonからdepth情報を受信
void process_usb_command() {
    static char line[128];
    if (!fgets(line, sizeof(line), stdin)) return;

    // Expect "dx,dy,depth" (depth in meters or millimeters). Robust parse.
    int dx_i = 0, dy_i = 0;
    float depth_f = 0.0f;
    int n = sscanf(line, "%d,%d,%f", &dx_i, &dy_i, &depth_f);
    if (n < 3) return;

    cam_link_touch();

    // If depth seems millimeters (>= 10), convert to meters
    float depth_m = depth_f;
    if (depth_m > 10.0f) depth_m = depth_m * 0.001f;

    // Simple low-pass
    static float dx_f = 0.0f, dy_f = 0.0f, dm_f = 0.0f;
    const float alpha = 0.25f;
    dx_f = (1.0f - alpha)*dx_f + alpha*(float)dx_i;
    dy_f = (1.0f - alpha)*dy_f + alpha*(float)dy_i;
    dm_f = (1.0f - alpha)*dm_f + alpha*depth_m;

    // Publish to shared vars
    g_follow_dx         = dx_f;
    g_follow_dy         = dy_f;
    g_follow_depth_m    = dm_f;
    g_follow_last_us    = time_us_32();
    g_follow_data_valid = true;

    printf("[USB] depth=%.2f m dx=%.2f dy=%.2f\n", g_follow_depth_m,g_follow_dx,g_follow_dy);
}
int main(void)
{
  int start_wait=5;

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  stdio_init_all();                    // USB CDC有効化

  imu_mag_init();
  radio_init();
  variable_init();
  control_init();

  ESC_calib=0;    //初回のみ1にしてESCをキャリブレーション
  pwm_init();

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");

  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  
  
  // Core起動前後どちらでもOK（ISR中はNG）
  tof_setup();

  ina219_setup(i2c1, 0x40);

  // 3S の例（4Sなら 14.8f / warn 14.9f / crit 14.2f などに変更）
  ina219_set_vref_V(18.5f);               // 名目電圧（Vref）
  ina219_set_thresholds(18.0f, 17.2f);    // warn / crit しきい値
  ina219_set_iir_alpha(0.10f);            // IIR平滑（0.05〜0.20 推奨）

  Arm_flag=1;

  while(1)
  {
    process_usb_command();  // JetsonからUSB経由でdepth受信
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}