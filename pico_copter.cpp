#include "pico_copter.hpp"
#include "hardware/clocks.h"
#include <stdio.h>

// グローバル変数
uint8_t Arm_flag=0;
semaphore_t sem;

// USB経由でJetsonからdepth情報を受信
// void process_usb_command() {
//     static char buffer[64];

//     if (fgets(buffer, sizeof(buffer), stdin)) {
//         int dx = 0, dy = 0;
//         float depth = 0;
//         if (sscanf(buffer, "%d,%d,%f", &dx, &dy, &depth) == 3) {
//             printf("[USB] depth=%.2f m dx=%d dy=%d\n", depth, dx, dy);
//             //printf("System clock frequency: %lu Hz\n", clock_get_hz(clk_sys));
//             // ★ depth を使った制御処理をここに追加可能
//         } else {
//             printf("Parse failed: %s\n", buffer);
//         }
//     }
// }

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

  ESC_calib=1;    //初回のみ1にしてESCをキャリブレーション
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

  Arm_flag=1;

  while(1)
  {
    //process_usb_command();  // JetsonからUSB経由でdepth受信
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}