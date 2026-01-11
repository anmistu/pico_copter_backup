#include "pico_copter.hpp"
#include "hardware/clocks.h"
#include "modules/tof/tof_bridge.hpp"
#include "modules/rgbled/rgbled.hpp"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "tusb.h"              // USB接続/タスク管理（TinyUSB）

// ===== 既存のグローバル =====
uint8_t Arm_flag = 0;
semaphore_t sem;

// control.cpp 側で定義されるログフラグ
extern volatile uint8_t Logoutputflag;

// follow系の共有変数（ヘッダでexternされている想定）
extern volatile float g_follow_dx;
extern volatile float g_follow_dy;
extern volatile float g_follow_depth_m;
extern volatile uint32_t g_follow_last_us;
extern volatile bool g_follow_data_valid;
extern volatile float g_follow_rx_hz;


// ===== 追加ヘルパ =====
// USB CDC 受信バッファを捨てる（ブロック回避）
static inline void usb_flush_input(void) {
    for (;;) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) break;
    }
}

// USB CDC がホスト接続済みか
static inline bool usb_connected(void) {
    return tud_cdc_connected();
}

// 非ブロッキングで1行を組み立てる（\n 終端で1行完成）
static bool usb_readline_nonblock(char* out, size_t cap) {
    static char buf[128];
    static size_t len = 0;

    int ch = getchar_timeout_us(0);
    bool got = false;

    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\r') {
            // ignore
        } else if (ch == '\n') {
            buf[len] = '\0';
            if (out && cap) {
                strncpy(out, buf, cap - 1);
                out[cap - 1] = '\0';
            }
            len = 0;
            got = true;
            break;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = (char)ch;
        }
        ch = getchar_timeout_us(0);
    }
    return got;
}

// Jetsonから "dx,dy,depth\n" を受信して共有変数を更新（非ブロッキング）
static inline void process_usb_command() {
    // ログ出力中は depth 受信を完全無効化（ブロックを絶対に作らない）
    if (Logoutputflag == 1) return;

    char line[128];
    if (!usb_readline_nonblock(line, sizeof(line))) return;  // 行が到着していない→何もしない

    int   dx_i = 0, dy_i = 0;
    float depth_f = 0.0f;
    int n = sscanf(line, "%d,%d,%f", &dx_i, &dy_i, &depth_f);
    if (n < 3) return;  // 形式不正

    cam_link_touch();

    // === 受信レート(Hz)推定 ===
    static uint32_t prev_us = 0;
    uint32_t now_us = time_us_32();

    if (prev_us != 0) {
        uint32_t dt_us = now_us - prev_us;  // wrapしてもunsigned差分でOK
        if (dt_us > 0) {
            float hz = 1e6f / (float)dt_us;

            // 異常値対策（必要なら）
            if (hz > 500.0f) hz = 500.0f;

            // 簡易LPF（なめらかにする）
            const float beta = 0.25f;
            g_follow_rx_hz = (1.0f - beta) * g_follow_rx_hz + beta * hz;
        }
    }
    prev_us = now_us;

    // depth(mm)→m の自動判定
    float depth_m = depth_f;
    if (depth_m > 10.0f) depth_m *= 0.001f;

    // 簡易LPF
    static float dx_f = 0.0f, dy_f = 0.0f, dm_f = 0.0f;
    const float alpha = 0.25f;
    dx_f = (1.0f - alpha) * dx_f + alpha * (float)dx_i;
    dy_f = (1.0f - alpha) * dy_f + alpha * (float)dy_i;
    dm_f = (1.0f - alpha) * dm_f + alpha * depth_m;

    // 共有変数へ
    g_follow_dx         = dx_f;
    g_follow_dy         = dy_f;
    g_follow_depth_m    = dm_f;
    g_follow_last_us    = now_us;
    g_follow_data_valid = true;
}

int main(void)
{
    int start_wait = 5;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    stdio_init_all();                      // USB CDC 初期化
    setvbuf(stdout, NULL, _IONBF, 0);      // 標準出力を無バッファ化（詰まりにくく）

    imu_mag_init();
    radio_init();
    variable_init();
    control_init();

    ESC_calib = 0;                         // 必要時のみ1に
    pwm_init();

    while (start_wait) {
        start_wait--;
        printf("#Please wait %d[s]\r", start_wait);
        sleep_ms(1000);
    }
    printf("\n");

    sem_init(&sem, 0, 1);
    multicore_launch_core1(angle_control);

    tof_setup();

    // 電圧モニタ（必要なければ無視）
    ina219_setup(i2c1, 0x40);
    ina219_set_vref_V(18.5f);
    ina219_set_thresholds(18.0f, 17.2f);
    ina219_set_iir_alpha(0.10f);

    Arm_flag = 1;

    // ログ状態の立ち上がり/立ち下がり検出
    bool was_logging = false;

    while (1)
    {
        // ===== ログ中の特殊処理 =====
        if (Logoutputflag == 1) {
            if (!was_logging) {
                // 立ち上がり：入力捨て＋USB接続を少しだけ待つ
                usb_flush_input();
                was_logging = true;

                absolute_time_t deadline = make_timeout_time_ms(3000);
                while (Logoutputflag == 1 && !usb_connected() && !time_reached(deadline)) {
                    tud_task();
                    sleep_ms(10);
                }
                if (Logoutputflag == 1 && !usb_connected()) {
                    // 接続が無いままならログを諦めて通常へ（固まり防止）
                    Logoutputflag = 0;
                    was_logging = false;
                }
            }
        } else {
            // 立ち下がり：ログ直後のゴミを捨てる
            if (was_logging) {
                usb_flush_input();
                was_logging = false;
            }
            // 平常時のみ Jetson→Pico の depth を受信（非ブロッキング）
            process_usb_command();
        }

        tight_loop_contents();

        // ===== ログ出力ループ =====
        while (Logoutputflag == 1) {
            log_output();     // 1行ずつCSV出力（既存実装）
            tud_task();       // TinyUSBタスクを回す（詰まり防止）
            sleep_ms(1);      // 他処理へCPUを譲る
        }
    }

    return 0;
}
