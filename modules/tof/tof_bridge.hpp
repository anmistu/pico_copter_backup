#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void tof_setup();                    // 初期化（起動待ち含む）
void tof_poll();                     // 非ブロッキング更新（Core1で回す）
bool tof_read_valid(uint16_t* mm);   // 有効データなら true を返し、mm に距離[mm]
#ifdef __cplusplus
}
#endif
