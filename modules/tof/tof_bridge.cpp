#include "tof_vl53l1x.hpp"
static ToF_VL53L1X g_tof(0x29);
static bool g_inited = false;

extern "C" void tof_setup() {
    if (g_inited) return;
    if (!g_tof.init()) return;
    g_tof.configure(/*distance_mode=*/2, /*budget_ms=*/50, /*inter_ms=*/60);
    g_tof.start();
    g_inited = true;
}

extern "C" void tof_poll() {
    if (!g_inited) return;
    g_tof.poll(); // data ready時だけ内部更新
}

extern "C" bool tof_read_valid(uint16_t* mm) {
    if (!g_inited) return false;
    if (!g_tof.valid()) return false;
    if (mm) *mm = g_tof.filtered_mm();
    return true;
}
