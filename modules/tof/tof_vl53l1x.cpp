#include "tof_vl53l1x.hpp"
#include <algorithm>
#include <stdio.h>

ToF_VL53L1X::ToF_VL53L1X(uint16_t dev_addr): dev_(dev_addr) {}

bool ToF_VL53L1X::init(){
    // I2C初期化（プラットフォームヘッダの定義に従う）
    i2c_init(I2C_PORT, i2C_CLOCK);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    sleep_ms(3000); // センサ起動待ち

    uint8_t state = 0; int8_t st = 0;
    for (int i=0;i<40 && (state&1)==0;i++){ st = VL53L1X_BootState(dev_, &state); sleep_ms(10); }
    if ((state&1)==0){ printf("[ToF] boot timeout\n"); return false; }
    st = VL53L1X_SensorInit(dev_);
    if (st != 0){ printf("[ToF] SensorInit fail: %d\n", st); return false; }
    return true;
}

bool ToF_VL53L1X::configure(uint16_t mode, uint16_t budget_ms, uint16_t inter_ms){
    int8_t st = 0;
    st |= VL53L1X_SetDistanceMode(dev_, mode);
    st |= VL53L1X_SetTimingBudgetInMs(dev_, budget_ms);
    st |= VL53L1X_SetInterMeasurementInMs(dev_, inter_ms);
    return st==0;
}
bool ToF_VL53L1X::start(){ return VL53L1X_StartRanging(dev_)==0; }
bool ToF_VL53L1X::stop (){ return VL53L1X_StopRanging (dev_)==0; }

bool ToF_VL53L1X::poll(){
    int8_t st = VL53L1X_CheckForDataReady(dev_, &ready_);
    if (st!=0 || ready_==0) return false;
    ready_ = 0;
    st |= VL53L1X_GetRangeStatus(dev_, &range_status_);
    st |= VL53L1X_GetDistance(dev_, &raw_mm_);
    st |= VL53L1X_ClearInterrupt(dev_);
    valid_ = (st==0) && (range_status_==0);
    if (valid_){
        if (filled_ < K) ring_[filled_++] = raw_mm_; else ring_[idx_] = raw_mm_;
        idx_ = (idx_+1)%K;
        uint16_t tmp[K]; for(int i=0;i<filled_;++i) tmp[i]=ring_[i];
        std::sort(tmp, tmp+filled_);
        uint16_t med = tmp[filled_/2];
        ema_ = (ema_<0.f) ? med : (alpha_*med + (1.f-alpha_)*ema_);
        filtered_mm_ = (uint16_t)(ema_ + 0.5f);
    }
    return true;
}
