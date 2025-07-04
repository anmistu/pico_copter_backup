/*
 * pwm.phh 
 * Header file for PWM function
*/
#ifndef PWM_HPP
#define PWM_HPP
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico_copter.hpp"
#include "control.hpp"

//#define DUTYMIN 1250
//#define DUTYMAX 2500
#define DUTYMIN 1650    //pico2の設定
#define DUTYMAX 2850    //pico2の設定

//グローバル変数
extern uint8_t ESC_calib;

//関数の宣言
void pwm_init();
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

#endif
