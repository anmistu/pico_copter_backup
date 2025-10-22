#ifndef RGBLED_HPP
#define RGBLED_HPP

#include "WS2812.hpp"

#define RGBLED_PIN 16
#define RGBLED_LENGTH 36

void rgbled_off(void);
void rgbled_normal(void);
void rgbled_green(void);
void rgbled_red(void);
void rgbled_blue(void);
void rgbled_pink_rightOrange(void);
void rgbled_orange(void);
void rgbled_switch(uint8_t);
void rgbled_wait(void);
void rgbled_rocking(void);
void rgbled_lightblue(void);
void rgbled_pink(void);
void rgbled_redcircle(void);
void rgbled_Hovering(void);
// void rgbled_rocking_new(void);
void unique_led(void);


#endif
