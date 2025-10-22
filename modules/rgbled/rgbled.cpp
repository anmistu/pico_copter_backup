#include "rgbled.hpp"

WS2812 ledStrip(
    RGBLED_PIN,         // Data line is connected to pin 0. (GP0)
    RGBLED_LENGTH,      // Strip is 6 LEDs long.
    pio0,               // Use PIO 0 for creating the state machine.
    0,                  // Index of the state machine that will be created for controlling the LED strip
                        // You can have 4 state machines per PIO-Block up to 8 overall.
                        // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    WS2812::FORMAT_GRB  // Pixel format used by the LED strip
);


void rgbled_normal(void)
{
  // 黄色
  ledStrip.setPixelColor(0, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(1, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(2, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(3, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(4, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(5, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(6, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(7, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(8, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(9, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(10, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(11, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(12, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(13, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(14, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(15, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(16, WS2812::RGB(0,0,0));
  // 赤
  ledStrip.setPixelColor(17, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(18, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(19, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(20, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(21, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(22, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(23, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(24, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(25, WS2812::RGB(0,0,0));
  // 青
  ledStrip.setPixelColor(26, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(27, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(28, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(29, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(30, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(31, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(32, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(33, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(34, WS2812::RGB(0,0,225));
  // 青
  ledStrip.setPixelColor(35, WS2812::RGB(0,0,0));
  ledStrip.show();
}

void rgbled_green(void)
{
  
  ledStrip.setPixelColor(0, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(1, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(2, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(3, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(4, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(5, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(6, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(7, WS2812::RGB(0,225,0));

  ledStrip.setPixelColor(8, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(9, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(10, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(11, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(12, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(13, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(14, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(15, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(16, WS2812::RGB(0,225,0));
  
  ledStrip.setPixelColor(17, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(18, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(19, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(20, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(21, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(22, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(23, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(24, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(25, WS2812::RGB(0,225,0));
  
  ledStrip.setPixelColor(26, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(27, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(28, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(29, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(30, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(31, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(32, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(33, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(34, WS2812::RGB(0,225,0));
  // 青
  ledStrip.setPixelColor(35, WS2812::RGB(0,225,0));
  ledStrip.show();
}


void rgbled_redcircle(void)
{
   // 黄色
  ledStrip.setPixelColor(0, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(1, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(2, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(3, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(4, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(5, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(6, WS2812::RGB(225,135,0));
  //見やすさを加味して、色の変わる部分の色は付けない 
  ledStrip.setPixelColor(7, WS2812::RGB(0,0,0));
  // 緑
  ledStrip.setPixelColor(8, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(9, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(10, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(11, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(12, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(13, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(14, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(15, WS2812::RGB(0,225,0));
  ledStrip.setPixelColor(16, WS2812::RGB(0,0,0));
  // 赤
  ledStrip.setPixelColor(17, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(18, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(19, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(20, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(21, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(22, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(23, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(24, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(25, WS2812::RGB(0,0,0));
  // 青
  ledStrip.setPixelColor(26, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(27, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(28, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(29, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(30, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(31, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(32, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(33, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(34, WS2812::RGB(0,0,225));
  // 青
  ledStrip.setPixelColor(35, WS2812::RGB(0,0,0));
  ledStrip.show();
}

void rgbled_red(void)
{
  ledStrip.setPixelColor(0, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(1, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(2, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(3, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(4, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(5, WS2812::RGB(225,0,0));

  ledStrip.setPixelColor(6, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(7, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(8, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(9, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(10, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(11, WS2812::RGB(225,0,0));

  ledStrip.setPixelColor(12, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(13, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(14, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(15, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(16, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(17, WS2812::RGB(225,0,0));

  ledStrip.setPixelColor(18, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(19, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(20, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(21, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(22, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(23, WS2812::RGB(225,0,0));

  ledStrip.setPixelColor(24, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(25, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(26, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(27, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(28, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(29, WS2812::RGB(225,0,0));

  ledStrip.setPixelColor(30, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(31, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(32, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(33, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(34, WS2812::RGB(225,0,0));
  ledStrip.setPixelColor(35, WS2812::RGB(225,0,0));
  ledStrip.show();
}
void rgbled_blue(void)
{
  ledStrip.setPixelColor(0, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(1, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(2, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(3, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(4, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(5, WS2812::RGB(0,0,225));

  ledStrip.setPixelColor(6, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(7, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(8, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(9, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(10, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(11, WS2812::RGB(0,0,225));

  ledStrip.setPixelColor(12, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(13, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(14, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(15, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(16, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(17, WS2812::RGB(0,0,225));

  ledStrip.setPixelColor(18, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(19, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(20, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(21, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(22, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(23, WS2812::RGB(0,0,225));

  ledStrip.setPixelColor(24, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(25, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(26, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(27, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(28, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(29, WS2812::RGB(0,0,225));

  ledStrip.setPixelColor(30, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(31, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(32, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(33, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(34, WS2812::RGB(0,0,225));
  ledStrip.setPixelColor(35, WS2812::RGB(0,0,225));
  ledStrip.show();
}

void rgbled_pink(void)
{
  ledStrip.setPixelColor(0, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(1, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(2, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(3, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(4, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(5, WS2812::RGB(225,80,130));

  ledStrip.setPixelColor(6, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(7, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(8, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(9, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(10, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(11, WS2812::RGB(225,80,130));

  ledStrip.setPixelColor(12, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(13, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(14, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(15, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(16, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(17, WS2812::RGB(225,80,130));

  ledStrip.setPixelColor(18, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(19, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(20, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(21, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(22, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(23, WS2812::RGB(225,80,130));

  ledStrip.setPixelColor(24, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(25, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(26, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(27, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(28, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(29, WS2812::RGB(225,80,130));

  ledStrip.setPixelColor(30, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(31, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(32, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(33, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(34, WS2812::RGB(225,80,130));
  ledStrip.setPixelColor(35, WS2812::RGB(225,80,130));
  ledStrip.show();
}


void rgbled_yellow(void)
{
  ledStrip.setPixelColor(0, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(1, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(2, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(3, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(4, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(5, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(6, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(7, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(8, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(9, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(10, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(11, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(12, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(13, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(14, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(15, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(16, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(17, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(18, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(19, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(20, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(21, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(22, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(23, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(24, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(25, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(26, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(27, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(28, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(29, WS2812::RGB(225,135,0));

  ledStrip.setPixelColor(30, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(31, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(32, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(33, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(34, WS2812::RGB(225,135,0));
  ledStrip.setPixelColor(35, WS2812::RGB(225,135,0));
  ledStrip.show();
}

void rgbled_off(void)
{
  // 0,0,0
  ledStrip.setPixelColor(0, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(1, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(2, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(3, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(4, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(5, WS2812::RGB(0,0,0));

  ledStrip.setPixelColor(6, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(7, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(8, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(9, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(10, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(11, WS2812::RGB(0,0,0));

  ledStrip.setPixelColor(12, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(13, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(14, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(15, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(16, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(17, WS2812::RGB(0,0,0));

  ledStrip.setPixelColor(18, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(19, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(20, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(21, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(22, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(23, WS2812::RGB(0,0,0));

  ledStrip.setPixelColor(24, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(25, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(26, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(27, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(28, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(29, WS2812::RGB(0,0,0));

  ledStrip.setPixelColor(30, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(31, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(32, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(33, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(34, WS2812::RGB(0,0,0));
  ledStrip.setPixelColor(35, WS2812::RGB(0,0,0));
  ledStrip.show();
}

void rgbled_switch(uint8_t flag)
{
  if (flag ==1)rgbled_green();
  else rgbled_off();
}

void rgbled_wait(void)
{
  static uint8_t index=1;
  static uint16_t counter = 0;
  if(counter==0)
  {
    ledStrip.setPixelColor(index, WS2812::RGB(0,0,0));
    ledStrip.setPixelColor(index + 35, WS2812::RGB(0,0,0));
    if(index<36)index++;
    else index=0;
    ledStrip.setPixelColor(index, WS2812::RGB(0,0,50));
    ledStrip.setPixelColor(index + 35, WS2812::RGB(0,0,50));
    ledStrip.show();
  }
  counter ++;
  if(counter==30)counter =0;
}

// void rgbled_rocking_new(void)
// {
//   static uint8_t index=1;
//   static uint16_t counter = 0;
//   if(counter==0)
//   {
//     ledStrip.setPixelColor(index, WS2812::RGB(0,0,0));
//     ledStrip.setPixelColor(index + 35, WS2812::RGB(0,0,0));
//     if(index<36)index++;
//     else index=0;
//     ledStrip.setPixelColor(index, WS2812::RGB(0,0,50));
//     ledStrip.setPixelColor(index + 35, WS2812::RGB(0,0,50));
//     ledStrip.show();
//   }
//   counter ++;
//   if(counter==30)counter =0;
// }

uint16_t uni_count=0;

void unique_led(void){
  
  if (uni_count<420){
    rgbled_red();
    uni_count++;

  }
  else if(uni_count<1030){
    rgbled_yellow();
    uni_count++;
  }
  else{
    rgbled_blue();
    uni_count++;
    
  }
}
