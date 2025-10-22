#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstddef>
#include <cstdint>

// SPIでWS2812/SK6812系(1線)を駆動する簡易ドライバ
// MOSI=DINへ直結（330Ω直列推奨）、SCKは配線不要（内部タイミング生成のみ）

void spi_ws2812_init(spi_inst_t* inst, uint mosi_gpio, uint sck_gpio,
                     uint32_t bitrate_hz = 2400000);

void spi_ws2812_show_grb(const uint8_t* grb, size_t count); // GRB配列をそのまま送信
void spi_ws2812_fill_rgb(uint8_t r, uint8_t g, uint8_t b, size_t count); // 全LED一括
