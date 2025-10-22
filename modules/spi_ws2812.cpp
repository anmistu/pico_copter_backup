#include "spi_ws2812.hpp"

static spi_inst_t* g_spi = nullptr;
static uint g_mosi = 19;
static uint g_sck  = 18;

static uint32_t g_enc24[256];

static inline uint32_t encode_byte_24(uint8_t x) {
    // '0'->100, '1'->110 をMSBから詰めて 8bit→24bit（SPI 2.4MHz 前提）
    uint32_t out = 0;
    for (int i = 7; i >= 0; --i) {
        bool bit = (x >> i) & 1;
        out <<= 3;
        out |= bit ? 0b110 : 0b100;
    }
    return out;
}

void spi_ws2812_init(spi_inst_t* inst, uint mosi_gpio, uint sck_gpio, uint32_t bitrate_hz) {
    g_spi  = inst;
    g_mosi = mosi_gpio;
    g_sck  = sck_gpio;

    for (int v = 0; v < 256; ++v) g_enc24[v] = encode_byte_24((uint8_t)v);

    spi_init(g_spi, bitrate_hz); // 2.4MHz
    spi_set_format(g_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(g_mosi, GPIO_FUNC_SPI);
    gpio_set_function(g_sck,  GPIO_FUNC_SPI);
}

static inline void write_enc24(uint8_t v) {
    uint32_t e = g_enc24[v];
    uint8_t buf[3] = {
        (uint8_t)((e >> 16) & 0xFF),
        (uint8_t)((e >>  8) & 0xFF),
        (uint8_t)( e        & 0xFF)
    };
    spi_write_blocking(g_spi, buf, 3);
}

void spi_ws2812_show_grb(const uint8_t* grb, size_t count) {
    if (!g_spi || !grb || count == 0) return;
    for (size_t i = 0; i < count; ++i) {
        uint8_t G = grb[i*3 + 0];
        uint8_t R = grb[i*3 + 1];
        uint8_t B = grb[i*3 + 2];
        write_enc24(G); write_enc24(R); write_enc24(B);
    }
    uint8_t zeros[16] = {0};               // >50us リセット
    spi_write_blocking(g_spi, zeros, sizeof(zeros));
}

void spi_ws2812_fill_rgb(uint8_t r, uint8_t g, uint8_t b, size_t count) {
    if (!g_spi || count == 0) return;
    for (size_t i = 0; i < count; ++i) {
        write_enc24(g); write_enc24(r); write_enc24(b); // GRB順
    }
    uint8_t zeros[16] = {0};
    spi_write_blocking(g_spi, zeros, sizeof(zeros));
}
