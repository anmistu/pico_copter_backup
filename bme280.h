#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

int32_t BME280_compensate_T_int32(int32_t adc_T);
uint32_t BME280_compensate_P_int64(int32_t adc_P);
uint32_t bme280_compensate_H_int32(int32_t adc_H);
int _bme280_setByte(byte reg, byte data);
uint16_t _bme280_getReg(byte reg);
void _bme280_cal();
float bme280_getTemp();
float bme280_getHum();
float bme280_getPress();
int bme280_init();
int bme280_stop();
void bme280_print(float temp, float hum, float press);
int get_value(int argc,char **argv);


#ifdef __cplusplus
}
#endif