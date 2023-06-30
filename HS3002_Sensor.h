#include "stdio.h"
#include "driver/i2c.h"
#include "freertos/freeRTOS.h"
#include "freertos/task.h"

#define HS300X_ADR                  0x44
#define HS300X_TEMP_MULTY           0.010071415
#define HS300X_TEMP_MIN             40
#define HS300X_HUMD_MULTY           0.006163516
#define HS300X_MAX_ITERATION        100
#define HS300X_DELAY_MEASUREMENT    35000          // typical on datasheet 16.90 ms, rounded up a little (35ms = 1 try)
#define HS300X_DELAY_ITERATION      1000

#define HS300X_STALE_DATA           2
#define HS300X_OK                   1
#define HS300X_ERROR_SENSOR_BUSY    0
#define HS300X_ERROR_COLLISION_I2C  -1

#define I2C_SCL_GPIO 6
#define I2C_SDA_GPIO 7
#define I2C_FREQUENCY 100000
#define I2C_MASTER_NUM 0

typedef enum {

  FAHRENHEIT,
  CELSIUS,

}degree;

float readTemperature(int units);
float readHumidity();

extern  uint8_t _status;

uint8_t _readSensor();
int8_t _measurementReq();

void i2c_master_init();
