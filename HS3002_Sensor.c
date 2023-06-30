#include "HS3002_Sensor.h"

float _humidity = 0;
float _temperature = 0;


int8_t _measurementReq()
{
  int8_t _status = 0;
  uint8_t _iteration = 0;

  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd , (HS300X_ADR << 1) | I2C_MASTER_WRITE , true);
  // i2c_master_write_byte(cmd , read_buffer , true);
  // i2c_master_stop(cmd);
  // esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM , cmd , HS300X_DELAY_MEASUREMENT / portTICK_PERIOD_MS);
  // i2c_cmd_link_delete(cmd);

  // if(ret != ESP_OK)
  // {
  //   printf("I2C_send_req fail!\n"); 
  //   return HS300X_ERROR_COLLISION_I2C;
  // }

  uint8_t cmd = HS300X_ADR << 1;
  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM , HS300X_ADR , &cmd , sizeof(cmd) , pdMS_TO_TICKS(HS300X_DELAY_MEASUREMENT));

  if(ret != ESP_OK)
  {
    printf("I2C_send_cmd fail!\n"); 
    printf("ret %d\n" , ret);
    return HS300X_ERROR_COLLISION_I2C;
  }


  vTaskDelay(HS300X_DELAY_MEASUREMENT / portTICK_PERIOD_MS);

  do
  {
    _status = _readSensor();
    _iteration++;
    if (_iteration > HS300X_MAX_ITERATION)
      return HS300X_ERROR_SENSOR_BUSY;
    vTaskDelay(HS300X_DELAY_ITERATION / portTICK_PERIOD_MS);
  } while (!_status);

  return _status;
}

uint8_t _readSensor()
{
  uint16_t _rawTempMSB;
  uint16_t _rawTemp;
  uint16_t _rawHumMSB;
  uint16_t _rawHum;
  uint8_t _rawStatus;

  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd , (HS300X_ADR << 1) | I2C_MASTER_READ , true);
  // i2c_master_read_byte(cmd , &_rawHumMSB , I2C_MASTER_ACK);
  // i2c_master_read_byte(cmd , &_rawHum , I2C_MASTER_ACK);
  // i2c_master_read_byte(cmd , &_rawTempMSB , I2C_MASTER_ACK);
  // i2c_master_read_byte(cmd , &_rawTemp , I2C_MASTER_ACK);
  // i2c_master_stop(cmd);

  // esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM , cmd , HS300X_DELAY_MEASUREMENT / portTICK_PERIOD_MS);
  // i2c_cmd_link_delete(cmd);

  uint8_t read_buffer[4];
  uint8_t cmd = HS300X_ADR << 1 | 0x01;
  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, HS300X_ADR , &cmd, sizeof(cmd) , pdMS_TO_TICKS(HS300X_DELAY_MEASUREMENT));
  esp_err_t ret1 = i2c_master_read_from_device(I2C_MASTER_NUM , HS300X_ADR , read_buffer , sizeof(read_buffer) ,  pdMS_TO_TICKS(HS300X_DELAY_MEASUREMENT));

  
  if(ret != ESP_OK)
  {
    printf("I2C_write_req fail!\n");
    printf("ret %d\n" , ret);
    return -1;
  }

    if(ret1 != ESP_OK)
  {
    printf("I2C_read_req fail!\n");
    printf("ret1 %d\n" , ret);
    return -1;
  }

  _rawHumMSB = read_buffer[0] << 8;
  _rawHum = read_buffer[1];
  _rawTempMSB = read_buffer[2] << 8;
  _rawTemp = read_buffer[3];

  
  _rawHumMSB = read_buffer[0] << 8;
  _rawHum = read_buffer[1];
  _rawTempMSB = read_buffer[2] << 8;
  _rawTemp = read_buffer[3];

  _rawHum |= _rawHumMSB;
  _rawTemp |= _rawTempMSB;

  _rawStatus = _rawTemp >> 14;
  _rawHum = _rawHum & 0x3FFF; // mask 2 bit first
  _rawTemp = _rawTemp >> 2;   // mask 2 bit last

  if (_rawHum == 0x3FFF) return 0;
  if (_rawTemp == 0x3FFF) return 0;
  
 _temperature = (_rawTemp * HS300X_TEMP_MULTY) - HS300X_TEMP_MIN;
 _humidity = _rawHum * HS300X_HUMD_MULTY;
  return _rawStatus + 1;

}

float readTemperature(int units)
{
  if (_measurementReq() <= 0)
  {
    //return 0;
  }

  if (units == FAHRENHEIT) { // Fahrenheit = (Celsius * 9 / 5) + 32
    return (_temperature * 9.0 / 5.0) + 32.0;
  } else {
    return _temperature;
  }
}

float readHumidity()
{
  if (_measurementReq() <= 0) {
    return 0;
  }

  return _humidity;
}

