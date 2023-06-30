void app_main(void)
{
    int i2c_master_port =  I2C_MASTER_NUM ;

    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_GPIO,
    .scl_io_num = I2C_SCL_GPIO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_FREQUENCY,    
};
    i2c_param_config(i2c_master_port, &conf);

    esp_err_t i2c_ret = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    if(i2c_ret != ESP_OK)
    {
        printf("Failed to install I2C driver!\n");
        return;
    }

    while (1)
    {
        int8_t status = _measurementReq();
        float temp = readTemperature(CELSIUS);
        if(temp != 0)
        {
            printf("Temperature %2f °C\n", temp);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else 
        {
            printf("Error reading temperature sensor! Status %d\n" , status);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        float humidity = readHumidity();
        if(temp != 0)
        {
            printf("Humidity %2f\n", humidity);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else 
        {
            printf("Error reading humidity sensor! Status %d\n" , status);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }        

    }
