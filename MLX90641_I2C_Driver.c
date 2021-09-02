/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Adjusted for use with ESP-IDF I2C driver by Sivert Havso */

#include "MLX90641_I2C_Driver.h"
#include "driver/i2c.h"
#include "esp_log.h"

i2c_config_t conf;
int init = 0;

const int I2C_MASTER_TX_BUF_DISABLE = 0;
const int I2C_MASTER_RX_BUF_DISABLE = 0;
const int WRITE_BIT = I2C_MASTER_WRITE;
const int READ_BIT = I2C_MASTER_READ;


const int I2C_MASTER_SDA_IO = 21;
const int I2C_MASTER_SCL_IO = 22;

#define ACK_CHECK_EN 0x1

static esp_err_t esp_i2c_init()
{
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 1000000; // 0.4 MHz is noted as the typical clock speed in the MLX90641 datasheet
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void MLX90641_I2CInit()
{
    ESP_ERROR_CHECK(esp_i2c_init());
}

int MLX90641_I2CGeneralReset(void)
{
    int ack = 0;
    uint8_t data_to_write[2] = {0,0};

    data_to_write[0] = 0x00;
    data_to_write[1] = 0x06;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, data_to_write[0] | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, &data_to_write[1], 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return -1;
    }

    return 0;
}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{

    uint8_t sa;
    //int ack = 0;
    int cnt = 0;
    int i = 0;
    uint8_t data_to_write[2] = {0,0};
    uint8_t i2cData[1664] = {0};
    uint16_t *p;

    p = data;
    sa = (slaveAddr << 1);
    data_to_write[0] = startAddress >> 8;
    data_to_write[1] = startAddress & 0x00FF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_to_write, 2, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | 0x01, ACK_CHECK_EN);
    i2c_master_read(cmd, i2cData, 2*nMemAddressRead, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return -1;
    }

    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }
    return 0;
}

void MLX90641_I2CFreqSet(int freq)
{
    i2c_driver_delete(I2C_NUM_0);
    conf.master.clk_speed = 1000 * freq;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t sa;
    int ack = 0;
    uint8_t data_to_write[4] = {0,0,0,0};
    static uint16_t dataCheck;

    sa = (slaveAddr << 1);
    data_to_write[0] = writeAddress >> 8;
    data_to_write[1] = writeAddress & 0x00FF;
    data_to_write[2] = data >> 8;
    data_to_write[3] = data & 0x00FF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sa | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_to_write, 4, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return -1;
    }

    MLX90641_I2CRead(slaveAddr,writeAddress,1, &dataCheck);

    if ( dataCheck != data)
    {
        return -2;
    }

    return 0;
}
