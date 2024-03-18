#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           20        // 定义 I2C 总线的 SCL 引脚
#define I2C_MASTER_SDA_IO           21         // 定义 I2C 总线的 SDA 引脚
#define I2C_MASTER_NUM              I2C_NUM_0  // 使用 I2C 0 控制器
#define I2C_MASTER_TX_BUF_DISABLE   0          // 不使用 TX 缓冲区
#define I2C_MASTER_RX_BUF_DISABLE   0          // 不使用 RX 缓冲区
#define I2C_MASTER_FREQ_HZ          100000     // I2C 主设备时钟频率为 100kHz

#define MPU6050_ADDR                0x68       // MPU6050 I2C 地址

static const char *TAG = "MPU6050";

void i2c_master_init() {
    //初始化i2c总线
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void MPU6050_init() {
    // 初始化 MPU6050
    uint8_t data[] = {0x6B, 0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(100 / portTICK_PERIOD_MS);  
}
 
void MPU6050_read_data() {
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    uint8_t data[14];

    // 读取加速度计和陀螺仪数据
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // 设置起始寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);

    // 解析数据
    accX = (data[0] << 8) | data[1];
    accY = (data[2] << 8) | data[3];
    accZ = (data[4] << 8) | data[5];
    gyroX = (data[8] << 8) | data[9];
    gyroY = (data[10] << 8) | data[11];
    gyroZ = (data[12] << 8) | data[13];

    // 打印数据
    ESP_LOGI(TAG, "Acc: %d, %d, %d | Gyro: %d, %d, %d", accX, accY, accZ, gyroX, gyroY, gyroZ);
}

void app_main() {
    i2c_master_init(); // 初始化 I2C 总线
    MPU6050_init();

    while (1) {
        MPU6050_read_data(); // 读取 MPU6050 数据
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延时 1 秒
    }
}
