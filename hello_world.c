
#include "driver/uart.h"
#include "string.h"
#include "freertos/FreeRTOS.h"


void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,                    //设置波特率    115200
        .data_bits = UART_DATA_8_BITS,          //设置数据位    8位
        .parity = UART_PARITY_DISABLE,          //设置奇偶校验  不校验
        .stop_bits = UART_STOP_BITS_1,          //设置停止位    1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //设置硬件流控制 不使能
        .source_clk = UART_SCLK_APB,            //设置时钟源
    };
    //安装串口驱动 
    uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
    //串口参数配置 
    uart_param_config(UART_NUM_0, &uart_config);
   
}

void app_main(void)
{
    init();    
    while(1)
    {
        uart_write_bytes(UART_NUM_0, "Hello, world!", 13); 
     // printf("Hello, world!\n");
    vTaskDelay(1000 / 100);
    } 
   
}

