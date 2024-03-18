
#include <stdio.h>
#include "driver/gpio.h"
#define BLINK_GPIO1 17
#define BLINK_GPIO2 18
static uint8_t s_led_state = 0;
static uint8_t led_state = 1;
void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO1);//重置gpio1
    gpio_reset_pin(BLINK_GPIO2);//重置gpio2

    
    gpio_set_direction(BLINK_GPIO1, GPIO_MODE_OUTPUT);//设置引脚为推挽输出模式
    gpio_set_direction(BLINK_GPIO2, GPIO_MODE_OUTPUT);//设置引脚为推挽输出模式

   gpio_set_level(BLINK_GPIO1, led_state);//设置gpio1输出为高电平
   gpio_set_level(BLINK_GPIO2, s_led_state);//设置gpio2输出为低电平
}
