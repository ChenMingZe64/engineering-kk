#include "led.h"
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"

static uint8_t idx;
static LEDInstance *bsp_led_ins[LED_MAX_NUM] = {NULL};

LEDInstance *LEDRegister(LED_Init_Config_s *led_config)
{
    LEDInstance *led_ins = (LEDInstance *)zero_malloc(sizeof(LEDInstance));
    // 剩下的值暂时都被置零
    led_ins->led_pwm = PWMRegister(&led_config->pwm_config);
    led_ins->led_switch = led_config->init_swtich;

    bsp_led_ins[idx++] = led_ins;
    return led_ins;
}

void LEDSet(LEDInstance *_led, uint8_t alpha, uint8_t color_value, uint8_t brightness)
{
}

void LEDSwitch(LEDInstance *_led, uint8_t led_switch)
{
    if (led_switch == 1)
    {
        _led->led_switch = 1;
    }
    else
    {
        _led->led_switch = 0;
        // PWMSetPeriod(_led,0);
    }
}

void LEDShow()
{
}
