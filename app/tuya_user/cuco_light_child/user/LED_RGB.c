#include "LED_RGB.h"
#include "espressif/c_types.h"
#include "esp8266/pin_mux_register.h"
#include "gpio.h"
#include "device.h"

void ledRgb_init(void)
{
	
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);

	GPIO_AS_OUTPUT(GPIO_Pin_0);
	GPIO_AS_OUTPUT(GPIO_Pin_4);
	GPIO_AS_OUTPUT(GPIO_Pin_5);
	GPIO_AS_OUTPUT(GPIO_Pin_15);

	GPIO_OUTPUT(GPIO_Pin_0,1);
	GPIO_OUTPUT(GPIO_Pin_4,1);
	GPIO_OUTPUT(GPIO_Pin_5,1);
	GPIO_OUTPUT(GPIO_Pin_15,1);
}

