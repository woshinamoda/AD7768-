#include "uApp_other.h"

/* Private user variables code ----------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t led_cnt = 0;


/* USER CODE END 0 */




/* Private user function business code ----------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LED_Blinky()
{ //周期指示灯， 1sec中闪烁50ms
	led_cnt++;
	switch(led_cnt)
	{
		case 1 :
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);	
			break;
		case 50:
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);	
			break;		
		case 1000:
			led_cnt = 0;
			break;
	}
}




















/* USER CODE END 0 */

























































