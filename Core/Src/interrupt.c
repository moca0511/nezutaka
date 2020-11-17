/*
 * interrupt.c
 *
 *  Created on: 2020/07/28
 *      Author: junmo
 */
#include"main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim2;
extern int32_t us;
extern uint32_t MotorStepCount_R;
extern uint32_t MotorStepCount_L;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {//wait_usのカウント値を0までデクリメントしたらセマフォ解放
		if (us <= 1) {
			HAL_TIM_Base_Stop_IT(&htim2);
			osSemaphoreRelease(wait_usSemHandle);
		} else {
			us--;
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/*　外部割込み*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MO_R_Pin) {
		MotorStepCount_R++;
	}
	if (GPIO_Pin == MO_L_Pin) {
		MotorStepCount_L++;
	}
	if (GPIO_Pin == BOARD_SW_Pin) {
		printf("Exit\n");
	}
}
