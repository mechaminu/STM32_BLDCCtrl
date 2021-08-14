#include "main.h"

extern TIM_HandleTypeDef htim2;
extern uint8_t dir_flag;
extern uint8_t curStep;

extern uint8_t cw_table[];
extern uint8_t ccw_table[];

uint8_t go = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_TIM_Base_Stop_IT(htim);

  // 100ms 이후에도 버튼이 계속 눌려있어야 버튼 입력을 인식
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET || go)
  {
    if (!go) {
      driveGate(0); // Prevent shoot-through via all off and 100ms delay
      go = 1;
      HAL_TIM_Base_Start_IT(&htim2);
      return;
    }
    dir_flag = !dir_flag; // Direction change
    driveGate(dir_flag?cw_table[curStep]:ccw_table[curStep]);
    go = 0;
  }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch(GPIO_Pin) {
    case GPIO_PIN_13:
      HAL_TIM_Base_Start_IT(&htim2);  // 디바운싱용 TIM2 타이머 시작
      break;
  }
}
