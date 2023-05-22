/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOSE 1
#define OPEN 0

#define FAIL_LIMIT 3
#define ERROR 1
#define CLEAR 0

#define G_sw 0
#define W_sw 1
#define LIM_sw 2
#define VALV_sw 3
#define VACIO_sw 4

#define ON 1
#define OFF 0
#define HEAT 1
#define COOL 2
#define HEAT_VENT 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define G HAL_GPIO_ReadPin(G_GPIO_Port, G_Pin)
#define W HAL_GPIO_ReadPin(W_GPIO_Port, W_Pin)
#define LIMITE HAL_GPIO_ReadPin(LIMITE_GPIO_Port, LIMITE_Pin)
#define VALV HAL_GPIO_ReadPin(VALV_GPIO_Port, VALV_Pin)
#define VACIO HAL_GPIO_ReadPin(VACIO_GPIO_Port, VACIO_Pin)

#define FAN_OFF HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, OFF)
#define FAN_ON HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, ON)
#define FAN_STATUS HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin)

#define EXTGAS_OFF HAL_GPIO_WritePin(EXTGAS_GPIO_Port, EXTGAS_Pin, OFF)
#define EXTGAS_ON HAL_GPIO_WritePin(EXTGAS_GPIO_Port, EXTGAS_Pin, ON)
#define EXTGAS_STATUS HAL_GPIO_ReadPin(EXTGAS_GPIO_Port, EXTGAS_Pin)

#define LED_GREEN_OFF HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, OFF)
#define LED_GREEN_ON HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, ON)
#define LED_GREEN_TOGGLE HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_GREEN_STATUS HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)

#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, OFF)
#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, ON)
#define LED_RED_TOGGLE HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_RED_STATUS HAL_GPIO_ReadPin(LED_RED_GPIO_Port, LED_RED_Pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint16_t proc_heat = 1, proc_cool = 1;
uint16_t sys = 0, cont = 0, cont_c = 0, hab_timer = 0, hab_timer_c = 0;
uint16_t fail_times = 0, fail_state = 0;
uint16_t state_vars[5] = {0,0,1,0,0}; // Variables used
uint16_t input_state[5] = {0,0,1,0,0}, count_vars[5] = {0,0,0,0,0};

// sys: 0 - OFF
//	    1 - HEAT
//      2 - COOL
//		3 - HEAT + Fixed FAN

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void input_chk(void);        // Read inputs to work with '_sw' variables
void proc_state_vars(void);  // Process state variables
void main_sys(void);         // FSM
void indicator_update(void); // Sets LED outputs
void heat_process(void);	 // HEAT sequence
void cool_process(void);	 // COOL sequence
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	proc_state_vars();
	main_sys();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3200 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_GREEN_Pin|FAN_Pin|EXTGAS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VALV_Pin LIMITE_Pin G_Pin W_Pin
                           VACIO_Pin */
  GPIO_InitStruct.Pin = VALV_Pin|LIMITE_Pin|G_Pin|W_Pin
                          |VACIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin FAN_Pin EXTGAS_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|FAN_Pin|EXTGAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16)
	{
		// Elapsed 0.5s
		if(hab_timer == 1){ // General counter
			cont++;
		}else{
			cont = 0;
		}

		if(hab_timer_c == 1){ // General counter
			cont_c++;
		}else{
			cont_c = 0;
		}

		input_chk();
		indicator_update();
	}
}

void input_chk(void)
{
	uint16_t i = 0;
	// Read actual input states
	input_state[0] = G;
	input_state[1] = W;
	input_state[2] = LIMITE;
	input_state[3] = VALV;
	input_state[4] = VACIO;
	// Process new states
	for(i = 0; i < 6; i++){
		if(input_state[i] != state_vars[i]){
			if((input_state[i] == CLOSE)&&(state_vars[i] == OPEN)){
				count_vars[i]++;
				if(count_vars[i] == 6){ // 3s
					state_vars[i] = CLOSE;
				}
			}
			else if((input_state[i] == OPEN)&&(state_vars[i] == CLOSE)){
				count_vars[i]++;
				if(count_vars[i] == 6){ // 3s
					state_vars[i] = OPEN;
				}
			}
		}else{
			if(count_vars[i] != 0) count_vars[i] = 0;
		}
	}

}

void proc_state_vars(void){
	if(state_vars[LIM_sw] == CLOSE){ // Normal operation
		if(sys == OFF){
			if(state_vars[G_sw] == CLOSE){
				sys = COOL;
			}else if(state_vars[W_sw] == CLOSE){
				sys = HEAT;
			}
		}

		if(fail_state == ERROR){
			fail_state = CLEAR;
			if(state_vars[G_sw] == OPEN){
				FAN_OFF;
			}
			proc_heat = 1;
		}
	}
	if(state_vars[LIM_sw] == OPEN){ // Limit OPEN
		if(fail_state == CLEAR){
			fail_state = ERROR;
			fail_times++;
			if(fail_times >= FAIL_LIMIT){
				sys = HEAT;
				hab_timer = 0;
				cont = 0;
				proc_heat = 5;
				EXTGAS_OFF;
				FAN_ON;
				while(1){
					// Endless block
				}
			}
		}
		if((sys != HEAT)||(proc_heat != 5)){
			sys = HEAT;
			hab_timer = 0;
			cont = 0;
			hab_timer_c = 0;
			cont_c = 0;
			proc_cool = 1;
			proc_heat = 5;
			EXTGAS_OFF;
			FAN_ON;
		}
		if(EXTGAS_STATUS == ON){
			EXTGAS_OFF;
		}
	}
}

void main_sys(void){
	switch(sys){
	case OFF:
		// System OFF
		break;
	case COOL:
		cool_process();
		break;
	case HEAT:
		heat_process();
		break;
	case HEAT_VENT:
		cool_process();
		heat_process();
	}
}

void indicator_update(void){
	if(fail_times >= FAIL_LIMIT){
		LED_GREEN_OFF;
		LED_RED_ON;
	}
	else{
		if(state_vars[LIM_sw] == CLOSE){
			if((sys == HEAT)&&(proc_heat == 3)){
				LED_RED_OFF;
				LED_GREEN_TOGGLE;
			}else if((sys == HEAT)&&(proc_heat == 1)&&(state_vars[VACIO_sw] == CLOSE)){
				if(LED_GREEN_STATUS == LED_RED_STATUS){
					LED_GREEN_OFF;
					LED_RED_ON;
				}else{
					LED_GREEN_TOGGLE;
					LED_RED_TOGGLE;
				}
			}else{
				LED_RED_OFF;
				if(LED_GREEN_STATUS == OFF){
					LED_GREEN_ON;
				}
			}
		}else if(state_vars[LIM_sw] == OPEN){
			LED_GREEN_OFF;
			LED_RED_TOGGLE;
		}
	}
}

void cool_process(void){
	switch(proc_cool){
	case 1:
		hab_timer_c = 1;
		cont_c = 0;
		proc_cool = 2;
		break;
	case 2:
		if((cont_c == 20)||(FAN_STATUS == ON)){ // 10s
			FAN_ON;
			hab_timer_c = 0;
			cont_c = 0;
			proc_cool = 3;
		}
		break;
	case 3:
		// COOL ON
		break;
	}
	if(sys != OFF){
		if(sys == COOL){
			if(state_vars[G_sw] == OPEN){
				hab_timer_c = 0;
				cont_c = 0;
				proc_cool = 1;
				FAN_OFF;
				sys = OFF;
			}else if(state_vars[W_sw] == CLOSE){
				sys = HEAT_VENT;
			}
		}else if(sys == HEAT_VENT){
			if(state_vars[G_sw] == OPEN){
				hab_timer_c = 0;
				cont_c = 0;
				proc_cool = 1;
				sys = HEAT;
				if(proc_heat < 5){
					FAN_OFF;
				}
			}else if(state_vars[W_sw] == OPEN){
				if(EXTGAS_STATUS == OFF){
					hab_timer = 0;
					cont = 0;
					proc_heat = 1;
					sys = COOL;
				}
			}
		}
	}
}

void heat_process(void){
	switch(proc_heat){
	case 1:
		if(state_vars[VACIO_sw] == OPEN){ // Checkeo sensor de vacio
			hab_timer = 1;
			cont = 0;
			proc_heat = 2;
		}
		break;
	case 2:
		if(state_vars[VACIO_sw] == CLOSE){ // Checkeo sensor de vacio
			hab_timer = 0;
			cont = 0;
			proc_heat = 1;
		}
		if(cont == 20){ // 10 segundos
			EXTGAS_ON;
			hab_timer = 0;
			cont = 0;
			proc_heat = 3;
		}
		break;
	case 3:
		if(state_vars[VALV_sw] == CLOSE){ // Espera valvula
			proc_heat = 4;
			hab_timer = 1;
			cont = 0;
		}
		break;
	case 4:
		if((cont == 40)||(FAN_STATUS == ON)){ // 20 segundos
			FAN_ON;
			hab_timer = 0;
			cont = 0;
			proc_heat = 5;
		}
		break;
	case 5:
		// Funcionamiento
		break;
	case 6:
		if((cont == 40)||(EXTGAS_STATUS == OFF)){ // 20 segundos
			EXTGAS_OFF;
			cont = 0;
			proc_heat = 7;
		}
		break;
	case 7:
		if((cont == 310)||(FAN_STATUS == OFF)){ // 2 min 35 seg
			FAN_OFF;
			hab_timer = 0;
			cont = 0;
			sys = OFF;
			proc_heat = 1;
		}
		break;
	}

	if(sys != OFF){
		if((state_vars[W_sw] == OPEN)&&(proc_heat <= 5)&&(state_vars[LIM_sw] == CLOSE)){ // Corta señal
			proc_heat = 6;
			hab_timer = 1;
			cont = 0;
		}
		if((state_vars[W_sw] == CLOSE)&&(proc_heat > 5)){ // Vuelve señal antes de terminar anterior
			hab_timer = 0;
			cont = 0;
			if(EXTGAS_STATUS == ON){
				proc_heat = 5;
			}else{
				proc_heat = 1;
			}
		}
		if((state_vars[G_sw] == CLOSE)&&(state_vars[LIM_sw] == CLOSE)){
			sys = HEAT_VENT;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
